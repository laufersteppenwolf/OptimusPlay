/*
 * arch/arm/mach-tegra/cpu-tegra3.c
 *
 * CPU auto-hotplug for Tegra3 CPUs
 *
 * Copyright (c) 2011-2012, NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/cpu.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/pm_qos_params.h>

#include "pm.h"
#include "cpu-tegra.h"
#include "clock.h"

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#define INITIAL_STATE		TEGRA_HP_DISABLED
#define UP2G0_DELAY_MS		70
#define UP2Gn_DELAY_MS		100
//                    
#ifdef CONFIG_MACH_X3
#define DOWN_DELAY_MS		1000
#else
#define DOWN_DELAY_MS		2000
#endif

/* Control flags */
unsigned char flags;
#define EARLYSUSPEND_ACTIVE	(1 << 3)
static int cpusallowed = 2;

static struct mutex *tegra3_cpu_lock;

static struct workqueue_struct *hotplug_wq;
static struct delayed_work hotplug_work;
#ifdef CONFIG_MACH_X3
static int threads_count_hotplug_control_enable = 1;
#endif
static bool no_lp;
module_param(no_lp, bool, 0644);

static unsigned long up2gn_delay;
static unsigned long up2g0_delay;
static unsigned long down_delay;
module_param(up2gn_delay, ulong, 0644);
module_param(up2g0_delay, ulong, 0644);
module_param(down_delay, ulong, 0644);

static unsigned int idle_top_freq;
static unsigned int idle_bottom_freq;
module_param(idle_top_freq, uint, 0644);
module_param(idle_bottom_freq, uint, 0644);

static int mp_overhead = 10;
module_param(mp_overhead, int, 0644);

//                    
#ifdef CONFIG_MACH_X3
static int balance_level = 75;
#else
static int balance_level = 60;
#endif

module_param(balance_level, int, 0644);

static struct clk *cpu_clk;
static struct clk *cpu_g_clk;
static struct clk *cpu_lp_clk;

static unsigned long last_change_time;

static struct {
	cputime64_t time_up_total;
	u64 last_update;
	unsigned int up_down_count;
} hp_stats[CONFIG_NR_CPUS + 1];	/* Append LP CPU entry at the end */

static void hp_init_stats(void)
{
	int i;
	u64 cur_jiffies = get_jiffies_64();

	for (i = 0; i <= CONFIG_NR_CPUS; i++) {
		hp_stats[i].time_up_total = 0;
		hp_stats[i].last_update = cur_jiffies;

		hp_stats[i].up_down_count = 0;
		if (is_lp_cluster()) {
			if (i == CONFIG_NR_CPUS)
				hp_stats[i].up_down_count = 1;
		} else {
			if ((i < nr_cpu_ids) && cpu_online(i))
				hp_stats[i].up_down_count = 1;
		}
	}

}

static void hp_stats_update(unsigned int cpu, bool up)
{
	u64 cur_jiffies = get_jiffies_64();
	bool was_up = hp_stats[cpu].up_down_count & 0x1;

	if (was_up)
		hp_stats[cpu].time_up_total = cputime64_add(
			hp_stats[cpu].time_up_total, cputime64_sub(
				cur_jiffies, hp_stats[cpu].last_update));

	if (was_up != up) {
		hp_stats[cpu].up_down_count++;
		if ((hp_stats[cpu].up_down_count & 0x1) != up) {
			/* FIXME: sysfs user space CPU control breaks stats */
			pr_err("tegra hotplug stats out of sync with %s CPU%d",
			       (cpu < CONFIG_NR_CPUS) ? "G" : "LP",
			       (cpu < CONFIG_NR_CPUS) ?  cpu : 0);
			hp_stats[cpu].up_down_count ^=  0x1;
		}
	}
	hp_stats[cpu].last_update = cur_jiffies;
}


enum {
	TEGRA_HP_DISABLED = 0,
	TEGRA_HP_IDLE,
	TEGRA_HP_DOWN,
	TEGRA_HP_UP,
};
static int hp_state;

static int hp_state_set(const char *arg, const struct kernel_param *kp)
{
	int ret = 0;
	int old_state;

	if (!tegra3_cpu_lock)
		return ret;

	mutex_lock(tegra3_cpu_lock);

	old_state = hp_state;
	ret = param_set_bool(arg, kp);	/* set idle or disabled only */

	if (ret == 0) {
		if ((hp_state == TEGRA_HP_DISABLED) &&
		    (old_state != TEGRA_HP_DISABLED)) {
			mutex_unlock(tegra3_cpu_lock);
			cancel_delayed_work_sync(&hotplug_work);
			mutex_lock(tegra3_cpu_lock);
			pr_info("Tegra auto-hotplug disabled\n");
		} else if (hp_state != TEGRA_HP_DISABLED) {
			if (old_state == TEGRA_HP_DISABLED) {
				pr_info("Tegra auto-hotplug enabled\n");
				hp_init_stats();
			}
			/* catch-up with governor target speed */
			tegra_cpu_set_speed_cap(NULL);
		}
	} else
		pr_warn("%s: unable to set tegra hotplug state %s\n",
				__func__, arg);

	mutex_unlock(tegra3_cpu_lock);
	return ret;
}

static int hp_state_get(char *buffer, const struct kernel_param *kp)
{
	return param_get_int(buffer, kp);
}

static struct kernel_param_ops tegra_hp_state_ops = {
	.set = hp_state_set,
	.get = hp_state_get,
};
module_param_cb(auto_hotplug, &tegra_hp_state_ops, &hp_state, 0644);


enum {
	TEGRA_CPU_SPEED_BALANCED,
	TEGRA_CPU_SPEED_BIASED,
	TEGRA_CPU_SPEED_SKEWED,
};

#define NR_FSHIFT	2

static unsigned int rt_profile_sel;

/* avg run threads * 4 (e.g., 9 = 2.25 threads) */

static unsigned int rt_profile_default[] = {
/*      1,  2,  3,  4 - on-line cpus target */
/*                                                                              */
	10,  11, 12, UINT_MAX
//	5,  9, 10, UINT_MAX
};

static unsigned int rt_profile_1[] = {
/*      1,  2,  3,  4 - on-line cpus target */
	8,  9, 10, UINT_MAX
};

static unsigned int rt_profile_2[] = {
/*      1,  2,  3,  4 - on-line cpus target */
	5,  13, 14, UINT_MAX
};

static unsigned int rt_profile_off[] = { /* disables runable thread */
	0,  0,  0, UINT_MAX
};

static unsigned int *rt_profiles[] = {
	rt_profile_default,
	rt_profile_1,
	rt_profile_2,
	rt_profile_off
};


static unsigned int nr_run_hysteresis = 2;	/* 0.5 thread */
static unsigned int nr_run_last;

static noinline int tegra_cpu_speed_balance(void)
{
	unsigned long highest_speed = tegra_cpu_highest_speed();
	unsigned long balanced_speed = highest_speed * balance_level / 100;
	unsigned long skewed_speed = balanced_speed / 2;
	unsigned int nr_cpus = num_online_cpus();
	unsigned int max_cpus = pm_qos_request(PM_QOS_MAX_ONLINE_CPUS) ? : 4;
	unsigned int min_cpus = pm_qos_request(PM_QOS_MIN_ONLINE_CPUS);
	unsigned int avg_nr_run = avg_nr_running();
	unsigned int nr_run;

	/* Evaluate:
	 * - distribution of freq targets for already on-lined CPUs
	 * - average number of runnable threads
	 * - effective MIPS available within EDP frequency limits,
	 * and return:
	 * TEGRA_CPU_SPEED_BALANCED to bring one more CPU core on-line
	 * TEGRA_CPU_SPEED_BIASED to keep CPU core composition unchanged
	 * TEGRA_CPU_SPEED_SKEWED to remove CPU core off-line
	 */

	unsigned int *current_profile = rt_profiles[rt_profile_sel];
	for (nr_run = 1; nr_run < ARRAY_SIZE(rt_profile_default); nr_run++) {
		unsigned int nr_threshold = current_profile[nr_run - 1];
		if (nr_run_last <= nr_run)
			nr_threshold += nr_run_hysteresis;
		if (avg_nr_run <= (nr_threshold << (FSHIFT - NR_FSHIFT)))
			break;
	}
	nr_run_last = nr_run;

//                           
#ifdef CONFIG_MACH_X3
	if(threads_count_hotplug_control_enable == 0 && highest_speed >= 640000 )
		nr_run++;
#endif

	if (((tegra_count_slow_cpus(skewed_speed) >= 2) ||
	     (nr_run < nr_cpus) ||
	     tegra_cpu_edp_favor_down(nr_cpus, mp_overhead) ||
	     (highest_speed <= idle_bottom_freq) || (nr_cpus > max_cpus)) &&
	    (nr_cpus > min_cpus))
		return TEGRA_CPU_SPEED_SKEWED;

	if (((tegra_count_slow_cpus(balanced_speed) >= 1) ||
	     (nr_run <= nr_cpus) ||
	     (!tegra_cpu_edp_favor_up(nr_cpus, mp_overhead)) ||
	     (highest_speed <= idle_bottom_freq) || (nr_cpus == max_cpus)) &&
	    (nr_cpus >= min_cpus))
		return TEGRA_CPU_SPEED_BIASED;

	return TEGRA_CPU_SPEED_BALANCED;
}

//                                                               
#if defined(CONFIG_MACH_LGE)
static void tegra_check_limeted_max_cores(void)
{
	if(is_lp_cluster())
	{
		if(cpufreq_limited_max_cores_cur != cpufreq_limited_max_cores_expected)
		{
			switch(cpufreq_limited_max_cores_expected)
			{
				case 1:
					set_cpu_present(1,false);
					set_cpu_present(2,false);
					set_cpu_present(3,false);
					set_cpu_possible(1,false);
					set_cpu_possible(2,false);
					set_cpu_possible(3,false);
					cpufreq_limited_max_cores_cur = cpufreq_limited_max_cores_expected;
					break;
				case 2:
					set_cpu_present(1,true);
					set_cpu_present(2,false);
					set_cpu_present(3,false);
					set_cpu_possible(1,true);
					set_cpu_possible(2,false);
					set_cpu_possible(3,false);
					cpufreq_limited_max_cores_cur = cpufreq_limited_max_cores_expected;
					break;
				case 4:
					set_cpu_present(1,true);
					set_cpu_present(2,true);
					set_cpu_present(3,true);
					set_cpu_possible(1,true);
					set_cpu_possible(2,true);
					set_cpu_possible(3,true);
					cpufreq_limited_max_cores_cur = cpufreq_limited_max_cores_expected;
					break;
				default:
					cpufreq_limited_max_cores_expected = cpufreq_limited_max_cores_cur;
					break;
			}
		}
	}

}
#endif
//                                                               

static void tegra_auto_hotplug_work_func(struct work_struct *work)
{
	bool up = false;
	unsigned int cpu = nr_cpu_ids;
	unsigned long now = jiffies;

	mutex_lock(tegra3_cpu_lock);
	
	//                                                               
#if defined(CONFIG_MACH_LGE)	
	tegra_check_limeted_max_cores();
#endif	
	//                                                               

	switch (hp_state) {
	case TEGRA_HP_DISABLED:
	case TEGRA_HP_IDLE:
		break;
	case TEGRA_HP_DOWN:
//                               
#ifdef CONFIG_MACH_X3
		if ((now - last_change_time) < down_delay) {
			queue_delayed_work(
				hotplug_wq, &hotplug_work, up2gn_delay);
			break;
		}
#endif
		cpu = tegra_get_slowest_cpu_n();
		if (cpu < nr_cpu_ids) {
			up = false;
		} else if (!is_lp_cluster() && !no_lp &&
			   !pm_qos_request(PM_QOS_MIN_ONLINE_CPUS) &&
			   ((now - last_change_time) >= down_delay) &&
			   (flags & EARLYSUSPEND_ACTIVE)) {
			if(!clk_set_parent(cpu_clk, cpu_lp_clk)) {
				hp_stats_update(CONFIG_NR_CPUS, true);
				hp_stats_update(0, false);
				/* catch-up with governor target speed */
				tegra_cpu_set_speed_cap(NULL);
				break;
			}
		}
		queue_delayed_work(
			hotplug_wq, &hotplug_work, up2gn_delay);
		break;
	case TEGRA_HP_UP:
		if (is_lp_cluster() && !no_lp) {
			if(!clk_set_parent(cpu_clk, cpu_g_clk)) {
				last_change_time = now;
				hp_stats_update(CONFIG_NR_CPUS, false);
				hp_stats_update(0, true);
				/* catch-up with governor target speed */
				tegra_cpu_set_speed_cap(NULL);
			}
		} else {
			switch (tegra_cpu_speed_balance()) {
			/* cpu speed is up and balanced - one more on-line */
			case TEGRA_CPU_SPEED_BALANCED:
				cpu = cpumask_next_zero(0, cpu_online_mask);
				if (cpu < nr_cpu_ids)
					up = true;
				break;
			/* cpu speed is up, but skewed - remove one core */
			case TEGRA_CPU_SPEED_SKEWED:
				cpu = tegra_get_slowest_cpu_n();
				if (cpu < nr_cpu_ids)
					up = false;
				break;
			/* cpu speed is up, but under-utilized - do nothing */
			case TEGRA_CPU_SPEED_BIASED:
			default:
				break;
			}
		}
		queue_delayed_work(
			hotplug_wq, &hotplug_work, up2gn_delay);
		break;
	default:
		pr_err("%s: invalid tegra hotplug state %d\n",
		       __func__, hp_state);
	}

	if (!up && ((now - last_change_time) < down_delay))
			cpu = nr_cpu_ids;

	if (cpu < nr_cpu_ids) {
		last_change_time = now;
		hp_stats_update(cpu, up);
	}
	mutex_unlock(tegra3_cpu_lock);
	if (system_state > SYSTEM_RUNNING) {
		CPU_DEBUG_PRINTK(CPU_DEBUG_HOTPLUG, " system is not running\n");
	} else if (cpu < nr_cpu_ids) {
		if (up) {
			if ((num_online_cpus() < cpusallowed) && (!(flags & EARLYSUSPEND_ACTIVE)))
				cpu_up(cpu);
		} else {
			if ((num_online_cpus() > cpusallowed) || (flags & EARLYSUSPEND_ACTIVE))
				cpu_down(cpu);
		}
	}
}

#if defined(CONFIG_BEST_TRADE_HOTPLUG)
extern unsigned int best_core_to_turn_up (void);

unsigned int g2lp_bottom_freq (void) {
    return idle_bottom_freq;
}
EXPORT_SYMBOL (g2lp_bottom_freq);

unsigned int lp2g_top_freq (void) {
    return idle_top_freq;
}
EXPORT_SYMBOL (lp2g_top_freq);

bool on_plugging (void) {
    return ACCESS_ONCE (is_plugging);
}
EXPORT_SYMBOL (on_plugging);

bool can_do_bthp (void) {
    return (!ACCESS_ONCE (is_plugging) && hp_state != TEGRA_HP_DISABLED);
}
EXPORT_SYMBOL (can_do_bthp);

static void init_bthp_params (void) {
    bthp_wq_params.core_num_diff = 0;
    bthp_wq_params.dest_core = NR_CPUS;
    bthp_wq_params.ref_speeds.prev_loc = NULL;
    bthp_wq_params.ref_speeds.prev = 0UL;
    bthp_wq_params.ref_speeds.next = 0UL;

	if (likely(bthp_wq_params.fallback.fallback_time_loc)) {
		*bthp_wq_params.fallback.fallback_time_loc =
            bthp_wq_params.fallback.fallback_time;
	}
}

static int min_cpus_notify(struct notifier_block *nb, unsigned long n, void *p)
{
	mutex_lock(tegra3_cpu_lock);

	if ((n >= 1) && is_lp_cluster() && !no_lp) {
		/* make sure cpu rate is within g-mode range before switching */
		unsigned int speed = max((unsigned long)tegra_getspeed(0),
			clk_get_min_rate(cpu_g_clk) / 1000);
		tegra_update_cpu_speed(speed);

		if (!clk_set_parent(cpu_clk, cpu_g_clk)) {
			last_change_time = jiffies;
			hp_stats_update(CONFIG_NR_CPUS, false);
			hp_stats_update(0, true);
		}
	}
	/* update governor state machine */
	tegra_cpu_set_speed_cap(NULL);
	mutex_unlock(tegra3_cpu_lock);
	return NOTIFY_OK;
}

static struct notifier_block min_cpus_notifier = {
	.notifier_call = min_cpus_notify,
};
#ifdef CONFIG_MACH_X3
int tegra_get_threads_count_hotplug_control()
{	
	return threads_count_hotplug_control_enable;
}

void tegra_set_threads_count_hotplug_control(int enable)
{	
	threads_count_hotplug_control_enable = enable;
}
#endif
void tegra_auto_hotplug_governor(unsigned int cpu_freq, bool suspend)
{
	unsigned long up_delay, top_freq, bottom_freq;

	if (!is_g_cluster_present())
		return;

	if (hp_state == TEGRA_HP_DISABLED)
		return;

	if (suspend) {
		hp_state = TEGRA_HP_IDLE;

		/* Switch to G-mode if suspend rate is high enough */
		if (is_lp_cluster() && (cpu_freq >= idle_bottom_freq)) {
			if (!clk_set_parent(cpu_clk, cpu_g_clk)) {
				hp_stats_update(CONFIG_NR_CPUS, false);
				hp_stats_update(0, true);
			}
		}
		return;
	}

	if (is_lp_cluster()) {
		up_delay = up2g0_delay;
		top_freq = idle_top_freq;
		bottom_freq = 0;
	} else {
		up_delay = up2gn_delay;
		top_freq = idle_bottom_freq;
		bottom_freq = idle_bottom_freq;
	}

	if (pm_qos_request(PM_QOS_MIN_ONLINE_CPUS) >= 2) {
		if (hp_state != TEGRA_HP_UP) {
			hp_state = TEGRA_HP_UP;
			queue_delayed_work(
				hotplug_wq, &hotplug_work, up_delay);
		}
		return;
	}

	switch (hp_state) {
	case TEGRA_HP_IDLE:
		if (cpu_freq > top_freq) {
			hp_state = TEGRA_HP_UP;
			queue_delayed_work(
				hotplug_wq, &hotplug_work, up_delay);
		} else if (cpu_freq <= bottom_freq) {
			hp_state = TEGRA_HP_DOWN;
			queue_delayed_work(
				hotplug_wq, &hotplug_work, up_delay);
		}
		break;
	case TEGRA_HP_DOWN:
		if (cpu_freq > top_freq) {
			hp_state = TEGRA_HP_UP;
			queue_delayed_work(
				hotplug_wq, &hotplug_work, up_delay);
		} else if (cpu_freq > bottom_freq) {
			hp_state = TEGRA_HP_IDLE;
		}
		break;
	case TEGRA_HP_UP:
		if (cpu_freq <= bottom_freq) {
			hp_state = TEGRA_HP_DOWN;
			queue_delayed_work(
				hotplug_wq, &hotplug_work, up_delay);
		} else if (cpu_freq <= top_freq) {
			hp_state = TEGRA_HP_IDLE;
		}
		break;
	default:
		pr_err("%s: invalid tegra hotplug state %d\n",
		       __func__, hp_state);
		BUG();
	}
}

int tegra_auto_hotplug_init(struct mutex *cpu_lock)
{
	/*
	 * Not bound to the issuer CPU (=> high-priority), has rescue worker
	 * task, single-threaded, freezable.
	 */
	hotplug_wq = alloc_workqueue(
		"cpu-tegra3", WQ_UNBOUND | WQ_RESCUER | WQ_FREEZABLE, 1);
	if (!hotplug_wq)
		return -ENOMEM;
	INIT_DELAYED_WORK(&hotplug_work, tegra_auto_hotplug_work_func);

	cpu_clk = clk_get_sys(NULL, "cpu");
	cpu_g_clk = clk_get_sys(NULL, "cpu_g");
	cpu_lp_clk = clk_get_sys(NULL, "cpu_lp");
	if (IS_ERR(cpu_clk) || IS_ERR(cpu_g_clk) || IS_ERR(cpu_lp_clk))
		return -ENOENT;

	idle_top_freq = clk_get_max_rate(cpu_lp_clk) / 1000;
	idle_bottom_freq = clk_get_min_rate(cpu_g_clk) / 1000;

	up2g0_delay = msecs_to_jiffies(UP2G0_DELAY_MS);
	up2gn_delay = msecs_to_jiffies(UP2Gn_DELAY_MS);
	down_delay = msecs_to_jiffies(DOWN_DELAY_MS);

	tegra3_cpu_lock = cpu_lock;
	hp_state = INITIAL_STATE;
	hp_init_stats();
	pr_info("Tegra auto-hotplug initialized: %s\n",
		(hp_state == TEGRA_HP_DISABLED) ? "disabled" : "enabled");

	if (pm_qos_add_notifier(PM_QOS_MIN_ONLINE_CPUS, &min_cpus_notifier))
		pr_err("%s: Failed to register min cpus PM QoS notifier\n",
			__func__);

	return 0;
}

#ifdef CONFIG_DEBUG_FS

static struct dentry *hp_debugfs_root;

struct pm_qos_request_list min_cpu_req;
struct pm_qos_request_list max_cpu_req;

static int hp_stats_show(struct seq_file *s, void *data)
{
	int i;
	u64 cur_jiffies = get_jiffies_64();

	mutex_lock(tegra3_cpu_lock);
	if (hp_state != TEGRA_HP_DISABLED) {
		for (i = 0; i <= CONFIG_NR_CPUS; i++) {
			bool was_up = (hp_stats[i].up_down_count & 0x1);
			hp_stats_update(i, was_up);
		}
	}
	mutex_unlock(tegra3_cpu_lock);

	seq_printf(s, "%-15s ", "cpu:");
	for (i = 0; i < CONFIG_NR_CPUS; i++) {
		seq_printf(s, "G%-9d ", i);
	}
	seq_printf(s, "LP\n");

	seq_printf(s, "%-15s ", "transitions:");
	for (i = 0; i <= CONFIG_NR_CPUS; i++) {
		seq_printf(s, "%-10u ", hp_stats[i].up_down_count);
	}
	seq_printf(s, "\n");

	seq_printf(s, "%-15s ", "time plugged:");
	for (i = 0; i <= CONFIG_NR_CPUS; i++) {
		seq_printf(s, "%-10llu ",
			   cputime64_to_clock_t(hp_stats[i].time_up_total));
	}
	seq_printf(s, "\n");

	seq_printf(s, "%-15s %llu\n", "time-stamp:",
		   cputime64_to_clock_t(cur_jiffies));

	return 0;
}

static int hp_stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, hp_stats_show, inode->i_private);
}

static const struct file_operations hp_stats_fops = {
	.open		= hp_stats_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int rt_bias_get(void *data, u64 *val)
{
	*val = rt_profile_sel;
	return 0;
}
static int rt_bias_set(void *data, u64 val)
{
	if (val < ARRAY_SIZE(rt_profiles))
		rt_profile_sel = (u32)val;

	pr_debug("rt_profile_sel set to %d\nthresholds are now [%d, %d, %d]\n",
		rt_profile_sel,
		rt_profiles[rt_profile_sel][0],
		rt_profiles[rt_profile_sel][1],
		rt_profiles[rt_profile_sel][2]);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(rt_bias_fops, rt_bias_get, rt_bias_set, "%llu\n");

static int min_cpus_get(void *data, u64 *val)
{
	*val = pm_qos_request(PM_QOS_MIN_ONLINE_CPUS);
	return 0;
}
static int min_cpus_set(void *data, u64 val)
{
	pm_qos_update_request(&min_cpu_req, (s32)val);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(min_cpus_fops, min_cpus_get, min_cpus_set, "%llu\n");

static int max_cpus_get(void *data, u64 *val)
{
	*val = pm_qos_request(PM_QOS_MAX_ONLINE_CPUS);
	return 0;
}
static int max_cpus_set(void *data, u64 val)
{
	pm_qos_update_request(&max_cpu_req, (s32)val);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(max_cpus_fops, max_cpus_get, max_cpus_set, "%llu\n");
//                                                                          
void tegra_auto_hotplug_set_min_cpus(int num_cpus)
{
	if (num_cpus < 0) {
		pr_err("%s: invalid num_cpus=%d\n", __func__, num_cpus);
		return;
	}
	min_cpus_set(NULL, num_cpus);
}
//                                                                          

void tegra_auto_hotplug_set_max_cpus(int num_cpus)
{
	if (num_cpus < 0) {
		pr_err("%s: invalid num_cpus=%d\n", __func__, num_cpus);
		return;
	}
	max_cpus_set(NULL, num_cpus);
}

static struct kernel_param_ops at_ctrl_ops = {
	.set = at_ctrl_set,
	.get = at_ctrl_get,
};
module_param_cb(at_en, &at_ctrl_ops, &at_en, 0664);

static ssize_t cpusallowed_status_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	int available_cpus = sprintf(buf,"%u\n", cpusallowed);

	if (available_cpus > 4) available_cpus = 4;
	if (available_cpus < 1) available_cpus = 1;
	return available_cpus;
}

static ssize_t cpusallowed_status_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int data;

	if (sscanf(buf, "%u\n", &data) == 1)
		cpusallowed = data;
	else
		pr_info("%s: input error\n", __FUNCTION__);

	return size;
}

static DEVICE_ATTR(cpusallowed, S_IRUGO | S_IWUGO, cpusallowed_status_read, cpusallowed_status_write);

static struct attribute *cpusallowed_attributes[] = {
	&dev_attr_cpusallowed.attr,
	NULL
};

static struct attribute_group cpusallowed_group = {
	.attrs  = cpusallowed_attributes,
};

static struct miscdevice cpusallowed_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "cpusallowed",
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void auto_hotplug_early_suspend(struct early_suspend *handler)
{
	pr_info("auto_hotplug: early suspend handler\n");
	flags |= EARLYSUSPEND_ACTIVE;
}

static void auto_hotplug_late_resume(struct early_suspend *handler)
{
	pr_info("auto_hotplug: late resume handler\n");
	flags &= ~EARLYSUSPEND_ACTIVE;
}

static struct early_suspend auto_hotplug_suspend = {
	.suspend = auto_hotplug_early_suspend,
	.resume = auto_hotplug_late_resume,
};
#endif /* CONFIG_HAS_EARLYSUSPEND */

static int __init tegra_auto_hotplug_debug_init(void)
{
	int ret;

	if (!tegra3_cpu_lock)
		return -ENOENT;

	hp_debugfs_root = debugfs_create_dir("tegra_hotplug", NULL);
	if (!hp_debugfs_root)
		return -ENOMEM;

	pm_qos_add_request(&min_cpu_req, PM_QOS_MIN_ONLINE_CPUS,
			   PM_QOS_DEFAULT_VALUE);
	pm_qos_add_request(&max_cpu_req, PM_QOS_MAX_ONLINE_CPUS,
			   PM_QOS_DEFAULT_VALUE);

	// sysfs interface
	pr_info("%s misc_register(%s)\n", __FUNCTION__, cpusallowed_device.name);
	ret = misc_register(&cpusallowed_device);
	if (ret) {
		pr_err("%s misc_register(%s) fail\n", __FUNCTION__,
				cpusallowed_device.name);
		return 1;
	}
	if (sysfs_create_group(&cpusallowed_device.this_device->kobj,
				&cpusallowed_group) < 0) {
		pr_err("%s sysfs_create_group fail\n", __FUNCTION__);
		pr_err("Failed to create sysfs group for device (%s)!\n",
				cpusallowed_device.name);
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend(&auto_hotplug_suspend);
#endif

	if (!debugfs_create_file(
		"min_cpus", S_IRUGO, hp_debugfs_root, NULL, &min_cpus_fops))
		goto err_out;

	if (!debugfs_create_file(
		"max_cpus", S_IRUGO, hp_debugfs_root, NULL, &max_cpus_fops))
		goto err_out;

	if (!debugfs_create_file(
		"stats", S_IRUGO, hp_debugfs_root, NULL, &hp_stats_fops))
		goto err_out;

	if (!debugfs_create_file(
		"core_bias", S_IRUGO, hp_debugfs_root, NULL, &rt_bias_fops))
		goto err_out;

	return 0;

err_out:
	debugfs_remove_recursive(hp_debugfs_root);
	pm_qos_remove_request(&min_cpu_req);
	pm_qos_remove_request(&max_cpu_req);
	return -ENOMEM;
}

late_initcall(tegra_auto_hotplug_debug_init);
#endif

void tegra_auto_hotplug_exit(void)
{
	destroy_workqueue(hotplug_wq);
#ifdef CONFIG_DEBUG_FS
	debugfs_remove_recursive(hp_debugfs_root);
	pm_qos_remove_request(&min_cpu_req);
	pm_qos_remove_request(&max_cpu_req);
#endif
}
