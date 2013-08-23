/*
 * arch/arm/mach-tegra/cpu-tegra.c
 *
 * Copyright (C) 2010 Google, Inc.
 *
 * Author:
 *	Colin Cross <ccross@google.com>
 *	Based on arch/arm/plat-omap/cpu-omap.c, (C) 2005 Nokia Corporation
 *
 * Copyright (C) 2010-2012 NVIDIA CORPORATION. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/suspend.h>
#include <linux/debugfs.h>
#include <linux/cpu.h>

#include <asm/system.h>

#include <mach/clk.h>
#include <mach/edp.h>

#include "clock.h"
#include "cpu-tegra.h"
#include "dvfs.h"

/* tegra throttling and edp governors require frequencies in the table
   to be in ascending order */
static struct cpufreq_frequency_table *freq_table;

static struct clk *cpu_clk;
static struct clk *emc_clk;

static unsigned long policy_max_speed[CONFIG_NR_CPUS];
static unsigned long target_cpu_speed[CONFIG_NR_CPUS];
static DEFINE_MUTEX(tegra_cpu_lock);
static bool is_suspended;
static int suspend_index;

static bool force_policy_max;

static int force_policy_max_set(const char *arg, const struct kernel_param *kp)
{
	int ret;
	bool old_policy = force_policy_max;

	mutex_lock(&tegra_cpu_lock);

	ret = param_set_bool(arg, kp);
	if ((ret == 0) && (old_policy != force_policy_max))
		tegra_cpu_set_speed_cap(NULL);

	mutex_unlock(&tegra_cpu_lock);
	return ret;
}

static int force_policy_max_get(char *buffer, const struct kernel_param *kp)
{
	return param_get_bool(buffer, kp);
}

static struct kernel_param_ops policy_ops = {
	.set = force_policy_max_set,
	.get = force_policy_max_get,
};
module_param_cb(force_policy_max, &policy_ops, &force_policy_max, 0644);

//                    
#ifdef CONFIG_MACH_X3
#define LGE_EDP_ENABLE_CONTROL 
#ifdef LGE_EDP_ENABLE_CONTROL
static bool force_disable_edp = false;

static int force_disable_edp_set(const char *arg, const struct kernel_param *kp)
{
	int ret;
	bool old_disable_edp = force_disable_edp;

	mutex_lock(&tegra_cpu_lock);

	ret = param_set_bool(arg, kp);
	if ((ret == 0) && (old_disable_edp != force_disable_edp))
		tegra_cpu_set_speed_cap(NULL);

	mutex_unlock(&tegra_cpu_lock);
	return ret;
}

static int force_disable_edp_get(char *buffer, const struct kernel_param *kp)
{
	return param_get_bool(buffer, kp);
}

static struct kernel_param_ops force_disable_edp_ops = {
	.set = force_disable_edp_set,
	.get = force_disable_edp_get,
};
module_param_cb(force_disable_edp, &force_disable_edp_ops, &force_disable_edp, 0644);
#endif

#ifdef LOAD_SHAPER_BY_VOTE_MAX_FREQ
#define MAX_BUCKET_SIZE 10000
static int token_in_bucket = MAX_BUCKET_SIZE;
static long threashold_allowed_clk = 880000;
static long threashold_limited_clk = 1000000;
static long threashold_enable_token = 2000;
static int restore_token = 5;
static bool is_enable_boost_load_shaper = true;

static long vmf_for_load_shaper = 0;
static long vmf_for_record = 0;
static long vmf_for_camera = 0;
static long vmf_for_player = 0;
static long vmf_for_network = 0;

static int threashold_allowed_clk_set(const char *arg, const struct kernel_param *kp)
{
	int ret;

	mutex_lock(&tegra_cpu_lock);
	ret = param_set_long(arg, kp);
	mutex_unlock(&tegra_cpu_lock);
	return ret;
}

static int threashold_allowed_clk_get(char *buffer, const struct kernel_param *kp)
{
	return param_get_long(buffer, kp);
}


static struct kernel_param_ops threashold_allowed_clk_ops = {
	.set = threashold_allowed_clk_set,
	.get = threashold_allowed_clk_get,
};
module_param_cb(threashold_allowed_clk, &threashold_allowed_clk_ops, &threashold_allowed_clk, 0644);


static int threashold_limited_clk_set(const char *arg, const struct kernel_param *kp)
{
	int ret;

	mutex_lock(&tegra_cpu_lock);
	ret = param_set_long(arg, kp);
	mutex_unlock(&tegra_cpu_lock);
	return ret;
}

static int threashold_limited_clk_get(char *buffer, const struct kernel_param *kp)
{
	return param_get_long(buffer, kp);
}

static struct kernel_param_ops threashold_limited_clk_ops = {
	.set = threashold_limited_clk_set,
	.get = threashold_limited_clk_get,
};
module_param_cb(threashold_limited_clk, &threashold_limited_clk_ops, &threashold_limited_clk, 0644);

static int threashold_enable_token_set(const char *arg, const struct kernel_param *kp)
{
	int ret;

	mutex_lock(&tegra_cpu_lock);
	ret = param_set_long(arg, kp);
	mutex_unlock(&tegra_cpu_lock);
	return ret;
}

static int threashold_enable_token_get(char *buffer, const struct kernel_param *kp)
{
	return param_get_long(buffer, kp);
}

static struct kernel_param_ops threashold_enable_token_ops = {
	.set = threashold_enable_token_set,
	.get = threashold_enable_token_get,
};
module_param_cb(threashold_enable_token, &threashold_enable_token_ops, &threashold_enable_token, 0644);

static int restore_token_set(const char *arg, const struct kernel_param *kp)
{
	int ret;
	
	mutex_lock(&tegra_cpu_lock);
	ret = param_set_int(arg, kp);
	mutex_unlock(&tegra_cpu_lock);
	return ret;
}

static int restore_token_get(char *buffer, const struct kernel_param *kp)
{
	return param_get_int(buffer, kp);
}

static struct kernel_param_ops restore_token_ops = {
	.set = restore_token_set,
	.get = restore_token_get,
};
module_param_cb(restore_token, &restore_token_ops, &restore_token, 0644);

static int is_enable_boost_load_shaper_set(const char *arg, const struct kernel_param *kp)
{
	int ret;

	mutex_lock(&tegra_cpu_lock);
	ret = param_set_bool(arg, kp);
	mutex_unlock(&tegra_cpu_lock);
	return ret;
}

static int is_enable_boost_load_shaper_get(char *buffer, const struct kernel_param *kp)
{
	return param_get_bool(buffer, kp);
}

static struct kernel_param_ops is_enable_boost_load_shaper_ops = {
	.set = is_enable_boost_load_shaper_set,
	.get = is_enable_boost_load_shaper_get,
};
module_param_cb(is_enable_boost_load_shaper, &is_enable_boost_load_shaper_ops, &is_enable_boost_load_shaper, 0644);

static int token_in_bucket_set(const char *arg, const struct kernel_param *kp)
{
	int ret;

	mutex_lock(&tegra_cpu_lock);
	ret = param_set_int(arg, kp);
	mutex_unlock(&tegra_cpu_lock);
	return ret;
}

static int token_in_bucket_get(char *buffer, const struct kernel_param *kp)
{
	return param_get_int(buffer, kp);
}

static struct kernel_param_ops token_in_bucket_ops = {
	.set = token_in_bucket_set,
	.get = token_in_bucket_get,
};
module_param_cb(token_in_bucket, &token_in_bucket_ops, &token_in_bucket, 0644);

static int vmf_for_load_shaper_set(const char *arg, const struct kernel_param *kp)
{
	int ret;

	mutex_lock(&tegra_cpu_lock);
	ret = param_set_long(arg, kp);
	mutex_unlock(&tegra_cpu_lock);
	return ret;
}

static int vmf_for_load_shaper_get(char *buffer, const struct kernel_param *kp)
{
	return param_get_long(buffer, kp);
}

static struct kernel_param_ops vmf_for_load_shaper_ops = {
	.set = vmf_for_load_shaper_set,
	.get = vmf_for_load_shaper_get,
};
module_param_cb(vmf_for_load_shaper, &vmf_for_load_shaper_ops, &vmf_for_load_shaper, 0644);

static int vmf_for_record_set(const char *arg, const struct kernel_param *kp)
{
	int ret;

	mutex_lock(&tegra_cpu_lock);
	ret = param_set_long(arg, kp);
	mutex_unlock(&tegra_cpu_lock);
	return ret;
}

static int vmf_for_record_get(char *buffer, const struct kernel_param *kp)
{
	return param_get_long(buffer, kp);
}

static struct kernel_param_ops vmf_for_record_ops = {
	.set = vmf_for_record_set,
	.get = vmf_for_record_get,
};
module_param_cb(vmf_for_record, &vmf_for_record_ops, &vmf_for_record, 0644);

static int vmf_for_camera_set(const char *arg, const struct kernel_param *kp)
{
	int ret;

	mutex_lock(&tegra_cpu_lock);
	ret = param_set_long(arg, kp);
	mutex_unlock(&tegra_cpu_lock);
	return ret;
}

static int vmf_for_camera_get(char *buffer, const struct kernel_param *kp)
{
	return param_get_long(buffer, kp);
}

static struct kernel_param_ops vmf_for_camera_ops = {
	.set = vmf_for_camera_set,
	.get = vmf_for_camera_get,
};
module_param_cb(vmf_for_camera, &vmf_for_camera_ops, &vmf_for_camera, 0644);

static int vmf_for_player_set(const char *arg, const struct kernel_param *kp)
{
	int ret;

	mutex_lock(&tegra_cpu_lock);
	ret = param_set_long(arg, kp);
	mutex_unlock(&tegra_cpu_lock);
	return ret;
}

static int vmf_for_player_get(char *buffer, const struct kernel_param *kp)
{
	return param_get_long(buffer, kp);
}

static struct kernel_param_ops vmf_for_player_ops = {
	.set = vmf_for_player_set,
	.get = vmf_for_player_get,
};
module_param_cb(vmf_for_player, &vmf_for_player_ops, &vmf_for_player, 0644);

static int vmf_for_network_set(const char *arg, const struct kernel_param *kp)
{
	int ret;

	mutex_lock(&tegra_cpu_lock);
	ret = param_set_long(arg, kp);
	mutex_unlock(&tegra_cpu_lock);
	return ret;
}

static int vmf_for_network_get(char *buffer, const struct kernel_param *kp)
{
	return param_get_long(buffer, kp);
}

static struct kernel_param_ops vmf_for_network_ops = {
	.set = vmf_for_network_set,
	.get = vmf_for_network_get,
};
module_param_cb(vmf_for_network, &vmf_for_network_ops, &vmf_for_network, 0644);
#endif
#endif /* CONFIG_MACH_X3 */
static unsigned int cpu_user_cap;

static inline void _cpu_user_cap_set_locked(void)
{
#ifndef CONFIG_TEGRA_CPU_CAP_EXACT_FREQ
	if (cpu_user_cap != 0) {
		int i;
		for (i = 0; freq_table[i].frequency != CPUFREQ_TABLE_END; i++) {
			if (freq_table[i].frequency > cpu_user_cap)
				break;
		}
		i = (i == 0) ? 0 : i - 1;
		cpu_user_cap = freq_table[i].frequency;
	}
#endif
	tegra_cpu_set_speed_cap(NULL);
}

void tegra_cpu_user_cap_set(unsigned int speed_khz)
{
	mutex_lock(&tegra_cpu_lock);

	cpu_user_cap = speed_khz;
	_cpu_user_cap_set_locked();

	mutex_unlock(&tegra_cpu_lock);
}

static int cpu_user_cap_set(const char *arg, const struct kernel_param *kp)
{
	int ret;

	mutex_lock(&tegra_cpu_lock);

	ret = param_set_uint(arg, kp);
	if (ret == 0)
		_cpu_user_cap_set_locked();

	mutex_unlock(&tegra_cpu_lock);
	return ret;
}

static int cpu_user_cap_get(char *buffer, const struct kernel_param *kp)
{
	return param_get_uint(buffer, kp);
}

static struct kernel_param_ops cap_ops = {
	.set = cpu_user_cap_set,
	.get = cpu_user_cap_get,
};
module_param_cb(cpu_user_cap, &cap_ops, &cpu_user_cap, 0644);

static unsigned int user_cap_speed(unsigned int requested_speed)
{
//                    
#ifdef CONFIG_MACH_X3
#ifdef LOAD_SHAPER_BY_VOTE_MAX_FREQ 
	if((vmf_for_load_shaper) && requested_speed > vmf_for_load_shaper)
		requested_speed = vmf_for_load_shaper;
	
	if((vmf_for_record) && requested_speed > vmf_for_record)
			requested_speed = vmf_for_record;

	if((vmf_for_camera) && requested_speed > vmf_for_camera)
			requested_speed = vmf_for_camera;
	
	if((vmf_for_player) && requested_speed > vmf_for_player)
			requested_speed = vmf_for_player;

	if((vmf_for_network) && requested_speed > vmf_for_network)
			requested_speed = vmf_for_network;
#endif
#endif /* CONFIG_MACH_X3 */
	if ((cpu_user_cap) && (requested_speed > cpu_user_cap))
		return cpu_user_cap;
	return requested_speed;
}

#ifdef CONFIG_TEGRA_THERMAL_THROTTLE

static ssize_t show_throttle(struct cpufreq_policy *policy, char *buf)
{
	return sprintf(buf, "%u\n", tegra_is_throttling());
}

cpufreq_freq_attr_ro(throttle);
#endif /* CONFIG_TEGRA_THERMAL_THROTTLE */

#ifdef CONFIG_TEGRA_EDP_LIMITS

static const struct tegra_edp_limits *cpu_edp_limits;
static int cpu_edp_limits_size;

static const unsigned int *system_edp_limits;
static bool system_edp_alarm;

static int edp_thermal_index;
static cpumask_t edp_cpumask;
static unsigned int edp_limit;

unsigned int tegra_get_edp_limit(void)
{
	return edp_limit;
}

static unsigned int edp_predict_limit(unsigned int cpus)
{
	unsigned int limit = 0;

	BUG_ON(cpus == 0);
	if (cpu_edp_limits) {
		BUG_ON(edp_thermal_index >= cpu_edp_limits_size);
		limit = cpu_edp_limits[edp_thermal_index].freq_limits[cpus - 1];
	}
	if (system_edp_limits && system_edp_alarm)
		limit = min(limit, system_edp_limits[cpus - 1]);

	return limit;
}

static void edp_update_limit(void)
{
	unsigned int limit = edp_predict_limit(cpumask_weight(&edp_cpumask));

#ifdef CONFIG_TEGRA_EDP_EXACT_FREQ
	edp_limit = limit;
#else
	unsigned int i;
	for (i = 0; freq_table[i].frequency != CPUFREQ_TABLE_END; i++) {
		if (freq_table[i].frequency > limit) {
			break;
		}
	}
	BUG_ON(i == 0);	/* min freq above the limit or table empty */
	edp_limit = freq_table[i-1].frequency;
#endif
}

static unsigned int edp_governor_speed(unsigned int requested_speed)
{
//                    
#ifdef CONFIG_MACH_X3
#ifdef LGE_EDP_ENABLE_CONTROL
	if(force_disable_edp)
		return requested_speed;
#endif
#endif /* CONFIG_MACH_X3 */
	if ((!edp_limit) || (requested_speed <= edp_limit))
		return requested_speed;
	else
		return edp_limit;
}

int tegra_edp_update_thermal_zone(int temperature)
{
	int i;
	int ret = 0;
	int nlimits = cpu_edp_limits_size;
	int index;

	if (!cpu_edp_limits)
		return -EINVAL;

	index = nlimits - 1;

	if (temperature < cpu_edp_limits[0].temperature) {
		index = 0;
	} else {
		for (i = 0; i < (nlimits - 1); i++) {
			if (temperature >= cpu_edp_limits[i].temperature &&
			   temperature < cpu_edp_limits[i + 1].temperature) {
				index = i + 1;
				break;
			}
		}
	}

	mutex_lock(&tegra_cpu_lock);
	edp_thermal_index = index;

	/* Update cpu rate if cpufreq (at least on cpu0) is already started;
	   alter cpu dvfs table for this thermal zone if necessary */
	tegra_cpu_dvfs_alter(edp_thermal_index, &edp_cpumask, true, 0);
	if (target_cpu_speed[0]) {
		edp_update_limit();
		tegra_cpu_set_speed_cap(NULL);
	}
	tegra_cpu_dvfs_alter(edp_thermal_index, &edp_cpumask, false, 0);
	mutex_unlock(&tegra_cpu_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(tegra_edp_update_thermal_zone);

int tegra_system_edp_alarm(bool alarm)
{
	int ret = -ENODEV;

	mutex_lock(&tegra_cpu_lock);
	system_edp_alarm = alarm;

	/* Update cpu rate if cpufreq (at least on cpu0) is already started
	   and cancel emergency throttling after either edp limit is applied
	   or alarm is canceled */
	if (target_cpu_speed[0]) {
		edp_update_limit();
		ret = tegra_cpu_set_speed_cap(NULL);
	}
	if (!ret || !alarm)
		tegra_edp_throttle_cpu_now(0);

	mutex_unlock(&tegra_cpu_lock);

	return ret;
}

bool tegra_cpu_edp_favor_up(unsigned int n, int mp_overhead)
{
	unsigned int current_limit, next_limit;

	if (n == 0)
		return true;

	if (n >= ARRAY_SIZE(cpu_edp_limits->freq_limits))
		return false;

	current_limit = edp_predict_limit(n);
	next_limit = edp_predict_limit(n + 1);

	return ((next_limit * (n + 1)) >=
		(current_limit * n * (100 + mp_overhead) / 100));
}

bool tegra_cpu_edp_favor_down(unsigned int n, int mp_overhead)
{
	unsigned int current_limit, next_limit;

	if (n <= 1)
		return false;

	if (n > ARRAY_SIZE(cpu_edp_limits->freq_limits))
		return true;

	current_limit = edp_predict_limit(n);
	next_limit = edp_predict_limit(n - 1);

	return ((next_limit * (n - 1) * (100 + mp_overhead) / 100)) >
		(current_limit * n);
}

static int tegra_cpu_edp_notify(
	struct notifier_block *nb, unsigned long event, void *hcpu)
{
	int ret = 0;
	unsigned int cpu_speed, new_speed;
	int cpu = (long)hcpu;

	switch (event) {
	case CPU_UP_PREPARE:
		mutex_lock(&tegra_cpu_lock);
		cpu_set(cpu, edp_cpumask);
		edp_update_limit();

		cpu_speed = tegra_getspeed(0);
		new_speed = edp_governor_speed(cpu_speed);
		if (new_speed < cpu_speed) {
			ret = tegra_cpu_set_speed_cap(NULL);
			printk(KERN_DEBUG "cpu-tegra:%sforce EDP limit %u kHz"
				"\n", ret ? " failed to " : " ", new_speed);
		}
		if (!ret)
			ret = tegra_cpu_dvfs_alter(
				edp_thermal_index, &edp_cpumask, false, event);
		if (ret) {
			cpu_clear(cpu, edp_cpumask);
			edp_update_limit();
		}
		mutex_unlock(&tegra_cpu_lock);
		break;
	case CPU_DEAD:
		mutex_lock(&tegra_cpu_lock);
		cpu_clear(cpu, edp_cpumask);
		tegra_cpu_dvfs_alter(
			edp_thermal_index, &edp_cpumask, true, event);
		edp_update_limit();
		tegra_cpu_set_speed_cap(NULL);
		mutex_unlock(&tegra_cpu_lock);
		break;
	}
	return notifier_from_errno(ret);
}

static struct notifier_block tegra_cpu_edp_notifier = {
	.notifier_call = tegra_cpu_edp_notify,
};

static void tegra_cpu_edp_init(bool resume)
{
	tegra_get_system_edp_limits(&system_edp_limits);
	tegra_get_cpu_edp_limits(&cpu_edp_limits, &cpu_edp_limits_size);

	if (!(cpu_edp_limits || system_edp_limits)) {
		if (!resume)
			pr_info("cpu-tegra: no EDP table is provided\n");
		return;
	}

	/* FIXME: use the highest temperature limits if sensor is not on-line?
	 * If thermal zone is not set yet by the sensor, edp_thermal_index = 0.
	 * Boot frequency allowed SoC to get here, should work till sensor is
	 * initialized.
	 */
	edp_cpumask = *cpu_online_mask;
	edp_update_limit();

	if (!resume) {
		register_hotcpu_notifier(&tegra_cpu_edp_notifier);
		pr_info("cpu-tegra: init EDP limit: %u MHz\n", edp_limit/1000);
	}
}

static void tegra_cpu_edp_exit(void)
{
	if (!(cpu_edp_limits || system_edp_limits))
		return;

	unregister_hotcpu_notifier(&tegra_cpu_edp_notifier);
}

#ifdef CONFIG_DEBUG_FS

static int system_edp_alarm_get(void *data, u64 *val)
{
	*val = (u64)system_edp_alarm;
	return 0;
}
static int system_edp_alarm_set(void *data, u64 val)
{
	if (val > 1) {	/* emulate emergency throttling */
		tegra_edp_throttle_cpu_now(val);
		return 0;
	}
	return tegra_system_edp_alarm((bool)val);
}
DEFINE_SIMPLE_ATTRIBUTE(system_edp_alarm_fops,
			system_edp_alarm_get, system_edp_alarm_set, "%llu\n");

static int __init tegra_edp_debug_init(struct dentry *cpu_tegra_debugfs_root)
{
	if (!debugfs_create_file("edp_alarm", 0644, cpu_tegra_debugfs_root,
				 NULL, &system_edp_alarm_fops))
		return -ENOMEM;

	return 0;
}
#endif

#else	/* CONFIG_TEGRA_EDP_LIMITS */
#define edp_governor_speed(requested_speed) (requested_speed)
#define tegra_cpu_edp_init(resume)
#define tegra_cpu_edp_exit()
#define tegra_edp_debug_init(cpu_tegra_debugfs_root) (0)
#endif	/* CONFIG_TEGRA_EDP_LIMITS */

#ifdef CONFIG_DEBUG_FS

static struct dentry *cpu_tegra_debugfs_root;

static int __init tegra_cpu_debug_init(void)
{
	cpu_tegra_debugfs_root = debugfs_create_dir("cpu-tegra", 0);

	if (!cpu_tegra_debugfs_root)
		return -ENOMEM;

	if (tegra_edp_debug_init(cpu_tegra_debugfs_root))
		goto err_out;

	return 0;

err_out:
	debugfs_remove_recursive(cpu_tegra_debugfs_root);
	return -ENOMEM;
}

static void __exit tegra_cpu_debug_exit(void)
{
	debugfs_remove_recursive(cpu_tegra_debugfs_root);
}

late_initcall(tegra_cpu_debug_init);
module_exit(tegra_cpu_debug_exit);
#endif /* CONFIG_DEBUG_FS */

int tegra_verify_speed(struct cpufreq_policy *policy)
{
	return cpufreq_frequency_table_verify(policy, freq_table);
}

unsigned int tegra_getspeed(unsigned int cpu)
{
	unsigned long rate;

	if (cpu >= CONFIG_NR_CPUS)
		return 0;

	rate = clk_get_rate(cpu_clk) / 1000;
	return rate;
}

int tegra_update_cpu_speed(unsigned long rate)
{
	int ret = 0;
	struct cpufreq_freqs freqs;

	freqs.old = tegra_getspeed(0);
	freqs.new = rate;

	rate = clk_round_rate(cpu_clk, rate * 1000);
	if (!IS_ERR_VALUE(rate))
		freqs.new = rate / 1000;

	if (freqs.old == freqs.new)
		return ret;

	/*
	 * Vote on memory bus frequency based on cpu frequency
	 * This sets the minimum frequency, display or avp may request higher
	 */
	if (freqs.old < freqs.new) {
		ret = tegra_update_mselect_rate(freqs.new);
		if (ret) {
			pr_err("cpu-tegra: Failed to scale mselect for cpu"
			       " frequency %u kHz\n", freqs.new);
			return ret;
		}
		ret = clk_set_rate(emc_clk, tegra_emc_to_cpu_ratio(freqs.new));
		if (ret) {
			pr_err("cpu-tegra: Failed to scale emc for cpu"
			       " frequency %u kHz\n", freqs.new);
			return ret;
		}
	}

	for_each_online_cpu(freqs.cpu)
		cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);

#ifdef CONFIG_CPU_FREQ_DEBUG
	printk(KERN_DEBUG "cpufreq-tegra: transition: %u --> %u\n",
	       freqs.old, freqs.new);
#endif

	ret = clk_set_rate(cpu_clk, freqs.new * 1000);
	if (ret) {
		pr_err("cpu-tegra: Failed to set cpu frequency to %d kHz\n",
			freqs.new);
		return ret;
	}

	for_each_online_cpu(freqs.cpu)
		cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);

	if (freqs.old > freqs.new) {
		clk_set_rate(emc_clk, tegra_emc_to_cpu_ratio(freqs.new));
		tegra_update_mselect_rate(freqs.new);
	}

	return 0;
}

unsigned int tegra_count_slow_cpus(unsigned long speed_limit)
{
	unsigned int cnt = 0;
	int i;

	for_each_online_cpu(i)
		if (target_cpu_speed[i] <= speed_limit)
			cnt++;
	return cnt;
}

unsigned int tegra_get_slowest_cpu_n(void) {
	unsigned int cpu = nr_cpu_ids;
	unsigned long rate = ULONG_MAX;
	int i;

	for_each_online_cpu(i)
		if ((i > 0) && (rate > target_cpu_speed[i])) {
			cpu = i;
			rate = target_cpu_speed[i];
		}
	return cpu;
}

unsigned long tegra_cpu_lowest_speed(void) {
	unsigned long rate = ULONG_MAX;
	int i;

	for_each_online_cpu(i)
		rate = min(rate, target_cpu_speed[i]);
	return rate;
}

unsigned long tegra_cpu_highest_speed(void) {
	unsigned long policy_max = ULONG_MAX;
	unsigned long rate = 0;
	int i;

	for_each_online_cpu(i) {
		if (force_policy_max)
			policy_max = min(policy_max, policy_max_speed[i]);
		rate = max(rate, target_cpu_speed[i]);
	}
	rate = min(rate, policy_max);
	return rate;
}
//                    
#ifdef CONFIG_MACH_X3
#ifdef LOAD_SHAPER_BY_VOTE_MAX_FREQ
void tegra_cpu_check_boost_limit(unsigned int new_speed)
{
	long use_token = new_speed - threashold_allowed_clk;

	static bool registered = false;


	if(is_enable_boost_load_shaper == false)
	{
		vmf_for_load_shaper = 0;
		registered = false;
		token_in_bucket = MAX_BUCKET_SIZE;
		return;
	}

	use_token = use_token > 0 ? use_token / 100000 : 0;

	if(!use_token)
	{
		token_in_bucket += restore_token;	
		token_in_bucket = token_in_bucket > MAX_BUCKET_SIZE ? MAX_BUCKET_SIZE : token_in_bucket;
	}
	else
		token_in_bucket -= use_token ;
	
	token_in_bucket = token_in_bucket > 0 ? token_in_bucket : 0;

	if(token_in_bucket > threashold_enable_token)		
	{
		if(registered) {
			vmf_for_load_shaper = 0;
			printk("boost load shaper off!\n");
		}
		registered = false;
	}
	else
	{
		if(!registered)	{
			vmf_for_load_shaper = threashold_limited_clk;
			printk("boost load shaper triggered!\n");
		}
		registered = true;
	}


}
#endif
#endif /* CONFIG_MACH_X3 */

int tegra_cpu_set_speed_cap(unsigned int *speed_cap)
{
	int ret = 0;
	unsigned int new_speed = tegra_cpu_highest_speed();

	if (is_suspended)
		return -EBUSY;

	new_speed = tegra_throttle_governor_speed(new_speed);
	new_speed = edp_governor_speed(new_speed);
	new_speed = user_cap_speed(new_speed);
	if (speed_cap)
		*speed_cap = new_speed;
//                    
#ifdef CONFIG_MACH_X3
#ifdef LOAD_SHAPER_BY_VOTE_MAX_FREQ
	tegra_cpu_check_boost_limit(new_speed);
#endif
#endif /* CONFIG_MACH_X3 */


	ret = tegra_update_cpu_speed(new_speed);
	if (ret == 0)
		tegra_auto_hotplug_governor(new_speed, false);
	return ret;
}

int tegra_suspended_target(unsigned int target_freq)
{
	unsigned int new_speed = target_freq;

	if (!is_suspended)
		return -EBUSY;

	/* apply only "hard" caps */
	new_speed = tegra_throttle_governor_speed(new_speed);
	new_speed = edp_governor_speed(new_speed);

	return tegra_update_cpu_speed(new_speed);
}

static int tegra_target(struct cpufreq_policy *policy,
		       unsigned int target_freq,
		       unsigned int relation)
{
	int idx;
	unsigned int freq;
	unsigned int new_speed;
	int ret = 0;

	mutex_lock(&tegra_cpu_lock);

	ret = cpufreq_frequency_table_target(policy, freq_table, target_freq,
		relation, &idx);
	if (ret)
		goto _out;

	freq = freq_table[idx].frequency;

	target_cpu_speed[policy->cpu] = freq;
	ret = tegra_cpu_set_speed_cap(&new_speed);
_out:
	mutex_unlock(&tegra_cpu_lock);

	return ret;
}


static int tegra_pm_notify(struct notifier_block *nb, unsigned long event,
	void *dummy)
{
	printk("%s start [%d]\n", __func__, event);  //for debug
	mutex_lock(&tegra_cpu_lock);
	if (event == PM_SUSPEND_PREPARE) {
		is_suspended = true;
		pr_info("Tegra cpufreq suspend: setting frequency to %d kHz\n",
			freq_table[suspend_index].frequency);
		tegra_update_cpu_speed(freq_table[suspend_index].frequency);
		tegra_auto_hotplug_governor(
			freq_table[suspend_index].frequency, true);
//                    
#ifdef CONFIG_MACH_X3
		#ifdef LOAD_SHAPER_BY_VOTE_MAX_FREQ
		token_in_bucket = MAX_BUCKET_SIZE;
		#endif
#endif /* CONFIG_MACH_X3 */
	} else if (event == PM_POST_SUSPEND) {
		unsigned int freq;
		is_suspended = false;
		tegra_cpu_edp_init(true);
		tegra_cpu_set_speed_cap(&freq);
		pr_info("Tegra cpufreq resume: restoring frequency to %d kHz\n",
			freq);
	}
	mutex_unlock(&tegra_cpu_lock);

	return NOTIFY_OK;
}

static struct notifier_block tegra_cpu_pm_notifier = {
	.notifier_call = tegra_pm_notify,
};

static int tegra_cpu_init(struct cpufreq_policy *policy)
{
	if (policy->cpu >= CONFIG_NR_CPUS)
		return -EINVAL;

	cpu_clk = clk_get_sys(NULL, "cpu");
	if (IS_ERR(cpu_clk))
		return PTR_ERR(cpu_clk);

	emc_clk = clk_get_sys("cpu", "emc");
	if (IS_ERR(emc_clk)) {
		clk_put(cpu_clk);
		return PTR_ERR(emc_clk);
	}

	clk_enable(emc_clk);
	clk_enable(cpu_clk);

	cpufreq_frequency_table_cpuinfo(policy, freq_table);
	cpufreq_frequency_table_get_attr(freq_table, policy->cpu);
	policy->cur = tegra_getspeed(policy->cpu);
	target_cpu_speed[policy->cpu] = policy->cur;

	/* FIXME: what's the actual transition time? */
	policy->cpuinfo.transition_latency = 300 * 1000;

	policy->shared_type = CPUFREQ_SHARED_TYPE_ALL;
	cpumask_copy(policy->related_cpus, cpu_possible_mask);

	if (policy->cpu == 0) {
		register_pm_notifier(&tegra_cpu_pm_notifier);
	}

	return 0;
}

static int tegra_cpu_exit(struct cpufreq_policy *policy)
{
	cpufreq_frequency_table_cpuinfo(policy, freq_table);
	clk_disable(emc_clk);
	clk_put(emc_clk);
	clk_put(cpu_clk);
	return 0;
}

static int tegra_cpufreq_policy_notifier(
	struct notifier_block *nb, unsigned long event, void *data)
{
	int i, ret;
	struct cpufreq_policy *policy = data;

	if (event == CPUFREQ_NOTIFY) {
		ret = cpufreq_frequency_table_target(policy, freq_table,
			policy->max, CPUFREQ_RELATION_H, &i);
		policy_max_speed[policy->cpu] =
			ret ? policy->max : freq_table[i].frequency;
	}
	return NOTIFY_OK;
}

static struct notifier_block tegra_cpufreq_policy_nb = {
	.notifier_call = tegra_cpufreq_policy_notifier,
};

static struct freq_attr *tegra_cpufreq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
#ifdef CONFIG_TEGRA_THERMAL_THROTTLE
	&throttle,
#endif
	NULL,
};

static struct cpufreq_driver tegra_cpufreq_driver = {
	.verify		= tegra_verify_speed,
	.target		= tegra_target,
	.get		= tegra_getspeed,
	.init		= tegra_cpu_init,
	.exit		= tegra_cpu_exit,
	.name		= "tegra",
	.attr		= tegra_cpufreq_attr,
};

static int __init tegra_cpufreq_init(void)
{
	int ret = 0;

	struct tegra_cpufreq_table_data *table_data =
		tegra_cpufreq_table_get();
	if (IS_ERR_OR_NULL(table_data))
		return -EINVAL;

	suspend_index = table_data->suspend_index;

	ret = tegra_throttle_init(&tegra_cpu_lock);
	if (ret)
		return ret;

	ret = tegra_auto_hotplug_init(&tegra_cpu_lock);
	if (ret)
		return ret;

	freq_table = table_data->freq_table;
	tegra_cpu_edp_init(false);

	ret = cpufreq_register_notifier(
		&tegra_cpufreq_policy_nb, CPUFREQ_POLICY_NOTIFIER);
	if (ret)
		return ret;

	return cpufreq_register_driver(&tegra_cpufreq_driver);
}

static void __exit tegra_cpufreq_exit(void)
{
	tegra_throttle_exit();
	tegra_cpu_edp_exit();
	tegra_auto_hotplug_exit();
	cpufreq_unregister_driver(&tegra_cpufreq_driver);
	cpufreq_unregister_notifier(
		&tegra_cpufreq_policy_nb, CPUFREQ_POLICY_NOTIFIER);
}


MODULE_AUTHOR("Colin Cross <ccross@android.com>");
MODULE_DESCRIPTION("cpufreq driver for Nvidia Tegra2");
MODULE_LICENSE("GPL");
module_init(tegra_cpufreq_init);
module_exit(tegra_cpufreq_exit);
