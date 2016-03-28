/*
 * Detect Hung Task
 *
 * kernel/hung_task.c - kernel thread for detecting tasks stuck in D state
 *
 */

#include <linux/mm.h>
#include <linux/cpu.h>
#include <linux/nmi.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/freezer.h>
#include <linux/kthread.h>
#include <linux/lockdep.h>
#include <linux/export.h>
#include <linux/sysctl.h>

#ifdef CONFIG_HW_DETECT_HUNG_TASK

#include <linux/proc_fs.h>
#include <linux/errno.h>
#include <linux/uaccess.h>
#include <linux/sched.h>
#include <linux/syscalls.h>

#define NAME_NUM  16

#define NO_MATCH_INLIST -1
#define NOT_DEFINE 0
#define WHITE_LIST 1
#define BLACK_LIST 2
static unsigned int whitelist;

#define HT_ENABLE 1
#define HT_DISABLE 0
static unsigned int hungtask_enable;

/* storage names and last switch counts of process in D status */
struct tag_switch_count {
	pid_t p_pid;
	unsigned long last_swithc_count;
};

/* "-blacklist mode-: " has the same size  */
#define APX_LENGTH sizeof("-whitelist mode-: ")

static char p_name[TASK_COMM_LEN*NAME_NUM + APX_LENGTH] = {0};
/* storage proccess list in hung task mechanism */
struct name_table {
	char name[TASK_COMM_LEN];
	pid_t p_pid;
};

static struct name_table p_name_table[NAME_NUM];
static struct tag_switch_count last_switch_count_table[NAME_NUM];

static bool rcu_lock_break(struct task_struct *g, struct task_struct *t);

#endif /*CONFIG_HW_DETECT_HUNG_TASK*/

/*
 * The number of tasks checked:
 */
unsigned long __read_mostly sysctl_hung_task_check_count = PID_MAX_LIMIT;

/*
 * Limit number of tasks checked in a batch.
 *
 * This value controls the preemptibility of khungtaskd since preemption
 * is disabled during the critical section. It also controls the size of
 * the RCU grace period. So it needs to be upper-bound.
 */
#define HUNG_TASK_BATCHING 1024

/*
 * Zero means infinite timeout - no checking done:
 */
unsigned long __read_mostly sysctl_hung_task_timeout_secs = CONFIG_DEFAULT_HUNG_TASK_TIMEOUT;

unsigned long __read_mostly sysctl_hung_task_warnings = 10;

static int __read_mostly did_panic;

static struct task_struct *watchdog_task;

/*
 * Should we panic (and reboot, if panic_timeout= is set) when a
 * hung task is detected:
 */
unsigned int __read_mostly sysctl_hung_task_panic =
				CONFIG_BOOTPARAM_HUNG_TASK_PANIC_VALUE;

static int __init hung_task_panic_setup(char *str)
{
	sysctl_hung_task_panic = simple_strtoul(str, NULL, 0);

	return 1;
}
__setup("hung_task_panic=", hung_task_panic_setup);

static int
hung_task_panic(struct notifier_block *this, unsigned long event, void *ptr)
{
	did_panic = 1;

	return NOTIFY_DONE;
}

static struct notifier_block panic_block = {
	.notifier_call = hung_task_panic,
};

#ifdef CONFIG_HW_DETECT_HUNG_TASK
static void findpname(struct task_struct *t)
{
	int i = 0;
	while ((i < NAME_NUM) && ('\0' != p_name_table[i].name[0])) {
		if (strlen(t->comm) == strlen(p_name_table[i].name))
			if (strncmp(p_name_table[i].name,
				t->comm, strlen(t->comm)) == 0) {
				p_name_table[i].p_pid = t->pid;
			}
		i++;
	}
}

static int checklist(pid_t ht_pid, pid_t ht_ppid, unsigned int list_category)
{
	int i = 0;
	int in_list = 0;
	if (WHITE_LIST == list_category) {
		while ((i < NAME_NUM) && (0 != p_name_table[i].p_pid)) {
			if ((p_name_table[i].p_pid == ht_pid)
				|| (p_name_table[i].p_pid == ht_ppid))
				in_list++;

			i++;
		}
	} else if (BLACK_LIST == list_category) {
		while ((i < NAME_NUM) && (0 != p_name_table[i].p_pid)) {
			if (p_name_table[i].p_pid == ht_pid)
				in_list++;

			i++;
		}
	}

	return in_list;
}

#endif /*CONFIG_HW_DETECT_HUNG_TASK*/

static void check_hung_task(struct task_struct *t, unsigned long timeout)
{
	unsigned long switch_count = t->nvcsw + t->nivcsw;

#ifdef CONFIG_HW_DETECT_HUNG_TASK

	int idx, first_empty_item = -1;
	int in_list = 0;
	unsigned int list_category = whitelist;
	pid_t ht_pid, ht_ppid;

#endif /*CONFIG_HW_DETECT_HUNG_TASK*/

	/*
	 * Ensure the task is not frozen.
	 * Also, skip vfork and any other user process that freezer should skip.
	 */
	if (unlikely(t->flags & (PF_FROZEN | PF_FREEZER_SKIP)))
	    return;

	/*
	 * When a freshly created task is scheduled once, changes its state to
	 * TASK_UNINTERRUPTIBLE without having ever been switched out once, it
	 * musn't be checked.
	 */
	if (unlikely(!switch_count))
		return;

#ifdef CONFIG_HW_DETECT_HUNG_TASK

	/*
	 * if whitelist is NOT_DEFINE or WHITE_LIST with list is null,
	 * then skip check.
	 */

	if (NOT_DEFINE == list_category)
		return;

	if ((WHITE_LIST == list_category) && ('\0' == p_name_table[0].name[0]))
		return;

	ht_pid = t->pid;
	ht_ppid = t->tgid;


	in_list = checklist(ht_pid, ht_ppid, list_category);
	if (0 == in_list) {
		if (WHITE_LIST == list_category)
			return;
	} else {
		if (BLACK_LIST == list_category)
			return;
	}

	/* find last swich count record in last_switch_count_table */
	for (idx = 0; idx < NAME_NUM; idx++) {
		if (0 == last_switch_count_table[idx].p_pid) {
			if (-1 == first_empty_item)
				first_empty_item = idx;
		} else {
			if (last_switch_count_table[idx].p_pid == ht_pid)
				break;
		}
	}

	/* if current proccess is not in last switch count table,
	 * insert a new record
	 */
	if (NAME_NUM == idx) {
		if (first_empty_item == -1)
			return;
		last_switch_count_table[first_empty_item].p_pid = ht_pid;
		last_switch_count_table[first_empty_item].last_swithc_count = 0;
		idx = first_empty_item;
	}

	if (switch_count != last_switch_count_table[idx].last_swithc_count) {

		last_switch_count_table[idx].last_swithc_count = switch_count;
		return;
	}

#else

	if (switch_count != t->last_switch_count) {
		t->last_switch_count = switch_count;
		return;
	}

#endif /*CONFIG_HW_DETECT_HUNG_TASK*/

	if (!sysctl_hung_task_warnings)
		return;

#ifdef CONFIG_HW_DETECT_HUNG_TASK
	/* sysctl_hung_task_warnings--;*/
#else
	sysctl_hung_task_warnings--;
#endif /*CONFIG_HW_DETECT_HUNG_TASK*/

	/*
	 * Ok, the task did not get scheduled for more than 2 minutes,
	 * complain:
	 */
	printk(KERN_ERR "INFO: task %s:%d blocked for more than "
			"%ld seconds.\n", t->comm, t->pid, timeout);
	printk(KERN_ERR "\"echo 0 > /proc/sys/kernel/hung_task_timeout_secs\""
			" disables this message.\n");
	sched_show_task(t);
	debug_show_held_locks(t);

	touch_nmi_watchdog();
#ifdef CONFIG_HW_DETECT_HUNG_TASK	
	schedule_timeout_interruptible(200);
#endif /*CONFIG_HW_DETECT_HUNG_TASK*/	
	if (sysctl_hung_task_panic) {
		trigger_all_cpu_backtrace();
		panic("hung_task: blocked tasks");
	}
}

/*
 * To avoid extending the RCU grace period for an unbounded amount of time,
 * periodically exit the critical section and enter a new one.
 *
 * For preemptible RCU it is sufficient to call rcu_read_unlock in order
 * to exit the grace period. For classic RCU, a reschedule is required.
 */
static bool rcu_lock_break(struct task_struct *g, struct task_struct *t)
{
	bool can_cont;

	get_task_struct(g);
	get_task_struct(t);
	rcu_read_unlock();
	cond_resched();
	rcu_read_lock();
	can_cont = pid_alive(g) && pid_alive(t);
	put_task_struct(t);
	put_task_struct(g);

	return can_cont;
}

/*
 * Check whether a TASK_UNINTERRUPTIBLE does not get woken up for
 * a really long time (120 seconds). If that happens, print out
 * a warning.
 */
static void check_hung_uninterruptible_tasks(unsigned long timeout)
{
	int max_count = sysctl_hung_task_check_count;
	int batch_count = HUNG_TASK_BATCHING;
	struct task_struct *g, *t;

	/*
	 * If the system crashed already then all bets are off,
	 * do not report extra hung tasks:
	 */
	if (test_taint(TAINT_DIE) || did_panic)
		return;

	rcu_read_lock();
	do_each_thread(g, t) {
		if (!max_count--)
			goto unlock;
		if (!--batch_count) {
			batch_count = HUNG_TASK_BATCHING;
			if (!rcu_lock_break(g, t))
				goto unlock;
		}
		/* get pid for each name in WhiteList */
#ifdef CONFIG_HW_DETECT_HUNG_TASK		
		findpname(t);
#endif /*CONFIG_HW_DETECT_HUNG_TASK*/
		/* use "==" to skip the TASK_KILLABLE tasks waiting on NFS */
		if (t->state == TASK_UNINTERRUPTIBLE)
			check_hung_task(t, timeout);
	} while_each_thread(g, t);
 unlock:
	rcu_read_unlock();
}

static unsigned long timeout_jiffies(unsigned long timeout)
{
	/* timeout of 0 will disable the watchdog */
	return timeout ? timeout * HZ : MAX_SCHEDULE_TIMEOUT;
}

/*
 * Process updating of timeout sysctl
 */
int proc_dohung_task_timeout_secs(struct ctl_table *table, int write,
				  void __user *buffer,
				  size_t *lenp, loff_t *ppos)
{
	int ret;

	ret = proc_doulongvec_minmax(table, write, buffer, lenp, ppos);

	if (ret || !write)
		goto out;

	wake_up_process(watchdog_task);

 out:
	return ret;
}

/*
 * kthread which checks for tasks stuck in D state
 */
static int watchdog(void *dummy)
{
	set_user_nice(current, 0);

	for ( ; ; ) {
		unsigned long timeout = sysctl_hung_task_timeout_secs;

		while (schedule_timeout_interruptible(timeout_jiffies(timeout)))
			timeout = sysctl_hung_task_timeout_secs;

#ifdef CONFIG_HW_DETECT_HUNG_TASK
		if (hungtask_enable)
			check_hung_uninterruptible_tasks(timeout);
#else
		check_hung_uninterruptible_tasks(timeout);
#endif /*CONFIG_HW_DETECT_HUNG_TASK*/
	}

	return 0;
}

#ifdef CONFIG_HW_DETECT_HUNG_TASK

/*
*	hwht_monitor_proc_show  -	Called when 'cat' method
*	is used on entry 'pname' in /proc fs.
*	most of the parameters is created by kernel.
*/
static int hwht_monitor_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", p_name);
	return 0;
}


static int hwht_monitor_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, hwht_monitor_proc_show, NULL);
}


/* storage proccess names in [pname] to
 * [pname_table], and return the numbers of process
 */
static int rebuild_name_table(char *pname, int pname_len)
{
	int count = 0;
	int proc_name_len;
	const char *curr = pname;
	char *curr_table;
	int idx = 0;
	int detected = 0;

	whitelist = NOT_DEFINE;


	/* reset the table to empty */

	memset(p_name_table, 0x00, sizeof(p_name_table));

	while ('\0' != *curr && pname_len) {
		/* proccess names are seperated by comma */
		while ((',' == *curr) && pname_len) {
			curr++;
			pname_len--;
		}

		/* check if the number of proccess exceed the limit,
		 * pointer [curr] not an end symbol indicates
		 * that the after [NAME_NUM] proccess,
		 * the [NAME_NUM + 1]th proccess was found
		 * Ignore the redundant names.
		 */
		if (NAME_NUM - 1 == count) {
			pr_err("more than 15 names,ignore redundant.\n");
			break;
		}
		
		/* the [count]th name should be storage in corresponding
		 * item in table, and [proc_name_len] is set to count
		 * the length of process name
		 */
		proc_name_len = 0;
		curr_table = p_name_table[count].name;

		while (',' != *curr && '\0' != *curr && pname_len) {
			*curr_table = *curr;
			curr_table++;
			curr++;
			proc_name_len++;

			/* check if the length of
			 * proccess name exceed the limit
			 */
			if (TASK_COMM_LEN == proc_name_len)
				goto err_proc_name;
			pname_len--;
		}
		*curr_table = '\0';

		if ((count == 0) && (detected == 0)) {
			detected++;
			if (strncmp(p_name_table[0].name, "whitelist",
					strlen("whitelist")) == 0) {
				pr_err("hung_task:set to whitelist.\n");
				whitelist = WHITE_LIST;
				continue;
			} else if (strncmp(p_name_table[0].name, "blacklist",
						strlen("blacklist")) == 0) {
				whitelist = BLACK_LIST;
				pr_err("hung_task:set to blacklist.\n");
				continue;
			} else {
				pr_err("hung_task: add whitelist or blacklist");
				pr_err("before process name.\n");
				goto err_proc_name;
			}
		}
		pr_err("\nhung_task: name_table: %d, %s,name_len: %d\n",
			count, p_name_table[count].name, proc_name_len);

		/* count how many proccess,
		 * only when [proc_name_len] is not zero,
		 * one new proccess was added into [pname_table]
		 */
		if (proc_name_len)
			count++;
	}
	/*last_switch_count_table  reset*/
	for (idx = 0; idx < NAME_NUM; idx++) {
		last_switch_count_table[idx].p_pid = 0;
		last_switch_count_table[idx].last_swithc_count = 0;
	}

	return count;

err_proc_name:
	memset(p_name_table, 0x00, sizeof(p_name_table));
	memset(p_name, 0x00, sizeof(p_name));
	whitelist = NOT_DEFINE;
	pr_err("hung_task: rebuild_name_table: Error: process name");
	pr_err(" is invallid, set monitor_list failed.\n");

	return 0;
}

/* since the proccess name written into [pname_table]
 * may be different from original input,
 * [p_name] should be modified to adjust [pname_table]
 */
static int modify_pname(int num_count)
{
	int i, len_count;
	size_t tlen;
	char buf[20+TASK_COMM_LEN*NAME_NUM];

	memset((void *)p_name, 0x00, sizeof(p_name));
	memset((void *)buf, 0x00, sizeof(buf));

	if (WHITE_LIST == whitelist) {
		tlen = strlcat(buf, "-whitelist mode-: ", sizeof(buf));
		if (tlen >= sizeof(buf))
			return -ENAMETOOLONG;
	} else if (BLACK_LIST == whitelist) {
		tlen = strlcat(buf, "-blacklist mode-: ", sizeof(buf));
		if (tlen >= sizeof(buf))
			return -ENAMETOOLONG;
	} else {
		len_count = 0;
		return len_count;
	}

	for (i = 0; i < num_count; i++) {
		tlen = strlcat(buf, p_name_table[i].name, sizeof(p_name));
		if (tlen >= sizeof(p_name))
			return -ENAMETOOLONG;

		/* seperate different proccess by a comma and a space */
		if (i != num_count - 1) {
			tlen = strlcat(buf, ",", sizeof(p_name));
			if (tlen >= sizeof(p_name))
				return -ENAMETOOLONG;
		}
	}
	tlen = strlcat(p_name, buf, sizeof(p_name));
	if (tlen >= sizeof(p_name))
		return -ENAMETOOLONG;
	len_count = strlen(p_name);
	return len_count;
}

/*
*	hwht_monitor_proc_write	-  Called when 'write' method is
*	used on entry 'p_name' in /proc fs.
*/
static ssize_t hwht_monitor_proc_write(struct file *rfile,
                const char __user *buffer, size_t count, loff_t *ppos)
{
	int num_count;
	size_t tlen;
	int ret;

	/* TASK_COMM_LEN * NAME_NUM * 2 might be larger than 128,
	 * put it into the static area is a better choice
	 */
	char tmp[TASK_COMM_LEN*NAME_NUM] = {0};

	/* reset p_name to NULL */
	whitelist = NOT_DEFINE;
	memset(p_name, 0x00, sizeof(p_name));
	memset(p_name_table, 0x00, sizeof(p_name_table));
	/*make sure count >= 1, although count will never smaller than 1*/
	if ((count < 2) || (count > sizeof(tmp) - 1)) {
		pr_err("hung_task: input string is too long or too short\n");
		return -EINVAL;
	}

	if (copy_from_user(tmp, buffer, (int)count))
		return -EFAULT;

	/* -1: remove '\n'	*/
	if (tmp[count - 1] == '\n')
		tmp[count - 1] = '\0';
	else if (tmp[count - 1] != '\0') {
		tmp[count] = '\0';
		count++;
	}
	tlen = strlcpy(p_name, tmp, count);
	if (tlen >= sizeof(p_name))
		return -ENAMETOOLONG;

	/* convert [p_name] to a table [p_name_table],
	 * and refresh the buffer [p_name]
	 */
	num_count = rebuild_name_table(p_name, count);
	ret = modify_pname(num_count);

	if (ret < 0)
		return -EFAULT;

	return count;
}

static const struct file_operations hwht_monitor_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= hwht_monitor_proc_open,
	.read	 = seq_read,
	.write	= hwht_monitor_proc_write,
	.llseek   = seq_lseek,
	.release  = single_release,
};


static int hwht_enable_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", hungtask_enable?"on":"off");
	return 0;
}


static int hwht_enable_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, hwht_enable_proc_show, NULL);
}


static ssize_t hwht_enable_proc_write(struct file *rfile,
			const char __user *buffer, size_t count, loff_t *ppos)
{
	char tmp[6];

	memset(tmp, 0, sizeof(tmp));

	if ((count < 2) || (count > sizeof(tmp) - 1)) {
		pr_err("hung_task: string too long or too short.\n");
		return -EINVAL;
	}
	if (copy_from_user(tmp, buffer, count))
		return -EFAULT;

	if (tmp[count - 1] == '\n') {
		tmp[count - 1] = '\0';
	}

	pr_err("hung_task:tmp=%s, count %d\n", tmp, (int)count);

	if (strncmp(tmp, "on", strlen("on")) == 0) {
		hungtask_enable = HT_ENABLE;
		pr_err("hung_task: hungtask_enable is set to enable.\n");
	} else if (strncmp(tmp, "off", strlen("off")) == 0) {
		hungtask_enable = HT_DISABLE;
		pr_err("hung_task: hungtask_enable is set to disable.\n");
	} else
		pr_err("hung_task: only accept on or off !\n");

	return count;
}

static const struct file_operations hwht_enable_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= hwht_enable_proc_open,
	.read	 = seq_read,
	.write	= hwht_enable_proc_write,
	.llseek   = seq_lseek,
	.release  = single_release,
};

/*
*	create proc node in /proc fs.
*/
int create_proc(void)
{
	struct proc_dir_entry *hwht_proc;
	struct proc_dir_entry *monitor_entry, *enable_entry;

	whitelist = NOT_DEFINE;

	/*Create hw_hung_task directory*/
	hwht_proc = proc_mkdir("hung_task", NULL);
	if (hwht_proc == NULL) {
		pr_err("hung_task: unable to create dir: hung_task\n");
		return -EIO;
	}

	/*Create attributes files*/
	enable_entry = proc_create("hw_enable",
		S_IFREG | S_IRUGO | S_IWUSR, hwht_proc, &hwht_enable_proc_fops);

	if (NULL == enable_entry) {
		remove_proc_entry("hung_task", NULL);
		return -EIO;
	}

	monitor_entry = proc_create("hw_monitor_list",
		S_IFREG | S_IRUGO | S_IWUSR, hwht_proc, &hwht_monitor_proc_fops);

	if (NULL == monitor_entry) {
		remove_proc_entry("hw_enable", hwht_proc);
		remove_proc_entry("hung_task", NULL);
		return -EIO;
	}

	return 0;
}

#endif /*CONFIG_HW_DETECT_HUNG_TASK*/

static int __init hung_task_init(void)
{
#ifdef CONFIG_HW_DETECT_HUNG_TASK
	int ret;
	ret = create_proc();
	if (ret)
		pr_err("hung_task: create_proc fail.\n");
#endif /*CONFIG_HW_DETECT_HUNG_TASK*/

	atomic_notifier_chain_register(&panic_notifier_list, &panic_block);
	watchdog_task = kthread_run(watchdog, NULL, "khungtaskd");

	return 0;
}

module_init(hung_task_init);
