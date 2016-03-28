#ifdef CONFIG_ARCH_MSM8909
#include <linux/mmc/core.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/swap.h>		/* For nr_free_buffer_pages() */
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/seq_file.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/moduleparam.h>
#include <linux/delay.h>
#include <linux/fcntl.h>
#include <linux/syscalls.h>
#include <asm/unistd.h>
#include <linux/string.h>
#include <linux/mmc/sdhci.h>
#include <soc/qcom/clock-local2.h>
#include <linux/clk/msm-clk-provider.h>
#include "../host/sdhci-pltfm.h"
#include "../host/sdhci.h"
#define HW_MMC_TEST
#define RESULT_OK		0
#define RESULT_FAIL		1
#define RESULT_UNSUP_HOST	2
#define RESULT_UNSUP_CARD	3
#define BUFFER_ORDER		2
#define BUFFER_SIZE		(PAGE_SIZE << BUFFER_ORDER)
#define PAR_NUM    8
#define HW_MMC_AGING_TEST

#define HW_MMC_AGING_TEST_VER "V1.0.0.0"
#define SDC0    (1<<0)
#define SDC1    (1<<1)
#define SDC2    (1<<2)

static int g_card_select = 0;
module_param(g_card_select, int, 0);
MODULE_PARM_DESC(g_card_select, "[HW_MMC_AGING_TEST]:eMMC card to run on.");

struct mmc_hw_test_debugfs_entry {
	struct dentry *hw_test_entry;
	struct dentry *hw_testlist_entry;
}g_hw_test_debugfs_entries = { NULL, NULL};

struct mmc_test_general_result {
	struct list_head link;
	struct mmc_card *card;
	int testcase;
	int result;
	struct list_head tr_lst;
};
struct mmc_host *host_clock_test=NULL;

struct mmc_test_card {
	struct mmc_card	*card;

	u8		scratch[BUFFER_SIZE];
	u8		*buffer;

	struct mmc_test_general_result	*gr;

	u32 par[PAR_NUM];
	void *trans_buf;
	u32 urgent_bkops;

};

/**
 * struct mmc_test_dbgfs_file - debugfs related file.
 * @link: double-linked list
 * @card: card under test
 * @file: file created under debugfs
 */
struct mmc_test_dbgfs_file {
	struct list_head link;
	struct mmc_card *card;
	struct dentry *file;
};

static DEFINE_MUTEX(mmc_test_lock);

static LIST_HEAD(mmc_test_result);

struct mmc_test_case {
	const char *name;
	int (*prepare)(struct mmc_test_card *);
	int (*run)(struct mmc_test_card *);
	int (*cleanup)(struct mmc_test_card *);
	int cap;
};
//#ifdef CONFIG_HW_MMC_AGING_TEST
//#define CARD_ADDR_MAGIC 0xA5A55A5A
//#endif

#ifdef HW_MMC_AGING_TEST
#define PAR_NUM    8

enum mmc_test_status{
	TEST_STATUS_OK = 0,
	TEST_STATUS_DATA_CHECK_ERR,
	TEST_STATUS_PARAM_ERR,
	TEST_STATUS_SYSTEM_ERR,
	TEST_STATUS_EMMC_ERR,
	TEST_STATUS_NOT_SUPPORT,
	TEST_STATUS_NUM
};
#endif

enum mmc_test_prep_media {
	MMC_TEST_PREP_NONE = 0,
	MMC_TEST_PREP_WRITE_FULL = 1 << 0,
	MMC_TEST_PREP_ERASE = 1 << 1,
};

/* Notice: Pls make sure the struct is synchronized within file sdhci_msm.c */
struct sdhci_msm_pltfm_data {
	/* Supported UHS-I Modes */
	u32 caps;

	/* More capabilities */
	u32 caps2;

	unsigned long mmc_bus_width;
	void *vreg_data; // modifyed, struct sdhci_msm_slot_reg_data
	bool nonremovable;
	bool pin_cfg_sts;
	void *pin_data; // modifyed, struct sdhci_msm_pin_data
	void *pctrl_data; // modifyed, struct sdhci_pinctrl_data
	u32 cpu_dma_latency_us;
	int status_gpio; /* card detection GPIO that is configured as IRQ */
	void *voting_data; // modifyed, struct sdhci_msm_bus_voting_data
	u32 *sup_clk_table;
	unsigned char sup_clk_cnt;
	int mpm_sdiowakeup_int;
	int sdiowakeup_irq;
};

/* Notice: Pls make sure the struct is synchronized within file sdhci_msm.c */
struct sdhci_msm_bus_vote {
	uint32_t client_handle;
	uint32_t curr_vote;
	int min_bw_vote;
	int max_bw_vote;
	bool is_max_bw_needed;
	struct delayed_work vote_work;
	struct device_attribute max_bus_bw;
};

/* Notice: Pls make sure the struct is synchronized within file sdhci_msm.c */
struct sdhci_msm_host {
	struct platform_device	*pdev;
	void __iomem *core_mem;    /* MSM SDCC mapped address */
	int	pwr_irq;	/* power irq */
	struct clk	 *clk;     /* main SD/MMC bus clock */
	struct clk	 *pclk;    /* SDHC peripheral bus clock */
	struct clk	 *bus_clk; /* SDHC bus voter clock */
	struct clk	 *ff_clk; /* CDC calibration fixed feedback clock */
	struct clk	 *sleep_clk; /* CDC calibration sleep clock */
	atomic_t clks_on; /* Set if clocks are enabled */
	struct sdhci_msm_pltfm_data *pdata;
	struct mmc_host  *mmc;
	struct sdhci_pltfm_data sdhci_msm_pdata;
	u32 curr_pwr_state;
	u32 curr_io_level;
	struct completion pwr_irq_completion;
	struct sdhci_msm_bus_vote msm_bus_vote;
	struct device_attribute	polling;
	u32 clk_rate; /* Keeps track of current clock rate that is set */
	bool tuning_done;
	bool calibration_done;
	u8 saved_tuning_phase;
	bool en_auto_cmd21;
	struct device_attribute auto_cmd21_attr;
	bool is_sdiowakeup_enabled;
	atomic_t controller_clock;
};

#define xo_source_val			0
#define gpll0_source_val		1
#define F(f, s, div, m, n) \
	{ \
		.freq_hz = (f), \
		.m_val = (m), \
		.n_val = ~((n)-(m)) * !!(n), \
		.d_val = ~(n),\
		.div_src_val = BVAL(4, 0, (int)(2*(div) - 1)) \
			| BVAL(10, 8, s##_source_val), \
	}

#define clk_freq_tbl_modify clk_freq_tbl

enum vdd_dig_levels {
	VDD_DIG_NONE,
	VDD_DIG_LOW,
	VDD_DIG_NOMINAL,
	VDD_DIG_HIGH,
	VDD_DIG_NUM
};

struct mmc_clk_back_info {
	u32 *sup_clk_tbl;
	u32 sup_clk_cnt;
	unsigned long fmax;
	struct clk_freq_tbl *freq_tbl;
};
struct mmc_clk_back_info g_mmc_clk_info_back;

static struct clk_freq_tbl_modify g_modify_clk_tbl_sdc[] = {
	F(    144000,	      xo,  16,	  3,   25),
	F(    400000,	      xo,  12,	  1,	4),
	F(  20000000,	   gpll0,  10,	  1,	4),
	F(  25000000,	   gpll0,  16,	  1,	2),
	F(  40000000,	   gpll0,  10,	  1,	2),
	F(  50000000,	   gpll0,  16,	  0,	0),
	F(  59200000,	   gpll0,13.5,	  0,	0),
	F( 100000000,	   gpll0,   8,	  0,	0),
	F( 114000000,	   gpll0,   7,	  0,	0),
	F( 160000000,	   gpll0,   5,	  0,	0),
	F( 177770000,	   gpll0, 4.5,	  0,	0),
	F( 200000000,	   gpll0,   4,	  0,	0),
	F( 210000000,	   gpll0,   1,	  5,   19),
	F( 230000000,	   gpll0, 3.5,	  0,	0),
	F_END
};

static u32 g_modify_sup_clk_tbl[] = {
	   400000,  25000000,  40000000,  50000000,  59200000,
	100000000, 114000000, 160000000, 177770000, 200000000,
	210000000, 230000000
}; /* we'd better don't change min clock rate */

static int __find_substr(char* str, char* substr, char** endstr)
{
	char *pStr, *pDst;

	pStr=str;
	pDst=substr;
	while(*pStr==' '){
		pStr++;
	};

	/* End of string */
	if(*pStr=='\0')
		return 1;

	while(*pStr!=' '){
		*pDst=*pStr;
		if(*pStr=='\0')
			break;
		pStr++;
		pDst++;
	};
	*pDst=0;
	*endstr=pStr;
	return 0;
}

static inline void _get_sup_clk_tbl(struct mmc_host *mmc, unsigned int **tbl, unsigned char *cnt)
{
	struct sdhci_host *host = mmc_priv(mmc);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;

	mutex_lock(&host->ios_mutex);

	*tbl = msm_host->pdata->sup_clk_table;
	*cnt = msm_host->pdata->sup_clk_cnt;

	mutex_unlock(&host->ios_mutex);
}

static inline void _set_sup_clk_tbl(struct mmc_host *mmc, unsigned int *tbl, unsigned char cnt)
{
	struct sdhci_host *host = mmc_priv(mmc);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;

	mutex_lock(&host->ios_mutex);

	msm_host->pdata->sup_clk_table = tbl;
	msm_host->pdata->sup_clk_cnt = cnt;

	mutex_unlock(&host->ios_mutex);
}

static inline struct clk *_get_host_clock(struct mmc_host *mmc)
{
	struct sdhci_host *host = mmc_priv(mmc);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;

	return msm_host->clk->parent;
}

static void init_freq_table(struct rcg_clk *p_rcg_clk)
{
	struct clk *p_xo_clk, *p_pll0_clk;
	int i;

	p_xo_clk = p_rcg_clk->freq_tbl[0].src_clk;
	p_pll0_clk = p_rcg_clk->freq_tbl[2].src_clk;
	g_modify_clk_tbl_sdc[0].src_clk = p_xo_clk;
	g_modify_clk_tbl_sdc[1].src_clk = p_xo_clk;
	for (i = 2; i < (sizeof(g_modify_clk_tbl_sdc) / sizeof(struct clk_freq_tbl_modify) - 1); i++)
		g_modify_clk_tbl_sdc[i].src_clk = p_pll0_clk;
}

static void
_modify_support_clock_table(struct mmc_host *host)
{
	unsigned int *tbl;
	unsigned char cnt;

	_get_sup_clk_tbl(host, &tbl, &cnt);

	if (tbl == g_modify_sup_clk_tbl){
		return;
	}

	/* save original table */
	if (g_mmc_clk_info_back.sup_clk_tbl == NULL)
	{
		g_mmc_clk_info_back.sup_clk_tbl = tbl;
		g_mmc_clk_info_back.sup_clk_cnt = cnt;
	}

	/* modify original table */
	cnt = sizeof(g_modify_sup_clk_tbl) / sizeof(unsigned int);
	_set_sup_clk_tbl(host, g_modify_sup_clk_tbl, cnt);

	host->f_max = g_modify_sup_clk_tbl[cnt - 1];
	host->f_min = g_modify_sup_clk_tbl[0];

}

static void
_restore_support_clock_table(struct mmc_host *host)
{
	unsigned int *tbl;
	unsigned char cnt;

	_get_sup_clk_tbl(host, &tbl, &cnt);

	if (tbl != g_modify_sup_clk_tbl)
		return;

	/* restore original table */
	_set_sup_clk_tbl(host, g_mmc_clk_info_back.sup_clk_tbl, g_mmc_clk_info_back.sup_clk_cnt);

	host->f_max = g_mmc_clk_info_back.sup_clk_tbl[g_mmc_clk_info_back.sup_clk_cnt - 1];
	host->f_min = g_mmc_clk_info_back.sup_clk_tbl[0];
}

static void
_modify_freq_table(struct mmc_host *mmc)
{
	unsigned long flags;

	struct clk *p_host_clk = _get_host_clock(mmc);
	struct rcg_clk *p_rcg_clk = container_of(p_host_clk, struct rcg_clk, c);

	if (p_rcg_clk->freq_tbl == (struct clk_freq_tbl*)g_modify_clk_tbl_sdc)
		return;

	if (g_mmc_clk_info_back.freq_tbl == NULL)
	{
		init_freq_table(p_rcg_clk);

		/* save original freq table */
		g_mmc_clk_info_back.freq_tbl = p_rcg_clk->freq_tbl;
		g_mmc_clk_info_back.fmax = p_host_clk->fmax[VDD_DIG_NOMINAL];
	}

	/* modify freq table */
	spin_lock_irqsave(&p_host_clk->lock, flags);
	p_host_clk->fmax[VDD_DIG_NOMINAL] = 230000000;
	p_rcg_clk->freq_tbl = (struct clk_freq_tbl*)(g_modify_clk_tbl_sdc);
	spin_unlock_irqrestore(&p_host_clk->lock, flags);

}

static void
_restore_freq_table(struct mmc_host *mmc)
{
	unsigned long flags;

	struct clk *p_host_clk = _get_host_clock(mmc);
	struct rcg_clk *p_rcg_clk = container_of(p_host_clk, struct rcg_clk, c);

	if (p_rcg_clk->freq_tbl != (struct clk_freq_tbl*)g_modify_clk_tbl_sdc)
		return;

	/* restore freq table */
	spin_lock_irqsave(&p_host_clk->lock, flags);
	p_host_clk->fmax[VDD_DIG_NOMINAL] = g_mmc_clk_info_back.fmax;
	p_rcg_clk->freq_tbl = g_mmc_clk_info_back.freq_tbl;
	spin_unlock_irqrestore(&p_host_clk->lock, flags);
}

static void
_add_clock_table(struct mmc_host *host, int flag)
{
	if (flag)
	{
		/* modify sup_clk_table in host->plat. It has been initialized with dtsi firstly */
		_modify_support_clock_table(host);
		/* modify freq_table in struct rcg_clk sdc1_clk */
		_modify_freq_table(host);
	}
	else
	{
		_restore_support_clock_table(host);
		_restore_freq_table(host);
	}
}

void mmc_set_clock(struct mmc_host *host, unsigned int hz);

static int
mmc_hw_test_overclocking(struct mmc_test_card * test)
{
	host_clock_test = test->card->host;

	//_add_clock_table(host_clock_test, 1);
	pr_info("change clock to %d\n", test->par[0]);
	//mmc_set_clock(host_clock_test, test->par[0]);

	/* set test result in /sys/bus/platform/devices/msm_sdcc.x/test_status */
	host_clock_test->test_status = TEST_STATUS_OK;
	return 0;
}

static int
mmc_hw_test_power_loss(struct mmc_test_card * test)
{
	pr_info("will power off the board with no any notify!\n");
	//delay 1000us, for print above msg
	usleep(1000);
	pm_power_off();
	return 0;
}

static const struct mmc_test_case mmc_aging_test_cases[] = {
	{
		.name="[HW]: EMMC power suddenly lost",
		.prepare = NULL,
		.run = mmc_hw_test_power_loss,
		.cleanup = NULL,
		.cap = SDC0|SDC1|SDC2,
	},
	{
		.name="[HW]: EMMC frequency change, <freq>",
		.prepare = NULL,
		.run = mmc_hw_test_overclocking,
		.cleanup = NULL,
		.cap = SDC0|SDC1|SDC2,
	},
};
static DEFINE_MUTEX(mmc_aging_test_lock);
static void mmc_aging_test_run(struct mmc_test_card *test, int testcase)
{
	int i, ret;

	printk("%s: Starting tests of card %s...\n",
			mmc_hostname(test->card->host), mmc_card_id(test->card));

	for (i = 0;i < ARRAY_SIZE(mmc_aging_test_cases);i++) {
		struct mmc_test_general_result *gr;

		if (testcase && ((i + 1) != testcase))
			continue;

		if((mmc_aging_test_cases[i].cap & (0x1 << g_card_select)) == 0){
			pr_warn("[HW]:%s.Tes case %d doesn't support for %s.\n",
					__func__, i + 1, mmc_hostname(test->card->host));

			return;
		}

		pr_info("%s: Test case %d. %s...\n",
				mmc_hostname(test->card->host), i + 1,
				mmc_aging_test_cases[i].name);

		if (mmc_aging_test_cases[i].prepare) {
			ret = mmc_aging_test_cases[i].prepare(test);
			if (ret) {
				pr_info("%s: Result: Prepare "
						"stage failed! (%d)\n",
						mmc_hostname(test->card->host),
						ret);
				continue;
			}
		}

		gr = kzalloc(sizeof(struct mmc_test_general_result),
				GFP_KERNEL);
		if (gr) {
			INIT_LIST_HEAD(&gr->tr_lst);

			/* Assign data what we know already */
			gr->card = test->card;
			gr->testcase = i;

			/* Append container to global one */
			list_add_tail(&gr->link, &mmc_test_result);

			/*
			 * Save the pointer to created container in our private
			 * structure.
			 */
			test->gr = gr;
		}

		ret = mmc_aging_test_cases[i].run(test);
		switch (ret) {
			case RESULT_OK:
				pr_info("%s: Result: OK\n",
						mmc_hostname(test->card->host));
				break;
			case RESULT_FAIL:
				pr_info("%s: Result: FAILED\n",
						mmc_hostname(test->card->host));
				break;
			case RESULT_UNSUP_HOST:
				pr_info("%s: Result: UNSUPPORTED "
						"(by host)\n",
						mmc_hostname(test->card->host));
				break;
			case RESULT_UNSUP_CARD:
				pr_info("%s: Result: UNSUPPORTED "
						"(by card)\n",
						mmc_hostname(test->card->host));
				break;
			default:
				pr_info("%s: Result: ERROR (%d)\n",
						mmc_hostname(test->card->host), ret);
		}

		/* Save the result */
		if (gr)
			gr->result = ret;

		if (mmc_aging_test_cases[i].cleanup) {
			ret = mmc_aging_test_cases[i].cleanup(test);
			if (ret) {
				pr_info("%s: Warning: Cleanup "
						"stage failed! (%d)\n",
						mmc_hostname(test->card->host),
						ret);
			}
		}
	}

	pr_info("%s: Tests completed.\n",
			mmc_hostname(test->card->host));
}


static int mtf_test_show(struct seq_file *sf, void *data)
{

	struct mmc_card *card = (struct mmc_card *)sf->private;
	struct mmc_test_general_result *gr;

	mutex_lock(&mmc_test_lock);

	list_for_each_entry(gr, &mmc_test_result, link) {
		if (gr->card != card)
			continue;

		seq_printf(sf, "Test %d: %d\n", gr->testcase + 1, gr->result);
	}

	mutex_unlock(&mmc_test_lock);

	return 0;
}

static int mtf_test_open(struct inode *inode, struct file *file)
{
	return single_open(file, mtf_test_show, inode->i_private);
}

static ssize_t mtf_test_write(struct file *file, const char __user *buf,
		size_t count, loff_t *pos)
{
 struct seq_file *sf = (struct seq_file *)file->private_data;
	struct mmc_card *card = (struct mmc_card *)sf->private;
	struct mmc_test_card *test;
#ifdef HW_MMC_TEST
	char lbuf[256];
	char tmp_buf[12];
	char *pStr;
	int idx;
	int ret;
#else
	char lbuf[12];
#endif
	long testcase;
	if (count >= sizeof(lbuf)){
		return -EINVAL;
	}
	if (copy_from_user(lbuf, buf, count)){
		return -EFAULT;
	}
	lbuf[count] = '\0';
#ifdef HW_MMC_TEST
	pStr = lbuf;
	ret = __find_substr(pStr,tmp_buf,&pStr);
	if(ret)
		return -EINVAL;

	if (strict_strtol(tmp_buf, 10, &testcase))
		return -EINVAL;

#else
	if (strict_strtol(lbuf, 10, &testcase))
		return -EINVAL;
#endif
	test = kzalloc(sizeof(struct mmc_test_card), GFP_KERNEL);
	if (!test)
		return -ENOMEM;

#ifdef HW_MMC_TEST
	ret = 0;
	idx = 0;
	while(0 == ret)
	{
		ret=__find_substr(pStr, tmp_buf, &pStr);
		if(ret)
			break;

		if(kstrtou32(tmp_buf, 10, &test->par[idx]))
			return -EINVAL;

		idx++;
		if(idx == PAR_NUM)
			break;
	}
#endif

	/*
	 * Remove all test cases associated with given card. Thus we have only
	 * actual data of the last run.
	 */

	test->card = card;

	test->buffer = kzalloc(BUFFER_SIZE, GFP_KERNEL);

	if (test->buffer) {
		mutex_lock(&mmc_aging_test_lock);
		mmc_aging_test_run(test, testcase);
		mutex_unlock(&mmc_aging_test_lock);
	}

	kfree(test->buffer);
	kfree(test);

	return count;
}

static const struct file_operations hw_mmc_test_fops_test = {
		.open		= mtf_test_open,
		.read		= seq_read,
		.write		= mtf_test_write,
		.llseek		= seq_lseek,
		.release	= single_release,
	};

static int mtf_testlist_show(struct seq_file *sf, void *data)
	{
		int i;

		mutex_lock(&mmc_test_lock);

		seq_printf(sf, "[HW_MMC_AGING_TEST]:version %s\n", HW_MMC_AGING_TEST_VER);
		for (i = 0; i < ARRAY_SIZE(mmc_aging_test_cases); i++)
			seq_printf(sf, "%d:\t%s\n", i+1, mmc_aging_test_cases[i].name);

		mutex_unlock(&mmc_test_lock);

		return 0;
	}

	static int mtf_testlist_open(struct inode *inode, struct file *file)
	{
		return single_open(file, mtf_testlist_show, inode->i_private);
	}

	static const struct file_operations hw_mmc_test_fops_testlist = {
		.open		= mtf_testlist_open,
		.read		= seq_read,
		.llseek		= seq_lseek,
		.release	= single_release,
	};

static int __init mmc_hw_aging_test_init(void)
	{
		struct mmc_card *card;
		long mmc_card_addr = (long)0LL;

		char format[] = "/sys/kernel/debug/mmc%d/mmc%d:0001/card_addr";
		char sys_file[64] = {0};
		char buf[64] = {0};
		mm_segment_t old_fs;

		struct file *fp = NULL;
		loff_t pos = 0;
		int read_count = 0;

		long ret;

		//default is 0;
		if(g_card_select < 0 || g_card_select > 4)
			g_card_select = 0;

		//config sys file path.
		snprintf(sys_file, sizeof(sys_file), format, g_card_select, g_card_select);

		fp = filp_open(sys_file, O_RDONLY, 0);
		if(IS_ERR(fp)){
			pr_err("[HW]:%s:%s open failed.", __func__, sys_file);
			goto err;
		}

		old_fs = get_fs();
		set_fs(KERNEL_DS);

		memset(buf, 0x00, sizeof(buf));
		read_count = vfs_read(fp, buf, sizeof(buf), &pos);
		if(read_count <= 0){
			pr_err("[HW]:%s:read_count=%d.", __func__, read_count);
			set_fs(old_fs);
			filp_close(fp, NULL);
			goto err;
		}

		set_fs(old_fs);
		filp_close(fp, NULL);

		//parse mmc card addr.
		ret = kstrtol(buf, 10, &mmc_card_addr);
		if(ret){
			pr_err("[HW]:%s:parse card_adrr error buf=%s.", __func__, buf);
			goto err;
		}

		mmc_card_addr = (long)(mmc_card_addr ^ CARD_ADDR_MAGIC);
		card = (struct mmc_card *)mmc_card_addr;

		if (mmc_card_mmc(card))
		{
			if((g_hw_test_debugfs_entries.hw_test_entry = debugfs_create_file("hw_aging_test",
							S_IFREG|S_IRWXU|S_IRGRP|S_IROTH, card->debugfs_root, card, &hw_mmc_test_fops_test)) == NULL){
				pr_err("[HW]:%s:debugfs_create_file(hw_test) failed.", __func__);
				goto err;
			}
		}

		if (mmc_card_mmc(card)){
			if((g_hw_test_debugfs_entries.hw_testlist_entry = debugfs_create_file("hw_aging_testlist",
							S_IFREG|S_IRWXU|S_IRGRP|S_IROTH, card->debugfs_root, card, &hw_mmc_test_fops_testlist)) == NULL){
				pr_err("[HW]:%s:debugfs_create_file(hw_testlist) failed.", __func__);
				goto err;
			}
		}

		return 0;

err:
		if(g_hw_test_debugfs_entries.hw_test_entry)
			debugfs_remove(g_hw_test_debugfs_entries.hw_test_entry);

		if(g_hw_test_debugfs_entries.hw_testlist_entry)
			debugfs_remove(g_hw_test_debugfs_entries.hw_testlist_entry);

		return -1;
	}


static void __exit mmc_hw_again_test_exit(void)
	{
		if(NULL!=host_clock_test){
			_add_clock_table(host_clock_test,0);
		}
		debugfs_remove(g_hw_test_debugfs_entries.hw_test_entry);
		debugfs_remove(g_hw_test_debugfs_entries.hw_testlist_entry);
		return;
	}

	module_init(mmc_hw_aging_test_init);
	module_exit(mmc_hw_again_test_exit);

	MODULE_LICENSE("GPL v2");
	MODULE_DESCRIPTION("MMC Huawei aging test");
#endif
