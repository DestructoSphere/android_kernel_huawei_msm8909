/*
 * Copyright (C) huawei company
 *
 * This    program    is free    software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public    License    version    2 as
 * published by    the Free Software Foundation.
 */
 
#include <linux/init.h>
#include <sound/hw_audio_log.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <linux/slab.h> 
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/of.h>

#ifdef CONFIG_HUAWEI_DSM
#include <sound/hw_audio_info.h>


static struct dsm_dev audio_dsm_info = 
{
    .name = DSM_AUDIO_MOD_NAME,
    .fops = NULL,
    .buff_size = DSM_AUDIO_BUF_SIZE,
};

static struct dsm_client *audio_dclient = NULL;
#endif

/* Audio property is an unsigned 32-bit integer stored in the variable of audio_property.
   The meaning of audio_property is defined as following MACROs in groups of 4 bits */

/* Delete ambiguous builtin mic type */
/* Bit4 ~ bit7:
   Actually existing mics on the phone, it's NOT relevant to fluence. Using one bit to denote 
   the existence of one kind of mic, possible mics are:
     master mic: the basic mic used for call and record, if it doesn't exist means the 
                 config or software is wrong.
     secondary mic: auxiliary mic, usually used for fluence or paired with speaker in 
                    handsfree mode, it's possible that this mic exist but fluence isn't enabled.
     error mic: used in handset ANC. */
#define AUDIO_PROP_MASTER_MIC_EXIST_NODE    "builtin-master-mic-exist"
#define AUDIO_PROP_SECONDARY_MIC_EXIST_NODE "builtin-2nd-mic-exist"
#define AUDIO_PROP_ERROR_MIC_EXIST_NODE     "builtin-error-mic-exist"
#define AUDIO_PROP_MASTER_MIC_EXIST_MASK    0x00000010
#define AUDIO_PROP_SECONDARY_MIC_EXIST_MASK 0x00000020
#define AUDIO_PROP_ERROR_MIC_EXIST_MASK     0x00000040

/* Bit12 ~ bit15:
   Denote which mic would be used in hand held mode, please add as needed */
#define AUDIO_PROP_HANDHELD_MASTER_MIC_NODE "hand_held_master_mic_strategy"
#define AUDIO_PROP_HANDHELD_DUAL_MIC_NODE "hand_held_dual_mic_strategy"
#define AUDIO_PROP_HANDHELD_AANC_MIC_NODE "hand_held_aanc_mic_strategy"
#define AUDIO_PROP_HANDHELD_MASTER_MIC 0x00001000
#define AUDIO_PROP_HANDHELD_DUAL_MIC 0x00002000
#define AUDIO_PROP_HANDHELD_AANC_MIC 0x00004000

#define PRODUCT_IDENTIFIER_NODE             "product-identifier"
#define PRODUCT_IDENTIFIER_BUFF_SIZE        64
#define AUD_PARAM_VER_NODE             "aud_param_ver"
#define AUD_PARAM_VER_BUFF_SIZE        32

#define SPEAKER_PA_NODE             "speaker-pa"
#define SPEAKER_PA_BUFF_SIZE        32

#define AUDIO_PROP_BTSCO_NREC_ADAPT_MASK 0xf0000000
#define AUDIO_PROP_BTSCO_NREC_ADAPT_OFF  0x10000000
#define AUDIO_PROP_BTSCO_NREC_ADAPT_ON   0x20000000

#define PRODUCT_NERC_ADAPT_CONFIG    "product-btsco-nrec-adapt"

/* Bit16 ~ bit19:
   Denote which mic would be used in loud speaker mode, please add as needed */
#define AUDIO_PROP_LOUDSPEAKER_MASTER_MIC_NODE "loud_speaker_master_mic_strategy"
#define AUDIO_PROP_LOUDSPEAKER_SECOND_MIC_NODE "loud_speaker_second_mic_strategy"
#define AUDIO_PROP_LOUDSPEAKER_ERROR_MIC_NODE "loud_speaker_error_mic_strategy"
#define AUDIO_PROP_LOUDSPEAKER_DUAL_MIC_NODE "loud_speaker_dual_mic_strategy"
#define AUDIO_PROP_LOUDSPEAKER_MASTER_MIC 0x00010000
#define AUDIO_PROP_LOUDSPEAKER_SECOND_MIC 0x00020000
#define AUDIO_PROP_LOUDSPEAKER_ERROR_MIC 0x00040000
#define AUDIO_PROP_LOUDSPEAKER_DUAL_MIC 0x00080000
/* Delete unused codes related to string_to_value function */

/**
* audio_property - Product specified audio properties
*/
static unsigned int audio_property = 0;

/**
* product_identifier - Product identifier, used for audio parameter auto-adapt
*/
static char product_identifier[PRODUCT_IDENTIFIER_BUFF_SIZE] = "default";

static char speaker_pa[SPEAKER_PA_BUFF_SIZE] = "none";
static bool tfa9895_flag = false;

static char aud_param_ver[AUD_PARAM_VER_BUFF_SIZE] = "default";

static ssize_t audio_property_show(struct device_driver *driver, char *buf)
{
    if(NULL != buf)
    {
        /* The size of buf should be one page(PAGE_SIZE), which will be surely 
           enough to hold the string converted from a 32 bit unsigned integer. 
           So omit unnecessary overflow check */
        return snprintf(buf, PAGE_SIZE, "%08X\n", audio_property);
    }

    return 0;
}
DRIVER_ATTR(audio_property, 0444, audio_property_show, NULL);

static ssize_t product_identifier_show(struct device_driver *driver, char *buf)
{
    if(NULL != buf)
    {
        /* Newline is not appended on purpose, for convenience of reader programs */
        snprintf(buf, PAGE_SIZE, "%s", product_identifier);
        return strlen(buf);
    }

    return 0;
}
DRIVER_ATTR(product_identifier, 0444, product_identifier_show, NULL);

static ssize_t speaker_pa_show(struct device_driver *driver, char *buf)
{
    if(NULL != buf)
    {
        /* Newline is not appended on purpose, for convenience of reader programs */
        snprintf(buf, PAGE_SIZE, "%s", speaker_pa);
        return strlen(buf);
    }

    return 0;
}
DRIVER_ATTR(speaker_pa, 0444, speaker_pa_show, NULL);

/**
* audiopara_version_show - used for cat the value of node aud_param_ver
*/
static ssize_t audiopara_version_show(struct device_driver *driver, char *buf)
{
    if(NULL != buf)
    {
        return snprintf(buf, PAGE_SIZE, "%s", aud_param_ver);
    }
    return 0;
}
DRIVER_ATTR(aud_param_ver, 0444, audiopara_version_show, NULL);
/* Default audio log level */
int audio_log_level = AUDIO_LOG_LEVEL_INFO;

/**
* audio_property - Product specified audio properties
*/
static ssize_t audio_log_level_show(struct device_driver *driver, char *buf)
{
    char level_char = ' ';

    if(NULL == buf)
    {
        ad_loge("%s get NULL argument\n", __func__);
        return 0;
    }

    switch(audio_log_level)
    {
        case AUDIO_LOG_LEVEL_VERBOSE:
            level_char = 'V';
            break;
        case AUDIO_LOG_LEVEL_DEBUG:
            level_char = 'D';
            break;
        case AUDIO_LOG_LEVEL_INFO:
            level_char = 'I';
            break;
        case AUDIO_LOG_LEVEL_NOTICE:
            level_char = 'N';
            break;
        case AUDIO_LOG_LEVEL_WARNING:
            level_char = 'W';
            break;
        case AUDIO_LOG_LEVEL_ERROR:
            level_char = 'E';
            break;
        case AUDIO_LOG_LEVEL_NONE:
            level_char = '0';
            break;
        default:
            level_char = ' ';
            ad_loge("Unkown audio log level\n");
            break;
    }

    ad_logi("Current log level is %d:%c\n", audio_log_level, level_char);
    
    /* Those logs are for log level confirm and test */
    ad_logv("This is verbose log\n");
    ad_logd("This is debug log\n");
    ad_logi("This is info log\n");
    ad_logn("This is notice log\n");
    ad_logw("This is warning log\n");
    ad_loge("This is error log\n");
    return snprintf(buf, PAGE_SIZE, "%d:%c", audio_log_level, level_char);
}

/**
* audio_property - Product specified audio properties
*/
static ssize_t audio_log_level_store(struct device_driver *driver, const char *buf, size_t count)
{
    char level_char;
    int new_log_level = audio_log_level;
    
    if((NULL == buf) || (count < 1))
    {
        ad_loge("%s get illegal arguments\n", __func__);
        return -EINVAL;
    }
    
    level_char = buf[0];
    ad_logi("Get audio log level char %c\n", level_char);

    switch(level_char)
    {
        case 'V':
            new_log_level = AUDIO_LOG_LEVEL_VERBOSE;
            break;
        case 'D':
            new_log_level = AUDIO_LOG_LEVEL_DEBUG;
            break;
        case 'I':
            new_log_level = AUDIO_LOG_LEVEL_INFO;
            break;
        case 'N':
            new_log_level = AUDIO_LOG_LEVEL_NOTICE;
            break;
        case 'W':
            new_log_level = AUDIO_LOG_LEVEL_WARNING;
            break;
        case 'E':
            new_log_level = AUDIO_LOG_LEVEL_ERROR;
            break;
        case '0':
            new_log_level = AUDIO_LOG_LEVEL_NONE;
            break;
        default:
            ad_loge("Unkown audio log level character: %c\n", level_char);
            new_log_level = audio_log_level;
            break;
    }
    
    if(audio_log_level != new_log_level)
    {
        ad_logi("Change audio log level from %d to %d\n", audio_log_level, new_log_level);
        audio_log_level = new_log_level;
    }
    else
    {
        ad_logi("Audio log level remains %d\n", audio_log_level);
    }
    return count;
}
DRIVER_ATTR(audio_log_level, 0644, audio_log_level_show, audio_log_level_store);

static struct attribute *audio_attrs[] = {
    &driver_attr_audio_property.attr,
    &driver_attr_product_identifier.attr,
    &driver_attr_aud_param_ver.attr,
    &driver_attr_speaker_pa.attr,
    &driver_attr_audio_log_level.attr,
    NULL,
};

#ifdef CONFIG_HUAWEI_DSM
int audio_dsm_register(void)
{
    if (NULL != audio_dclient) 
    {
        return 0;
    }

    audio_dclient = dsm_register_client(&audio_dsm_info);
    if(NULL == audio_dclient)
    {
        printk("audio_dclient register failed!\n");
    }

    return 0;
}

int audio_dsm_report_num(int error_no, unsigned int mesg_no)
{
    int err = 0;

    if(NULL == audio_dclient)
    {
        printk("%s: audio_dclient did not register!\n", __func__);
        return -1;
    }

    err = dsm_client_ocuppy(audio_dclient);
    if(0 != err)
    {
        printk("%s: user buffer is busy!\n", __func__);
        return -1;
    }

    printk("%s: after dsm_client_ocuppy, error_no=0x%x, mesg_no=0x%x!\n",
        __func__, error_no, mesg_no);
    
    err = dsm_client_record(audio_dclient, "Message code = 0x%x.\n", mesg_no);

    dsm_client_notify(audio_dclient, error_no);

    return 0;
}

int audio_dsm_report_info(int error_no, char *fmt, ...)
{
    int err = 0;
    int ret = 0;
    char dsm_report_buffer[DSM_REPORT_BUF_SIZE] = {0};
    va_list args;

    if(NULL == audio_dclient)
    {
        printk("%s: audio_dclient did not register!\n", __func__);
        return -1;
    }

    if(error_no < DSM_AUDIO_ERROR_NUM)
    {
        printk("%s: input error_no err!\n", __func__);
        return -1;
    }

    va_start(args, fmt);
    ret = vsnprintf(dsm_report_buffer, DSM_REPORT_BUF_SIZE, fmt, args);
    va_end(args);
    
    err = dsm_client_ocuppy(audio_dclient);
    if(0 != err)
    {
        printk("%s: user buffer is busy!\n", __func__);
        return -1;
    }

    printk("%s: after dsm_client_ocuppy, dsm_error_no = %d, %s\n",
            __func__, error_no, dsm_report_buffer);

    dsm_client_record(audio_dclient, "%s\n", dsm_report_buffer);
    
    dsm_client_notify(audio_dclient, error_no);
    
    return 0;
    
}

#else
int inline audio_dsm_register()
{
    return 0;
}

int inline audio_dsm_report_num(int error_no, int mesg_no)
{
    return 0;
}

int audio_dsm_report_info(int error_no, char *fmt, ...)
{
    return 0;
}

#endif


static struct attribute_group audio_group = {
    .name ="hw_audio_info",
    .attrs = audio_attrs,
}; 

static const struct attribute_group *groups[] = {
    &audio_group,
    NULL,
};

static struct of_device_id audio_info_match_table[] = {
    { .compatible = "huawei,hw_audio_info",},
    { },
};

bool get_tfa9895_flag(void)
{
   if(tfa9895_flag)
      return true;
   return false;   
}
EXPORT_SYMBOL_GPL(get_tfa9895_flag);

static int audio_info_probe(struct platform_device *pdev)
{
    int ret;
    const char *string;
    
    if(NULL == pdev)
    {
        ad_loge( "huawei_audio: audio_info_probe failed, pdev is NULL\n");
        return 0;
    }
    
    if(NULL == pdev->dev.of_node)
    {
        ad_loge( "huawei_audio: audio_info_probe failed, of_node is NULL\n");
        return 0;
    }

    /* Delete ambiguous builtin mic type */

    if(of_property_read_bool(pdev->dev.of_node, AUDIO_PROP_MASTER_MIC_EXIST_NODE))
    {
        audio_property |= AUDIO_PROP_MASTER_MIC_EXIST_MASK;
    }
    else
    {
        ad_loge( "huawei_audio: check mic config, no master mic found\n");
        audio_dsm_report_info(DSM_AUDIO_CARD_LOAD_FAIL_ERROR_NO, "master mic not found!");
    }
    
    if(of_property_read_bool(pdev->dev.of_node, AUDIO_PROP_SECONDARY_MIC_EXIST_NODE))
    {
        audio_property |= AUDIO_PROP_SECONDARY_MIC_EXIST_MASK;
    }
    
    if(of_property_read_bool(pdev->dev.of_node, AUDIO_PROP_ERROR_MIC_EXIST_NODE))
    {
        audio_property |= AUDIO_PROP_ERROR_MIC_EXIST_MASK;
    }
    
    if(of_property_read_bool(pdev->dev.of_node, AUDIO_PROP_HANDHELD_MASTER_MIC_NODE))
    {
        audio_property |= AUDIO_PROP_HANDHELD_MASTER_MIC;
    }

    if(of_property_read_bool(pdev->dev.of_node, AUDIO_PROP_HANDHELD_DUAL_MIC_NODE))
    {
        audio_property |= AUDIO_PROP_HANDHELD_DUAL_MIC;
    }

    if(of_property_read_bool(pdev->dev.of_node, AUDIO_PROP_HANDHELD_AANC_MIC_NODE))
    {
        audio_property |= AUDIO_PROP_HANDHELD_AANC_MIC;
    }

    if(of_property_read_bool(pdev->dev.of_node, AUDIO_PROP_LOUDSPEAKER_MASTER_MIC_NODE))
    {
        audio_property |= AUDIO_PROP_LOUDSPEAKER_MASTER_MIC;
    }

    if(of_property_read_bool(pdev->dev.of_node, AUDIO_PROP_LOUDSPEAKER_SECOND_MIC_NODE))
    {
        audio_property |= AUDIO_PROP_LOUDSPEAKER_SECOND_MIC;
    }

	if(of_property_read_bool(pdev->dev.of_node, AUDIO_PROP_LOUDSPEAKER_DUAL_MIC_NODE))
    {
        audio_property |= AUDIO_PROP_LOUDSPEAKER_DUAL_MIC;
    }
    if(of_property_read_bool(pdev->dev.of_node, AUDIO_PROP_LOUDSPEAKER_ERROR_MIC_NODE))
    {
        audio_property |= AUDIO_PROP_LOUDSPEAKER_ERROR_MIC;
    }
    
    string = NULL;
    ret = of_property_read_string(pdev->dev.of_node, PRODUCT_IDENTIFIER_NODE, &string);
    if(ret || (NULL == string))
    {
        ad_loge( "huawei_audio: of_property_read_string product-identifier failed %d\n", ret);
    }
    else
    {
        memset(product_identifier, 0, sizeof(product_identifier));
        strncpy(product_identifier, string, sizeof(product_identifier) - 1);
    }

    string = NULL;
    ret = of_property_read_string(pdev->dev.of_node, SPEAKER_PA_NODE, &string);
    if(ret || (NULL == string))
    {
        ad_loge( "huawei_audio: of_property_read_string speaker-pa failed %d\n", ret);
    }
    else
    {
        memset(speaker_pa, 0, sizeof(speaker_pa));
        strncpy(speaker_pa, string, sizeof(speaker_pa) - 1);
    }
    if(strcmp(speaker_pa, "tfa9895"))
    {
        ad_loge("tfa9895_flag = false\n");
        tfa9895_flag = false;
    }
    else
    {
        ad_loge("tfa9895_flag = true\n");
        tfa9895_flag = true;
    }

    if(false == of_property_read_bool(pdev->dev.of_node, PRODUCT_NERC_ADAPT_CONFIG))
    {
        ad_loge( "huawei_audio: of_property_read_bool PRODUCT_NERC_ADAPT_CONFIG failed %d\n", ret);
        audio_property |= (AUDIO_PROP_BTSCO_NREC_ADAPT_OFF & AUDIO_PROP_BTSCO_NREC_ADAPT_MASK);
    }
    else       
    {        
        audio_property |= ( AUDIO_PROP_BTSCO_NREC_ADAPT_ON & AUDIO_PROP_BTSCO_NREC_ADAPT_MASK);
    }
    
    string = NULL;
    ret = of_property_read_string(pdev->dev.of_node, AUD_PARAM_VER_NODE, &string);
    if(ret || (NULL == string))
    {
        ad_loge( "huawei_audio: of_property_read_string aud_param_ver failed %d\n", ret);
    }
    else
    {
        memset(aud_param_ver, 0, sizeof(aud_param_ver));
        strncpy(aud_param_ver, string, sizeof(aud_param_ver) - 1);
    }

    return 0;
}

static struct platform_driver audio_info_driver = {
    .driver = {
        .name  = "hw_audio_info",
        .owner  = THIS_MODULE,
        .groups = groups,
        .of_match_table = audio_info_match_table,
    },

    .probe = audio_info_probe,
    .remove = NULL,
};

static int __init audio_info_init(void)
{
    return platform_driver_register(&audio_info_driver);
}

static void __exit audio_info_exit(void)
{
    platform_driver_unregister(&audio_info_driver);
}

module_init(audio_info_init);
module_exit(audio_info_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Huawei audio info");
MODULE_AUTHOR("duhongyan<hongyan.du@huawei.com>");
