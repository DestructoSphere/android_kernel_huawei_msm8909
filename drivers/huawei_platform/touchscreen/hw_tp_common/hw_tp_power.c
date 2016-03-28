/* add first time by yanghaizhou 20140527 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/device.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include <huawei_platform/touchscreen/hw_tp_common.h>
#include <huawei_platform/touchscreen/hw_tp_power.h>

#define MAX_POWER_NODE_NUM (4)
#define MAX_PNODE_NAME_LEN  (20)
#define MAX_POWER_TMP_STR_LEN (256)

enum hw_tp_power_type {
    NONE_SUPPLY,
    GPIO_H_SUPPLY,
    GPIO_L_SUPPLY,
    PMU_SUPPLY,
    TYPE_NUM,    //keep as a counter
};

struct hw_tp_power_node{
    u32 power_type;
    char const * power_name;
    u32 power_gpio;
    u32 power_volt;
    u32 power_dlay;
    bool is_initialized;
    struct regulator * preg;
    struct hw_tp_power_node * next;
    struct device_node * np;
};

struct hw_tp_power_data {
    u32     pw_cnt;
    bool    pw_ready;
    struct hw_tp_power_node * pnode_list;
    struct device * dev;
    struct device_node * np;
};


static struct hw_tp_power_data * pwdata = NULL;

/*****************************************************************
Parameters:
    dev: consumer device of the power
    Return: zero if succeed, otherwise a negtive value
Description   :  parse config from dev and init configs
*****************************************************************/
struct hw_tp_power_data * hw_tp_get_power_data(void)
{
    if(pwdata) {
        return pwdata;
    }

    pwdata = kzalloc(sizeof(*pwdata),GFP_KERNEL);
    if(!pwdata) {
        tp_log_err("%s#%d: kzalloc for pwdata fail!\n",__func__,__LINE__);
        return NULL;
    }

    pwdata->dev = NULL;
    pwdata->np = NULL;
    pwdata->pnode_list = NULL;
    pwdata->pw_cnt = 0;
    pwdata->pw_ready = false;

    return pwdata;
}


/*
************************************************************************
*InputParms:
***dev - consumer device of the regulator
***np - a pointer to hw_tp_power_data where power configs saved
***name - name of regulator to get
*return: pointer to the regulator we got
************************************************************************
*/
struct regulator * hw_tp_power_get_regulator(struct device * dev, 
    struct device_node * np, const char * name)
{
    struct regulator * preg = NULL;
    struct device_node * tnp = NULL;

    if(!dev || !np || !name) {
        tp_log_err("%s#%d: invalid args: dev=%p, np=%p, name=%p\n",
            __func__,__LINE__,dev,np,name);
        return NULL;
    }

    tnp = dev->of_node;
    dev->of_node = np;
    preg = regulator_get(dev,name);
    dev->of_node = tnp;

    return preg;
}

/*
************************************************************************
*/
void hw_tp_power_release(void){
    struct hw_tp_power_data * pd = NULL;
    struct hw_tp_power_node * pn = NULL;
    struct hw_tp_power_node * pn2 = NULL;

    pd = hw_tp_get_power_data();
    if(!pd) {
        tp_log_err("%s#%d: pd is NULL!\n",__func__,__LINE__);
        return;
    }
    
    pn = pd->pnode_list;
    pd->pw_ready = false;
    pd->dev = NULL;
    pd->np = NULL;
    pd->pw_cnt = 0;
    pd->pnode_list = NULL;

    while(pn) {
        if (pn->is_initialized) {
            switch (pn->power_type) {
                case GPIO_H_SUPPLY:
                case GPIO_L_SUPPLY:
                    gpio_free(pn->power_gpio);
                    break;
                case PMU_SUPPLY:
                    regulator_put(pn->preg);
                    break;
            }
        }
        pn2 = pn;
        pn = pn->next;
        kfree(pn2);
    }
   
    tp_log_warning("%s#%d: hw_tp_power released!\n",__func__,__LINE__);
    return;
}
EXPORT_SYMBOL(hw_tp_power_release);


/*
************************************************************************
*InputParms:
***pd - a pointer to hw_tp_power_data where power configs saved
*return: number of power configs parsed
************************************************************************
*/
int hw_tp_power_parse_config(struct hw_tp_power_data * pd)
{
    int ret = 0;
    int i = 0;
    u32 value;
    struct hw_tp_power_node * pn = NULL;
    struct device_node * tnp = NULL;
    char pname[MAX_PNODE_NAME_LEN] = {0};
    struct device_node * np = pd->np;

    pd->pw_cnt = 0;
    pn = kzalloc(sizeof(*pn),GFP_KERNEL);
    if(!pn){
        tp_log_err("%s#%d: kzalloc of power_node failed!\n",__func__,__LINE__);
        goto hw_tp_parse_confg_exit;
    }
    memset(pn, 0, sizeof(*pn));
    pd->pnode_list = pn;

    for_each_child_of_node(np,tnp) {
        snprintf(pname,sizeof(pname),tnp->name);
        /* alloc memory for a new power node*/
        if(i != 0) {
            pn->next = kzalloc(sizeof(*pn),GFP_KERNEL);
            if(!pn->next) {
                tp_log_err("%s#%d: kzalloc of pn->next fail!\n",__func__,__LINE__);
                ret = -ENOMEM;
                goto hw_tp_parse_confg_exit;
            }
            memset(pn->next,0,sizeof(*pn));
            pn = pn->next;
        }
        i++;
        pn->np = tnp;
        
        /* read power name */
        ret = of_property_read_string(tnp, "power-name",&pn->power_name);
        if(ret) {
            tp_log_warning("%s#%d: %s -> power-name empty!\n",
                __func__,__LINE__,pname);
            pn->power_name = NULL;
        } else {
            tp_log_info("%s#%d: %s -> power_name = %s\n",
                __func__,__LINE__,pname,pn->power_name);
        }

        /* read power type */
        ret = of_property_read_u32(tnp,"power-type",&value);
        if(ret){
            tp_log_err("%s#%d: read power-type from %s fai!ret = %d\n",
            __func__,__LINE__,pname, ret);
            pn->power_type = NONE_SUPPLY;
        } else {
            pn->power_type = (u32)value;
            tp_log_info("%s#%d: %s -> power_type = %d\n",
                __func__,__LINE__,pname, pn->power_type);
        }

        /* read power gpio */
        if (of_find_property(tnp, "power-gpio", NULL)) {
            pn->power_gpio = of_get_named_gpio_flags(tnp, "power-gpio",0, NULL);
            tp_log_info("%s#%d: %s -> power_gpio = %d\n",
                 __func__,__LINE__,pname, pn->power_gpio);
        } else {
            tp_log_warning("%s#%d: read power-gpio from %s fail!ret = %d\n",
            __func__,__LINE__,pname, ret);
            pn->power_gpio = 0;
        }

        /* read power-voltage */
        ret = of_property_read_u32(tnp,"power-volt",&value);
        if(ret){
            tp_log_warning("%s#%d: read power-volt from %s fail!ret = %d\n",
            __func__,__LINE__,pname, ret);
            pn->power_volt = 0;
        } else {
            pn->power_volt= (u32)value;
            tp_log_info("%s#%d: %s -> power_volt = %d\n",
                __func__,__LINE__,pname, pn->power_volt);
        }

        /* read delay-time of power-node */
        ret = of_property_read_u32(tnp,"power-dlay",&value);
        if(ret){
            tp_log_warning("%s#%d: read power-dlay from %s fail!ret = %d\n",
            __func__,__LINE__,pname, ret);
            pn->power_dlay = 0;
        } else {
            pn->power_dlay= (u32)value;
            tp_log_info("%s#%d: %s -> power_dlay = %d\n",
                __func__,__LINE__,pname, pn->power_dlay);
        }
    }
    
hw_tp_parse_confg_exit:
    if(pn) {
        pn->next = NULL;
    }
    tp_log_info("%s#%d: power configs parse done!,ret=%d\n",__func__,__LINE__,ret);
    pd->pw_cnt = i;
    return pd->pw_cnt;
}

/*
************************************************************************
*InputParms:
***pd - a pointer to hw_tp_power_data where power configs saved
*return: number of power configs initialized successful
************************************************************************
*/
int hw_tp_power_init_config(struct hw_tp_power_data * pd)
{
    int ret = 0;
    struct device_node * np = NULL;
    struct hw_tp_power_node * pn = NULL;
    struct device * dev = NULL;

    if(!pd) {
        tp_log_err("%s#%d: pd is NULL!\n",__func__,__LINE__);
        ret = -EINVAL;
        goto hw_tp_power_init_config_exit;
    }

    np = pd->np;
    pn = pd->pnode_list;
    dev = pd->dev;
    if(!np || !pn || !dev) {
        ret = -EINVAL;
        tp_log_err("%s#%d: pn=%p, pn=%p, dev=%p\n",
            __func__,__LINE__,np,pn,dev);
        goto hw_tp_power_init_config_exit;
    }

    while(pn) {
        tp_log_info("%s#%d: init config of %s\n",
            __func__,__LINE__,pn->power_name);
        switch(pn->power_type) {
            case GPIO_H_SUPPLY:
            case GPIO_L_SUPPLY:
                ret = gpio_request(pn->power_gpio,dev_name(dev));
                if(ret) {
                    tp_log_err("%s#%d: %s -> request gpio fail!\n",
                        __func__,__LINE__,pn->power_name);
                    goto hw_tp_power_init_config_exit;
                }
                pn->is_initialized = true;
                break;
            case PMU_SUPPLY:
                //pn->preg = hw_tp_power_get_regulator(dev,np,pn->power_name);
                tp_log_info("testing##: %s\n",pn->np->name);
                pn->preg = hw_tp_power_get_regulator(dev,pn->np,pn->power_name);
                ret = IS_ERR(pn->preg);
                if(ret) {
                    tp_log_err("%s#%d: %s -> get preg fail!\n",
                        __func__,__LINE__,pn->power_name);
                    goto hw_tp_power_init_config_exit;
                }
                ret = regulator_set_voltage(pn->preg,pn->power_volt,pn->power_volt);
                if(ret) {
                    tp_log_err("%s#%d: %s -> get preg fail!\n",
                        __func__,__LINE__,pn->power_name);
                    goto hw_tp_power_init_config_exit;
                }
                pn->is_initialized = true;
            default:
                tp_log_warning("%s#%d:%s -> invalid type: %d\n",
                    __func__,__LINE__,pn->power_name,pn->power_type);
                break;
        }
        pn = pn->next;
    }

    pd->pw_ready = true;
hw_tp_power_init_config_exit:
    tp_log_info("%s#%d: power configs init done!,ret=%d\n",__func__,__LINE__,ret);
    return ret;
}

void hw_tp_power_off(void);
/*****************************************************************
Parameters:
    dev: consumer device of the power
    Return: zero if succeed, otherwise a negtive value
Description   :  parse config from dev and init configs
*****************************************************************/
int hw_tp_power_init(struct device * dev){
    int ret = 0;
    struct hw_tp_power_data * pd = NULL;

    if(!dev || !dev->of_node){
        tp_log_err("%s#%d: dev  or dev->of_node is NULL!\n",__func__,__LINE__);
        return -EINVAL;
    }

    /* get global power data throw this func */
    pd = hw_tp_get_power_data();
    if (!pd) {
        tp_log_err("%s#%d: get pwdata fail!\n",__func__,__LINE__);
        return -ENOMEM;
    } else if (pd->pw_ready){
        tp_log_warning("%s#%d: power already init, so release first...\n",
            __func__,__LINE__);
        hw_tp_power_release();
    }

    /* get hw_tp_power as a root node */
    pd->np = of_find_node_by_name(dev->of_node,"huawei-tp-power");
    if(!pd->np){
        tp_log_err("%s#%d: get hw_tp_power node failed!\n",__func__,__LINE__);
        ret = -ENODEV;
        goto hw_tp_power_init_exit;
    }
    pd->dev = dev;

    /* parse power configs from dts */
    ret = hw_tp_power_parse_config(pd);
    if(ret) {
        tp_log_info("%s#%d: %d power configs found!\n",__func__,__LINE__,ret);
    } else {
        tp_log_warning("%s#%d: no power configs found!\n",__func__,__LINE__);
        ret = -EINVAL;
        goto hw_tp_power_init_exit;
    }

    /* init power configs: request gpio, get regulaor and set ....*/
    ret = hw_tp_power_init_config(pd);
    if(ret) {
        tp_log_warning("%s#%d: power configs init fail, ret=%d\n",
            __func__,__LINE__,ret);
        ret = -EAGAIN;
        goto hw_tp_power_init_exit;
    } else {
        tp_log_info("%s#%d: power config init success!\n",__func__,__LINE__);
    }

    //hw_tp_power_off();
    tp_log_info("%s#%d: power init success!\n",__func__,__LINE__);
    return 0;
    
hw_tp_power_init_exit:
    tp_log_info("%s#%d: power init fail!\n",__func__,__LINE__);
    hw_tp_power_release();
    return ret;
}
EXPORT_SYMBOL(hw_tp_power_init);


/*
************************************************************************
*/
void hw_tp_power_on(void) {
    int ret = 0;
    struct hw_tp_power_data * pd = NULL;
    struct hw_tp_power_node * pn =NULL;

    pd = hw_tp_get_power_data();
    if(!pd) {
        tp_log_err("%s#%d: get power data fail!\n",__func__,__LINE__);
        return;
    }

    tp_log_info("%s#%d: tp_power_count=%d, tp_power_ready=%d\n",
        __func__,__LINE__,pd->pw_cnt,pd->pw_ready);

    pn = pd->pnode_list;
    while(pn) {
        if (pn->is_initialized) {
            tp_log_info("%s#%d: begin to power on %s\n",
                __func__,__LINE__,pn->power_name);
            switch (pn->power_type) {
                case GPIO_H_SUPPLY:
                    gpio_set_value(pn->power_gpio,1);
                    break;
                case GPIO_L_SUPPLY:
                    gpio_set_value(pn->power_gpio,0);
                    break;
                case PMU_SUPPLY:
                    ret = regulator_enable(pn->preg);
                    if(ret) {
                        tp_log_err("%s#%d: enable of %s fail!\n",
                            __func__,__LINE__,pn->power_name);
                    }
                    break;
                default:
                    tp_log_warning("%s#%d: invalid type: %s-%d\n",
                        __func__,__LINE__,pn->power_name,pn->power_type);
                    break;
            }
            if(pn->power_dlay) {
                mdelay(pn->power_dlay);
            }
        } else {
            tp_log_err("%s#%d: %s is not initialized!\n",
                __func__,__LINE__,pn->power_name);
        }
        pn = pn->next;
    }

    tp_log_info("%s#%d: power_on done!\n",__func__,__LINE__);
    return;
}
EXPORT_SYMBOL(hw_tp_power_on);

/*
************************************************************************
*/
void hw_tp_power_off(void) {
    int ret = 0;
    struct hw_tp_power_data * pd = NULL;
    struct hw_tp_power_node * pn =NULL;

    pd = hw_tp_get_power_data();
    if(!pd) {
        tp_log_err("%s#%d: get power data fail!\n",__func__,__LINE__);
        return;
    }


    tp_log_info("%s#%d: tp_power_count=%d, tp_power_ready=%d\n",
        __func__,__LINE__,pwdata->pw_cnt,pwdata->pw_ready);

    pn = pd->pnode_list;
    while(pn) {
        if (pn->is_initialized) {
            tp_log_info("%s#%d: begin to power off %s\n",
                __func__,__LINE__,pn->power_name);
            switch (pn->power_type) {
                case GPIO_H_SUPPLY:
                    gpio_set_value(pn->power_gpio,0);
                    break;
                case GPIO_L_SUPPLY:
                    gpio_set_value(pn->power_gpio,1);
                    break;
                case PMU_SUPPLY:
                    ret = regulator_disable(pn->preg);
                    if(ret) {
                        tp_log_err("%s#%d: disable of %s fail!\n",
                            __func__,__LINE__,pn->power_name);
                    }
                    break;
                default:
                    tp_log_warning("%s#%d: invalid type: %s-%d\n",
                        __func__,__LINE__,pn->power_name,pn->power_type);
                    break;
            }
            if(pn->power_dlay) {
                mdelay(pn->power_dlay);
            }
        } else {
            tp_log_err("%s#%d: %s is not initialized!\n",
                __func__,__LINE__,pn->power_name);
        }
        pn = pn->next;
    }

    tp_log_info("%s#%d: power_on done!\n",__func__,__LINE__);
    return;
}
EXPORT_SYMBOL(hw_tp_power_off);

bool hw_tp_power_is_ready(void) {
    return pwdata ? pwdata->pw_ready : false;
}
EXPORT_SYMBOL(hw_tp_power_is_ready);


void hw_tp_power_get_info(char * buf, ssize_t len) {
    struct hw_tp_power_data * pd = NULL;
    struct hw_tp_power_node * pn =NULL;
    char str_tmp[MAX_POWER_TMP_STR_LEN] = {0};
    ssize_t size = 0;
    
    pd = hw_tp_get_power_data();
    if(!pd) {
        tp_log_err("%s#%d: get power data fail!\n",__func__,__LINE__);
        return;
    }

     pn = pd->pnode_list;
     while(pn) {
        snprintf(str_tmp,sizeof(str_tmp),
            "power-name=%s; power-type=%d; power-gpio=%d; power-volt=%d; "
            "gpio-value=%d; is-initialized = %d; ",
            pn->power_name, pn->power_type, pn->power_gpio, pn->power_volt,
            gpio_get_value(pn->power_gpio), pn->is_initialized);
            strncat(buf,str_tmp, len-size);
            size  += strlen(str_tmp);
        if(pn->preg) {
            snprintf(str_tmp,sizeof(str_tmp),
            "power-reg-enable=%d; power-reg-value=%d\n",
            regulator_get_voltage(pn->preg), pn->power_volt);
            strncat(buf,str_tmp, len-size);
            size += strlen(str_tmp);
        } else {
            strncat(buf,"\n",1);
            size++;
        }
        pn = pn->next;
     }
}
EXPORT_SYMBOL(hw_tp_power_get_info);
