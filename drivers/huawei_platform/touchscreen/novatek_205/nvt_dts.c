#include "nvt.h"

int nvt_parse_dts(struct device *dev, struct nvt_ts_data * ts)
{

    struct device_node * np = dev->of_node;
    int ret = 0;
    uint32_t value = 0;
    int i=0;

    ret = of_get_named_gpio_flags(np,"nvt,irq-gpio",0,NULL);
    if(ret < 0){
        tp_log_err("%s: Failed to read irq gpio %d",__func__, ts->irq_gpio);
        return ret;      
    }else{
        ts->irq_gpio  = ret;
    }

    ret = of_get_named_gpio_flags(np,"nvt,reset-gpio",0,NULL);
    if(ret < 0){
        tp_log_err("%s: Failed to read reset gpio %d",__func__, ts->reset_gpio);
        return ret;      
    }else{
        ts->reset_gpio  = ret;
    }

    ret = of_get_named_gpio_flags(np,"nvt,vdd-gpio",0,NULL);
    if(ret < 0){
        tp_log_err("%s: Failed to read vdd gpio %d",__func__, ts->vdd_gpio);
        return ret;      
    }else{
        ts->vdd_gpio  = ret;
    }   

    ret = of_property_read_u32(np, "nvt,abs_x_max", &value);
    if(ret){
         tp_log_err("%s: Failed to read abs_x_max %d",__func__, value);
         return ret;
    }else{
        ts->abs_x_max = (uint16_t)value;
    }
    
    ret = of_property_read_u32(np, "nvt,abs_y_max", &value);
    if(ret){
         tp_log_err("%s: Failed to read abs_y_max %d",__func__, value);
         return ret;
    }else{
        ts->abs_y_max = (uint16_t)value;
    }

    ret = of_property_read_u32(np, "nvt,max_touch_num", &value);
    if(ret){
         tp_log_err("%s: Failed to read max_touch_num %d",__func__, value);
         return ret;
    }else{
        ts->max_touch_num = (uint8_t)value;
    }

    ret = of_property_read_u32(np, "nvt,max_key_num", &value);
    if(ret){
         tp_log_err("%s: Failed to read max_key_num %d",__func__, value);
         return ret;
    }else{
        ts->max_key_num = (uint8_t)value;
    }

    if(ts->max_key_num < 0){
        tp_log_err("%s: max_key_num can't be negtive: %d",__func__, ts->max_key_num);
        return ts->max_key_num;
    }else{
        ts->touch_key_array = kmalloc(sizeof(uint16_t)*ts->max_key_num,GFP_KERNEL);
        if(NULL == ts->touch_key_array){
            tp_log_err("%s:touch_key_array malloc fail!\n",__func__);
            return -ENOMEM;
        }
    }

    ret = of_property_read_u32(np, "nvt,easy_wakeup_supported_gestures", &value);
    if(ret){
         tp_log_err("%s: Failed to read supported gestures %d",__func__, value);
         ts->easy_wakeup_supported_gestures = 0;
    }else{
        ts->easy_wakeup_supported_gestures = (uint16_t)value;
    }
    ts->easy_wakeup_gestures = 0;
    ts->easy_wakeup_gestures_nova = 0;

    ret = of_property_read_u32_array(np, "nvt,easy_wakeup_gesture_keys", 
        ts->easy_wakeup_gesture_keys,12);
    if(ret){
         tp_log_err("%s: Failed to read wakeup gesture keys %d",__func__, value);
         memset(ts->easy_wakeup_gesture_keys,0,12);
    }else{
        for(i=0; i<12; i++)
        {
            tp_log_info("%s: easy_wakeup_gesture_keys[%d]=%d\n", 
                __func__, i, ts->easy_wakeup_gesture_keys[i]);
        }
    }

    ret = of_property_read_string(np,"nvt,product_name", &ts->product_name);
    if(ret)
    {
        tp_log_err("%s %d: no product_id configer in dtsi, use default value\n", __func__, __LINE__);
        ts->product_name= "unkown_product";
    }
    
    tp_log_info("%s: irq-gpio=%d, irq-rst=%d, vdd-gpio=%d,"
                            "x-max=%d, y-max=%d,"
                            "touch-max=%d, key-max=%d,"
                            "easy_wakeup_supported_gestures=%d"
                            "product_name = %s\n",
                            __func__, ts->irq_gpio, ts->reset_gpio, ts->vdd_gpio,
                            ts->abs_x_max, ts->abs_y_max,
                            ts->max_touch_num, ts->max_key_num,
                            ts->easy_wakeup_supported_gestures,
                            ts->product_name);

    return 0;
}


