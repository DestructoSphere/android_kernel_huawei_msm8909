#include "nvt.h"
#include "nvt_input.h"

const unsigned char gesture_bit_mapping_table[12]={
    GESTURE_BIT_DOUBLE_CLICK,       // 0    
    GESTURE_BIT_SLIDE_RIGHT,          // 1
    GESTURE_BIT_SLIDE_LEFT,            // 2
    GESTURE_BIT_SLIDE_DOWN,         // 3
    GESTURE_BIT_SLIDE_UP,               // 4
    15,                                                   // 5
    GESTURE_BIT_WORD_O,                 // 6
    GESTURE_BIT_WORD_C,                 // 7
    GESTURE_BIT_WORD_e,                 // 8
    GESTURE_BIT_WORD_M,                // 9
    GESTURE_BIT_WORD_W,               // 10
    15,                                                  // 11
};
//transfer hw gresture bits order to nova order
uint16_t toNovaGestureBits(uint16_t hw_gesture_bits)
{
    int i;
    uint16_t nova_gesture_bits=0;

    for(i=0; i<(sizeof(gesture_bit_mapping_table)/sizeof(gesture_bit_mapping_table[0])); i++)
    {
        if(hw_gesture_bits & BIT(i))
        {
            nova_gesture_bits |= (uint16_t)BIT(gesture_bit_mapping_table[i]);
        }
    }
    
    return nova_gesture_bits;
}

#if MT_PROTOCOL_B
void mt_protocol_b(void * data, uint8_t* point_data)
{
    unsigned char i;

    int finger_cnt=0;
    unsigned int position = 0;    
    unsigned int input_x = 0;
    unsigned int input_y = 0;
    unsigned char input_w = 0;
    unsigned char input_id = 0;
    unsigned char input_Prs = 10;
    unsigned char press_id[10]={0};
    struct nvt_ts_data * ts = (struct nvt_ts_data *)data;

    for(i=0; i<ts->max_touch_num; i++){
        position = 1 + 6*i;        
        input_id = (unsigned char)(point_data[position+0]>>3);
        if(input_id > ts->max_touch_num) continue;

        tp_log_debug("line=%d,input_id = %d, i=%d\n",  __LINE__,input_id, i);
        
        if(((point_data[position]&0x07) == 0x01) || ((point_data[position]&0x07) == 0x02)){        //finger down (enter&moving)
            input_x = (unsigned int)(point_data[position+1]<<4) + (unsigned int) (point_data[position+3]>>4);
            input_y = (unsigned int)(point_data[position+2]<<4) + (unsigned int) (point_data[position+3]&0x0f);
            input_w = (unsigned int)(point_data[position+4])*5;

            tp_log_debug("%d\n",input_id-1);
            press_id[input_id-1] = 1;

            input_Prs += (input_x + input_y)%64;
            input_mt_slot(ts->input_dev, input_id-1);
            input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);
            input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
            input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);
            input_report_abs(ts->input_dev, ABS_MT_PRESSURE, input_Prs+input_id);
            //input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, input_w);
            //input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, input_id -1);

            finger_cnt++;
        }
    }   

    for(i=0; i<ts->max_touch_num; i++){
        if(press_id[i] != 1 ){
            input_mt_slot(ts->input_dev, i);
            input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
            //input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, -1);
        }
    }
    input_report_key(ts->input_dev, BTN_TOUCH, (finger_cnt>0)); //all finger up

}
#else
void mt_protocol_a(void * data, uint8_t* point_data)
{
    int i;
    int finger_cnt=0;
    unsigned int position = 0;    
    unsigned int input_x = 0;
    unsigned int input_y = 0;
    unsigned char input_w = 0;
    unsigned char input_id = 0;
    unsigned char input_Prs = 0;
    struct nvt_ts_data * ts = (struct nvt_ts_data *)data;

      for(i=0; i<ts->max_touch_num; i++){
        position = 1 + 6*i;
        input_id = (unsigned int)(point_data[position+0]>>3)-1;

        if((point_data[position]&0x07) == 0x03){        // finger up (break)
            continue;//input_report_key(ts->input_dev, BTN_TOUCH, 0);
        }
        else if(((point_data[position]&0x07) == 0x01) || ((point_data[position]&0x07) == 0x02)){    //finger down (enter&moving)            
            input_x = (unsigned int)(point_data[position+1]<<4) + (unsigned int) (point_data[position+3]>>4);
            input_y = (unsigned int)(point_data[position+2]<<4) + (unsigned int) (point_data[position+3]&0x0f);
            input_w = (unsigned int)(point_data[position+4])*5;
            input_Prs = (unsigned int)(point_data[position+5])*5;

            if(input_w > 255){
                input_w = 255;
            }
            if(input_Prs > 255){
                input_Prs = 255;
            }
            
            tp_log_debug(" input_x = %d, input_y=%d\n", input_x, input_y);
            if((input_x < 0) || (input_y < 0)){
                continue;
            }

            if((input_x > ts->abs_x_max)||(input_y > ts->abs_y_max)){
                continue;
            }
            
            //input_x = ts->abs_x_max - input_x -1 ;
            //input_y = ts->abs_y_max -input_y -1;

            input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
            input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);
            input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, input_w);
            input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, input_w);
            input_report_abs(ts->input_dev, ABS_MT_PRESSURE, input_Prs);
            input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, input_id);
            input_report_key(ts->input_dev, BTN_TOUCH, 1);
            input_mt_sync(ts->input_dev);

            finger_cnt++;
        }
      }    

    tp_log_debug("finger_cnt=%d\n",finger_cnt); 

    if(finger_cnt == 0)
    {    
        input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
        input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);            
        input_report_key(ts->input_dev, BTN_TOUCH, 0);        
        input_mt_sync(ts->input_dev);
    }     
}
#endif

void nvt_ts_wakeup_gesture_report(unsigned char gesture_id)
{
    struct i2c_client *client = ts->client;
    unsigned int keycode=0;

    tp_log_info("%s line%d: gesture_id = %d\n", __func__,__LINE__,gesture_id);

    switch(gesture_id)
    {
        case GESTURE_ID_DOUBLE_CLICK:
            dev_info(&client->dev, "Gesture : Double Click.\n");
            keycode = ts->easy_wakeup_gesture_keys[0];
            break;    
        case GESTURE_ID_WORD_C:
            dev_info(&client->dev, "Gesture : Word-C.\n");
            keycode = ts->easy_wakeup_gesture_keys[7];
            break;
        case GESTURE_ID_WORD_e:
            dev_info(&client->dev, "Gesture : Word-e.\n");
            keycode = ts->easy_wakeup_gesture_keys[8];
            break;
        case GESTURE_ID_WORD_M:
            dev_info(&client->dev, "Gesture : Word-M.\n");
            keycode = ts->easy_wakeup_gesture_keys[9];
            break;
        case GESTURE_ID_WORD_W:
            dev_info(&client->dev, "Gesture : Word-W.\n");
            keycode = ts->easy_wakeup_gesture_keys[10];
            break;
        default:
            break;
    }

    if(keycode > 0)
    {
        input_report_key(ts->input_dev, keycode, 1);
        input_sync(ts->input_dev);
        input_report_key(ts->input_dev, keycode, 0);
        input_sync(ts->input_dev);
    }

    msleep(250);
}


void set_prot_parms(void * data)
{
    int retry = 0;
    struct nvt_ts_data * ts = (struct nvt_ts_data *)data;
    //set input device info
    ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
    ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
    //ts->input_dev->absbit[0] = BIT(ABS_X) | BIT(ABS_Y) | BIT(ABS_PRESSURE); 
    ts->input_dev->propbit[0] = BIT(INPUT_PROP_DIRECT);

#if MT_PROTOCOL_B
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 7, 0))
    input_mt_init_slots(ts->input_dev, ts->max_touch_num,0);
#else
    input_mt_init_slots(ts->input_dev, ts->max_touch_num);
#endif
//    input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, 65535, 0, 0);
#else
    input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, ts->max_touch_num, 0, 0);
#endif

    //input_set_abs_params(ts->input_dev, ABS_MT_TOOL_TYPE, 0, MT_TOOL_FINGER, 0, 0); 
    input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);             //pressure = 255
    //input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);      //area = 255

    if(ts->max_touch_num > 1){
        input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->abs_x_max, 0, 0);
        input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->abs_y_max, 0, 0);
    }else{
        input_set_abs_params(ts->input_dev, ABS_X, 0, ts->abs_x_max, 0, 0);
        input_set_abs_params(ts->input_dev, ABS_Y, 0, ts->abs_y_max, 0, 0);
    }

    if(ts->max_key_num > 0){
        for(retry = 0; retry < ts->max_key_num; retry++){
            input_set_capability(ts->input_dev, EV_KEY,ts->touch_key_array[retry]);    
        }
    }

    if(ts->easy_wakeup_supported_gestures){
        for(retry = 0; retry < 12; retry++)
        {
            input_set_capability(ts->input_dev, EV_KEY, ts->easy_wakeup_gesture_keys[retry]);
        }
    }

}


int nvt_input_init(struct nvt_ts_data * ts)
{
    int ret = 0;

    //allocate input device
    ts->input_dev = input_allocate_device();
    if(ts->input_dev == NULL){
        tp_log_err("%s line%d: allocate input device failed.\n", __func__, __LINE__);
        return -ENOMEM;
    }
    //set input_dev parms
    set_prot_parms(ts);
    ts->input_dev->name = NVT_TS_NAME;
    ts->input_dev->id.bustype = BUS_I2C;
    ts->input_dev->id.vendor = 0x0205;
    ts->input_dev->id.product = 0x0001;
    //register input device
    ret = input_register_device(ts->input_dev);
    if(ret){
        tp_log_err("%s line%d: register input device (%s) failed. ret=%d\n",
            __func__, __LINE__,ts->input_dev->name, ret);
        input_free_device(ts->input_dev);
        ts->input_dev = NULL;
        return ret;
    }
    return 0;
}

void nvt_input_report(struct nvt_ts_data * ts, uint8_t * point_data)
{
    int i = 0;
    int tmp_gesture_id = 0;

    if(NULL == point_data || NULL == ts){
        tp_log_err("%s line%d: invalid parms, point_data = %p, ts = %p\n",
            __func__,__LINE__,point_data, ts);
        return;
    }
    
    tp_log_debug(" point_data: \n");
    for(i=0; i<63; i++){
            tp_log_debug("%02x, ",point_data[i]);
    }
    tp_log_debug("\n point_data_end.\n");

    tmp_gesture_id= point_data[1]>>3;

    if(IS_GESTURE_SUPPORTED(ts->easy_wakeup_supported_gestures)
        && IS_GESTURE_ENBALED(ts->easy_wakeup_gestures)
        && tmp_gesture_id >= 12 
        && tmp_gesture_id <= 24){
        ts->gesture_id = tmp_gesture_id;
        nvt_ts_wakeup_gesture_report(ts->gesture_id);
        for(i=0;i<6;i++)
        {          
            ts->gesture_positions[i*2] = (unsigned int)(point_data[6*i+2]<<4) + (unsigned int) (point_data[6*i+4]>>4);
            ts->gesture_positions[i*2+1] = (unsigned int)(point_data[6*i+3]<<4) + (unsigned int) (point_data[6*i+4]&0x0f);
            tp_log_info("%s line%d: gesture-pos-%d: x=%d, y=%d\n",
                __func__,__LINE__,i,ts->gesture_positions[i*2],ts->gesture_positions[i*2+1]);
        }
        return;
    }

    tp_log_debug("%s line%d: do normal report!\n",__func__,__LINE__);

#if MT_PROTOCOL_B
    mt_protocol_b(ts, point_data);
#else
    mt_protocol_a(ts, point_data);
#endif

    if(ts->max_key_num > 0){
        if(point_data[61]==0xF8){
            for(i=0; i<ts->max_key_num; i++){
                input_report_key(ts->input_dev, ts->touch_key_array[i], ((point_data[62]>>i)&(0x01)));    
            }
        }else{
            for(i=0; i<ts->max_key_num; i++){
                input_report_key(ts->input_dev, ts->touch_key_array[i], 0);    
            }
        }    
    }

    input_sync(ts->input_dev);

    return;
}


//----------for easy_wakeup_gesture supportting-----------------------------------
static ssize_t nvt_easy_wakeup_gesture_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    return snprintf(buf,PAGE_SIZE,"0x%04X\n",ts->easy_wakeup_gestures);
}

static ssize_t nvt_easy_wakeup_gesture_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t size)
{
    unsigned long value;
    int ret = 0;
    
    if(!ts->easy_wakeup_supported_gestures){
        tp_log_info("%s line%d: easy wakeup not supported!\n",__func__,__LINE__);
        return -EINVAL;
    }else if(NVT_STATE_SUSPEND == ts->suspend_state){
        tp_log_info("%s line%d: can't change wakeup gesture when suspended!\n",__func__,__LINE__);
        return -EINVAL;
    }else{
        ret = kstrtoul(buf, 10, &value);
        if(ret < 0){
            tp_log_err("%s line%d: parse input failed!\n",__func__, __LINE__);
            return -EINVAL;
        }
        if(value > 0xFFFF){
            tp_log_err("%s %d:Invalid value: %lu !\n", __func__, __LINE__, value);
            return -EINVAL;
        }
        
        ts->easy_wakeup_gestures = (uint16_t)value;
        ts->easy_wakeup_gestures_nova = toNovaGestureBits(ts->easy_wakeup_gestures);
    }

    return size;
}

static ssize_t nvt_easy_wakeup_position_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int i = 0;
    int size = 0;

    if(ts->gesture_id == GESTURE_ID_DOUBLE_CLICK){
        for (i = 0; i < 2; i++){
            size += snprintf(buf + size, PAGE_SIZE - size, 
                    "%04x%04x",ts->gesture_positions[i*2],ts->gesture_positions[i*2+1]);
            tp_log_info("%s line%d: %d, %d\n",__func__,__LINE__,
                ts->gesture_positions[i*2],ts->gesture_positions[i*2+1]);
        }
        size += snprintf(buf + size, PAGE_SIZE - size, "\n");
    }else{
        for (i = 0; i < 6; i++){
            size += snprintf(buf + size, PAGE_SIZE - size, 
                    "%04x%04x",ts->gesture_positions[i*2],ts->gesture_positions[i*2+1]);
            tp_log_info("%s line%d: %d, %d\n",__func__,__LINE__,
                ts->gesture_positions[i*2],ts->gesture_positions[i*2+1]);
        }
        size += snprintf(buf + size, PAGE_SIZE - size, "\n");
    }
    
    return size;
}

static DEVICE_ATTR(easy_wakeup_gesture,   S_IRUSR | S_IWUSR,  
    nvt_easy_wakeup_gesture_show,nvt_easy_wakeup_gesture_store);
static DEVICE_ATTR(easy_wakeup_position,S_IRUGO,nvt_easy_wakeup_position_show,NULL);

static struct attribute *nvt_wakeup_attributes[] = {
    &dev_attr_easy_wakeup_gesture.attr,
    &dev_attr_easy_wakeup_position.attr,  
    NULL
};

static struct attribute_group nvt_wakeup_attribute_group = {
    .attrs = nvt_wakeup_attributes
};

void nvt_sysfs_easy_wakeup_init(void)
{
    int ret=0;
    struct kobject *kobject_ts = NULL;

    kobject_ts = tp_get_touch_screen_obj();
    if (!kobject_ts) {
        tp_log_err("%s line%d: create kobjetct error!\n",__func__,__LINE__);
        return ;
    }

    ret = sysfs_create_group(kobject_ts, &nvt_wakeup_attribute_group);
    if (ret){
       tp_log_err("%s line%d: ret=%d, sysfs_create_group failed!\n", __func__,__LINE__, ret);
    }else{
       tp_log_info("%s line%d:  sysfs_create_group succeed.\n", __func__,__LINE__);
    }
}


