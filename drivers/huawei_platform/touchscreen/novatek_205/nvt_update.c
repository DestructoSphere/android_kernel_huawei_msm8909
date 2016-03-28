
/*******************
  Auto Update FW in Probe
********************/

#include "nvt.h"
#define FW_DATA_LENGTH    32768


const struct firmware *fw_entry = NULL;
struct delayed_work nvt_fw_updatew_wq;

extern void nvt_hw_reset(void);
extern int nvt_i2c_read(struct i2c_client *client, uint8_t address, uint8_t *buf, uint8_t len);
extern void nvt_i2c_write (struct i2c_client *client, uint8_t address, uint8_t *data, uint8_t len);

extern void nvt_start_esd_timer(void);
extern void nvt_stop_esd_timer(void);

uint16_t get_fw_version(void)
{
    uint8_t fw_version = 0;
    uint8_t I2C_Buf[4]={0};
    I2C_Buf[0]=0xFF;
    I2C_Buf[1]=0x3F;
    I2C_Buf[2]=0x00;
    nvt_i2c_write(ts->client, I2C_FW_Address, I2C_Buf, 3);
    msleep(10);

    I2C_Buf[0] = 0x78;
    nvt_i2c_read(ts->client, I2C_FW_Address, I2C_Buf, 2);
    fw_version = I2C_Buf[1];

    tp_log_info("%s: fw_version is : %d\n", __func__,fw_version);
    
    return (uint16_t)fw_version;

}

uint16_t get_vendor_id(void)
{
    uint8_t vendor = 0;
    uint8_t I2C_Buf[4]={0};
    I2C_Buf[0]=0xFF;
    I2C_Buf[1]=0x3F;
    I2C_Buf[2]=0x00;
    nvt_i2c_write(ts->client, I2C_FW_Address, I2C_Buf, 3);
    msleep(10);

    I2C_Buf[0] = 0xD1;
    nvt_i2c_read(ts->client, I2C_FW_Address, I2C_Buf, 2);
    vendor = I2C_Buf[1];

    tp_log_info("%s: vendor is : %d\n", __func__,vendor);
    if(vendor != 12){
        vendor = 12;
        tp_log_info("%s line%d: change vendor to 12 enforce!!\n",__func__,__LINE__);
    }
    
    return (uint16_t)vendor;

}

int check_fm_ver(u8 *fw_data)
{
    uint8_t I2C_Buf[16];
    memset(I2C_Buf,0,sizeof(uint8_t)*16);

    I2C_Buf[0]=0xFF;
    I2C_Buf[1]=0x3F;
    I2C_Buf[2]=0x00;
    nvt_i2c_write(ts->client, I2C_FW_Address, I2C_Buf, 3);
    msleep(10);

    I2C_Buf[0] = 0x78;
    nvt_i2c_read(ts->client, 0x01, I2C_Buf, 2);
    tp_log_info("IC FW Ver = %d\n", I2C_Buf[1]);
    tp_log_info("Bin FW Ver = %d\n", *(fw_data + 0x7F00));
    
    if(I2C_Buf[1] >= *(fw_data + 0x7F00)){
        return 1;
    }else{
        return 0;
    }
}

int check_checksum(u8 *fw_data)
{
    uint8_t I2C_Buf[64];
    uint8_t buf2[64];
    int i, j, k, Retry_Counter=0;
    int addr=0;
    uint8_t addrH, addrL;
    unsigned short RD_Filechksum, WR_Filechksum;

    WR_Filechksum = 0;

    memset(I2C_Buf,0,sizeof(uint8_t)*64);
    memset(buf2,0,sizeof(uint8_t)*64);
    nvt_hw_reset();
    msleep(500);
    I2C_Buf[0]=0xFF;
    I2C_Buf[1]=0x3F;
    I2C_Buf[2]=0xE8;
    nvt_i2c_write(ts->client, I2C_FW_Address, I2C_Buf, 3);

    I2C_Buf[0]=0x00;
    I2C_Buf[1]=0xEA;
    nvt_i2c_write(ts->client, I2C_FW_Address, I2C_Buf, 2);

    for(i=0;i<FW_DATA_LENGTH/128;i++){
        for(j=0;j<16;j++){
            unsigned char tmp=0;
            addrH = addr>>8;
            addrL = addr&0xFF;
            for(k=0;k<8;k++){
                tmp+= *(fw_data + (i*128+j*8+k));
            }
            tmp = tmp+addrH+addrL+8;
            tmp = (255-tmp)+1;
            WR_Filechksum+=tmp;
            addr+=8;
        }
    }
    msleep(800);
    do{
        msleep(10);
        I2C_Buf[0]=0xFF;
        I2C_Buf[1]=0x3F;
        I2C_Buf[2]=0xF8;
        nvt_i2c_write(ts->client, I2C_FW_Address, I2C_Buf, 3);

        buf2[0]=0x00;
        buf2[1]=0x00;
        buf2[2]=0x00;
        buf2[3]=0x00;
        nvt_i2c_read(ts->client, I2C_FW_Address, buf2, 4);

        Retry_Counter++;
        msleep(10);

    }while((Retry_Counter<20)&& (buf2[1]!=0xAA));

    if(buf2[1]==0xAA){
        RD_Filechksum=(buf2[2]<<8)+buf2[3];
        if(RD_Filechksum==WR_Filechksum){
            return 1;    // checksum match
        }else{
            return 0;    // checksum not match
        }
    }else{
        return -1;    // read checksum failed
    }
}


uint8_t init_update(void)
{
    uint8_t I2C_Buf[16];
    memset(I2C_Buf,0,sizeof(uint8_t)*16);

    // initial BootLoader
    I2C_Buf[0] = 0x00;
    I2C_Buf[1] = 0xA5;
    nvt_i2c_write(ts->client, I2C_HW_Address, I2C_Buf, 2);
    msleep(2);
  
    // Initiate Flash Block
    I2C_Buf[0] = 0x00;
    I2C_Buf[1] = 0x00;
    nvt_i2c_write(ts->client, I2C_HW_Address, I2C_Buf, 2);
    msleep(20);

    // Read status
    I2C_Buf[0] = 0x00;
    nvt_i2c_read(ts->client, I2C_HW_Address, I2C_Buf, 2);
    return I2C_Buf[1];
}

uint8_t erase_flash(void)
{
    int i = 0;
    unsigned int Row_Address = 0;
    uint8_t I2C_Buf[16];    
    int retry = 0;

    memset(I2C_Buf,0,sizeof(uint8_t)*16);    
    
    I2C_Buf[0]=0x00;
    I2C_Buf[1]=0x66;
    I2C_Buf[2]=0x00;
    I2C_Buf[3]=0x0E;
    I2C_Buf[4]=0x01;
    I2C_Buf[5]=0xB4;
    I2C_Buf[6]=0x3D;
    nvt_i2c_write(ts->client, I2C_HW_Address, I2C_Buf, 7);

    for(retry = 0; retry < 100; retry++){
        msleep(1);
        nvt_i2c_read(ts->client, I2C_HW_Address, I2C_Buf, 2);
        if(I2C_Buf[1]==0xAA)  break;
    }
    if(retry == 100){
        tp_log_err("%s line%d: Program: unprotect(1) get status(0x%2X) error.", 
            __func__,__LINE__,I2C_Buf[1]);
        return I2C_Buf[1]; 
    }
    tp_log_info("%s line%d: Program: unprotect(1) succeed in %d times.\n",
        __func__,__LINE__, retry);


    I2C_Buf[0]=0x00;
    I2C_Buf[1]=0x66;
    I2C_Buf[2]=0x00;
    I2C_Buf[3]=0x0F;
    I2C_Buf[4]=0x01;
    I2C_Buf[5]=0xEF;
    I2C_Buf[6]=0x01;
    nvt_i2c_write(ts->client, I2C_HW_Address, I2C_Buf, 7);

    for(retry = 0; retry < 100; retry++){
        msleep(1);
        nvt_i2c_read(ts->client, I2C_HW_Address, I2C_Buf, 2);
        if(I2C_Buf[1]==0xAA)  break;
    }
    if(retry == 100){
        tp_log_err("%s line%d: Program: unprotect(2) get status(0x%2X) error.", 
            __func__,__LINE__,I2C_Buf[1]);
        return I2C_Buf[1]; 
    }
    tp_log_info("%s line%d: Program: unprotect(2) succeed in %d times.\n",
        __func__,__LINE__, retry);

    for (i = 0 ; i < FW_DATA_LENGTH/4096 ; i++){        // 32K = 8 times
        Row_Address = i * 4096;                                                             
        // Erase Flash    
        I2C_Buf [0] = 0x00;
        I2C_Buf [1] = 0x33;
        I2C_Buf [2] = (uint8_t)((Row_Address & 0xFF00) >> 8 );    // Address High Byte  
        I2C_Buf [3] = (uint8_t)(Row_Address & 0x00FF);    // Address Low Byte                     
        nvt_i2c_write(ts->client, I2C_HW_Address, I2C_Buf, 4);
        msleep(15);    // Delay 15 ms 
              
        // Read Erase status
        nvt_i2c_read(ts->client, I2C_HW_Address, I2C_Buf, 2); 

        // check status        
        if (I2C_Buf[1] != 0xAA){
            tp_log_err("%s line%d: Erase failed at %d times.\n",__func__,__LINE__,i);
            return I2C_Buf[1];
        }            
    }
    return I2C_Buf[1];
}

uint8_t write_ic(u8 *fw_data)
{
    int i,j;
    unsigned int Flash_Address = 0;
    uint8_t I2C_Buf[16];    
    uint8_t CheckSum[16];    // 128/8 = 16 times ;
    int retry = 0;
    
    memset(I2C_Buf,0,sizeof(uint8_t)*16);
    memset(CheckSum,0,sizeof(uint8_t)*16);    

    for(j=0;j<FW_DATA_LENGTH/128;j++){
            Flash_Address=(j)*128;
           for (i = 0 ; i < 16 ; i++, Flash_Address += 8){    // 128/8 = 16 times for One Row program
           
            // write bin data to IC
            I2C_Buf[0] = 0x00;
            I2C_Buf[1] = 0x55;    //Flash write command
            I2C_Buf[2] = (uint8_t)(Flash_Address  >> 8);    //Flash address [15:8]
            I2C_Buf[3] = (uint8_t)(Flash_Address & 0xFF);    //Flash address [7:0]
            I2C_Buf[4] = 0x08;    //Flash write length (byte)
            I2C_Buf[6] = *(fw_data + Flash_Address + 0);    //Binary data 1
            I2C_Buf[7] = *(fw_data + Flash_Address + 1);    //Binary data 2
            I2C_Buf[8] = *(fw_data + Flash_Address + 2);    //Binary data 3
            I2C_Buf[9] = *(fw_data + Flash_Address + 3);    //Binary data 4
            I2C_Buf[10] = *(fw_data + Flash_Address + 4);   //Binary data 5
            I2C_Buf[11] = *(fw_data + Flash_Address + 5);    //Binary data 6
            I2C_Buf[12] = *(fw_data + Flash_Address + 6);    //Binary data 7
            I2C_Buf[13] = *(fw_data + Flash_Address + 7);    //Binary data 8

            CheckSum[i] = ~(I2C_Buf[2] + I2C_Buf[3] + I2C_Buf[4] + I2C_Buf[6] + I2C_Buf[7] +
                          I2C_Buf[8] + I2C_Buf[9] + I2C_Buf[10] + I2C_Buf[11] + I2C_Buf[12] +
                          I2C_Buf[13]) + 1;

            // Load check sum to I2C Buffer
            I2C_Buf[5] = CheckSum[i];
            nvt_i2c_write(ts->client, I2C_HW_Address, I2C_Buf, 14);
           }
        msleep(10);
        
        // Read status
        I2C_Buf[0] = 0x00;
        for(retry = 0; retry < 10; retry++){
            nvt_i2c_read(ts->client, I2C_HW_Address, I2C_Buf, 2);
            if(I2C_Buf[1]==0xAA){
                break;
            }
            msleep(1);
        }
        if(retry == 10){
            tp_log_err("%s line%d: Write failed at %d times.\n",__func__,__LINE__,j);
            return I2C_Buf[1];
        }
    }
    return I2C_Buf[1];
}

int  update_firmware(u8 *fw_data)
{
    
    int ret;
    uint8_t status = 0;

    //step1:init
    status = init_update();
    if (status != 0xAA){
        tp_log_err("%s line%d: init get status(0x%2X) error.\n",__func__,__LINE__,status);
        return status;
    }
    tp_log_info("%s line%d: init get status(0x%2X) success.\n",__func__,__LINE__,status);

    //step2:erase
    status = erase_flash();    
    if (status != 0xAA){
        tp_log_err("%s line%d: erase(0x%2X) error.\n",__func__,__LINE__, status);
            return status;
    }            
    tp_log_info("%s line%d: erase(0x%2X) success.\n", __func__,__LINE__,status);

    //step3:write
    tp_log_info("%s line%d: write begin, please wait...\n",__func__,__LINE__);
    status = write_ic(fw_data);   
    if (status != 0xAA){
        tp_log_err("%s line%d: write_ic(0x%2X) error.\n", __func__,__LINE__,status);
            return status;
    }            
    tp_log_info("%s line%d: write_ic(0x%2X) success.\n",__func__,__LINE__, status);

    // step4:verify  
    tp_log_info("%s line%d: Verify begin, please wait....\n",__func__,__LINE__);
    ret=check_checksum(fw_data);
    if(ret==1){
        tp_log_info("%s line%d: Verify PASS\n",__func__,__LINE__);
    }else if(ret==0){
        tp_log_info("%s line%d: Verify FAIL\n",__func__,__LINE__);
    }else if(ret==-1){
        tp_log_info("%s line%d: Verify FAIL (FW not return)\n",__func__,__LINE__);
    }
    tp_log_info("Program: END\n,__func__,__LINE__");
    return ret;
}

int  update_firmware_request(char * filename)
{
    int ret = 0;

    if(NULL == filename){
        tp_log_err("firmware_name is NULL\n");
        return -1;
    }

    tp_log_info("firmware_name = %s\n", filename);

    ret = request_firmware(&fw_entry, filename, &ts->client->dev);    
    if (ret) {
        tp_log_err("Firmware %s not available, code = %d\n",
            filename, ret);
        return  ret;
    }

    tp_log_info("Got firmware, size: %d.\n", fw_entry->size);
    tp_log_info("fw_version: %d.\n", *(fw_entry->data + 0x7F00));
    
    return ret;
}

void update_firmware_release(void)
{
    if (fw_entry){
        release_firmware(fw_entry);
    }
    fw_entry = NULL;
}

void ckeck_and_update_firmware(struct work_struct *nvt_fw_updatew_wq)
{
    int ret = 0;
    char firmware_name[256] = "";

    snprintf(firmware_name, sizeof(firmware_name), "%s_novatek_fw.bin",
        ts->product_name);

    ret = update_firmware_request(firmware_name);
    if(ret < 0){
        tp_log_info("update firmware request failed.\n");
        return ;
    }

    if(!mutex_trylock(&ts->nvt_mutex)){
        tp_log_info("%s line%d: get nvt_mutex failed!\n",__func__,__LINE__);
        update_firmware_release();
        return ;
    }

    nvt_stop_esd_timer();

    nvt_pm_runtime_get(ts);
    
    ret = check_checksum((u8 *)fw_entry->data);    
    if(ret==-1){    // read fw checksum failed    
        tp_log_info("read checksum failed.\n");
        update_firmware((u8 *)fw_entry->data);
    }
    // (fw checksum not match) && (bin fw version > ic fw version)
    else if(ret==0&&(check_fm_ver((u8 *)fw_entry->data)==0))    {
        tp_log_info( "firmware version not match.\n");
        update_firmware((u8 *)fw_entry->data);
    }
    nvt_hw_reset();

    nvt_pm_runtime_put(ts);
    nvt_start_esd_timer();

    mutex_unlock(&ts->nvt_mutex);
    
    update_firmware_release();
}

void auto_update(void)
{
    INIT_DELAYED_WORK(&nvt_fw_updatew_wq, ckeck_and_update_firmware);
    schedule_delayed_work(&nvt_fw_updatew_wq, msecs_to_jiffies(2000));
}

static ssize_t nvt_sysfs_manual_fwup_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int ret = 0;

    ret = update_firmware_request(MANUAL_FIRMWARE_NAME);
    if(ret < 0){
        tp_log_info("%s line%d: update firmware request failed!\n",__func__,__LINE__);
        return snprintf(buf,PAGE_SIZE,"update firmware request failed!\n");
    }

    if(!mutex_trylock(&ts->nvt_mutex)){
        update_firmware_release();
        tp_log_info("%s line%d: get nvt_mutex failed!\n",__func__,__LINE__);
        return snprintf(buf,PAGE_SIZE,"get nvt_mutex failed!\n");
    }

    nvt_stop_esd_timer();

    nvt_pm_runtime_get(ts);

    ret = update_firmware((u8 *)fw_entry->data);
    nvt_hw_reset();

    nvt_pm_runtime_put(ts);

    nvt_start_esd_timer();

    mutex_unlock(&ts->nvt_mutex);
    
    update_firmware_release();   

    if(ret){
        tp_log_info("%s line%d: manual firmware update succeed!\n",__func__,__LINE__);   
        return snprintf(buf,PAGE_SIZE,"manual firmware update succeed!\n");
    }else{
        tp_log_info("%s line%d: manual firmware update failed!\n",__func__,__LINE__);   
        return snprintf(buf,PAGE_SIZE,"manual firmware update failed!\n");
    }
}

static ssize_t nvt_sysfs_fw_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{    
    uint8_t version = 0;
    uint8_t vendor = 0;

    uint8_t I2C_Buf[4];
    memset(I2C_Buf,0,sizeof(uint8_t)*4);

    I2C_Buf[0]=0xFF;
    I2C_Buf[1]=0x3F;
    I2C_Buf[2]=0x00;
    nvt_i2c_write(ts->client, I2C_FW_Address, I2C_Buf, 3);
    msleep(10);

    I2C_Buf[0] = 0x78;
    nvt_i2c_read(ts->client, 0x01, I2C_Buf, 2);
    version = I2C_Buf[1];

    I2C_Buf[0] = 0xD1;
    nvt_i2c_read(ts->client, 0x01, I2C_Buf, 2);
    vendor = I2C_Buf[1];
    
    return snprintf(buf, PAGE_SIZE, "0x%02X.%02X\n", vendor,version);
}


static DEVICE_ATTR(manual_fwup, S_IRUGO,  nvt_sysfs_manual_fwup_show,NULL);
static DEVICE_ATTR(fw_version, S_IRUGO,  nvt_sysfs_fw_version_show,NULL);


static struct attribute *nvt_fwup_attributes[] = {
    &dev_attr_manual_fwup.attr,
    &dev_attr_fw_version.attr,
    NULL
};

static struct attribute_group nvt_fwup_attributes_group = {
    .attrs = nvt_fwup_attributes
};

void nvt_sysfs_fwup_init(void)
{
    int ret=0;
    struct kobject *kobject_ts = NULL;
    
    kobject_ts = tp_get_touch_screen_obj();
    if (!kobject_ts) {
        tp_log_err("get kobjetct error!\n");
        return ;
    }

    ret = sysfs_create_group(kobject_ts, &nvt_fwup_attributes_group);
    if (ret){
        tp_log_info("%s line%d: ret = %d, sysfs_create_group failed!\n", __func__,__LINE__,ret);
    }else{
        tp_log_info("%s line%d: sysfs_create_group succeeded.\n", __func__,__LINE__);
    }

    auto_update();
    tp_log_info("%s line%d: auto-update proc launched.\n", __func__, __LINE__);
}


