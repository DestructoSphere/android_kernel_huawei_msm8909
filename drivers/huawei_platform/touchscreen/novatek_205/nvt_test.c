
/*******************
  Nvt Touchpanel Test
********************/
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include "nvt.h"
#include "nvt_test.h"

extern void nvt_start_esd_timer(void);
extern void nvt_stop_esd_timer(void);

#define RAW_DATA_SIZE 1024
#define FW_DATA_LENGTH    32768
#define FW_BUF  40960
//#define MANUAL_UPDATE_FIRMWARE_ENABLE

unsigned int xnum;
unsigned int ynum;
int iaccum;
int fcf;
int fcc;

unsigned char gridcolor_flag[40][40];
unsigned char record_result[40*40];    
int mutual_goldenratio[40][40];
int mutual_data[40*40]; //raw_data
int cm_data[40*40]; 

static int DiffTemp[40*40];
char fail[2048] = {0};
void init_sample_test_parms(struct nvt_ts_data * ts){
    if(strncmp(ts->product_name, PHONE_NAME_SCALE, sizeof(PHONE_NAME_SCALE)) == 0)
    {
        // 12 -> cmi  for temp using
        if(12 == ts->vendor_id){
            IPOSTIVE_TOLERANCE = IPOSTIVE_TOLERANCE_SCALE_CMI;
            ITOLERANCE_S = ITOLERANCE_S_SCALE_CMI;
            AIN = AIN_SCALE_CMI;
            FPC_CM = FPC_CM_SCALE_CMI;
            Mutual_AVG = Mutual_AVG_SCALE_CMI;
            tp_log_info("%s: select sample test parms for %s-cmi.\n"
                                "IPOSTIVE_TOLERANCE = %d\n"
                                "ITOLERANCE_S = %d\n"                           
                                "AIN = %p\n"
                                "FPC_CM = %p\n"
                                "Mutual_AVG = %p\n",
                                __func__,ts->product_name,
                                IPOSTIVE_TOLERANCE,ITOLERANCE_S,
                                AIN, FPC_CM, Mutual_AVG);
        }else{
        //default config
            IPOSTIVE_TOLERANCE = IPOSTIVE_TOLERANCE_SCALE_CMI;
            ITOLERANCE_S = ITOLERANCE_S_SCALE_CMI;
            AIN = AIN_SCALE_CMI;
            FPC_CM = FPC_CM_SCALE_CMI;
            Mutual_AVG = Mutual_AVG_SCALE_CMI;
            tp_log_info("%s: select default parms for %s-default.\n"
                                "IPOSTIVE_TOLERANCE = %d\n"
                                "ITOLERANCE_S = %d\n"                          
                                "AIN = %p\n"
                                "FPC_CM = %p\n"
                                "Mutual_AVG = %p\n",
                                __func__,ts->product_name,
                                IPOSTIVE_TOLERANCE,ITOLERANCE_S,
                                AIN, FPC_CM, Mutual_AVG);        
        }
    }
}

uint8_t nvt_read_touch_info(void)
{
    uint8_t buffer[16]={0};
    buffer[0]=0x78;
    nvt_i2c_read(ts->client, I2C_FW_Address, buffer, 5);
    msleep(50);
    xnum = buffer[3];
    ynum = buffer[4];
    tp_log_info("%s: xnum=%d, ynum=%d\n", __func__,xnum, ynum);
    xnum = xnum >= 40 ? 39 : xnum;
    ynum = ynum >= 40 ? 39 : ynum;
    return buffer[1];
}

static ssize_t nvt_sysfs_reset_touch_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    nvt_hw_reset();
    return snprintf(buf, PAGE_SIZE, "reset touch ic!\n");
}


int gpioshort_allcheck(uint8_t *buffer)
{
    unsigned short timeoutcnt1;
    timeoutcnt1 = 0;
short_allcheck:

    buffer[0]=0xFF;
    buffer[1]=0x3F;
    buffer[2]=0xE8;
    nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 3);    
    msleep(1);
    
    buffer[0]=0x00;
    buffer[1]=0xC5;
    nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 2);    
    msleep(100);

    while(1){
        buffer[0]=0xFF;
        buffer[1]=0x3F;
        buffer[2]=0xE9;
        nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 3);    
        msleep(1);

        buffer[0]=0x00;
        buffer[1]=0x00;        
        nvt_i2c_read(ts->client, I2C_FW_Address, buffer, 8);
        if(buffer[1]==0xBB){
            break;
        }
        msleep(1);
        
        timeoutcnt1++;
        if(timeoutcnt1 > 3){
            return 1;
        }else{
            goto short_allcheck;
        }        
    }    
    return 0;
}
char* gpio2ain2txrx(char *fail,unsigned char gpio)
{
    int record_ain=0;
    if(AIN[gpio] == -1){
        if(!record_ain){
            sprintf(fail, "UnusedPin, ");
        }else{
            sprintf(fail,"UnusedPin%d[AIN%d], ", (AIN[gpio]-32), (gpio));
        }
    }else if(AIN[gpio]>=32){
        if(!record_ain){
            sprintf(fail,"rx%d, ", (AIN[gpio]-32));
        }else{
            sprintf(fail,"rx%d[AIN%d], ", (AIN[gpio]-32), (gpio));
        }        
    }else{
        if(!record_ain){
            sprintf(fail,"tx%d, ", (AIN[gpio]));
        }else{
            sprintf(fail,"tx%d[AIN%d], ", (AIN[gpio]), (gpio));
        }
    }    
    return fail;
}

int enter_test_mode(void)
{
    int timeoutcnt0=0, timeoutcnt1=0;
    uint8_t buffer[16]={0};
lenter_test_mode:
    buffer[0]=0xFF;
    buffer[1]=0x3F;
    buffer[2]=0xE8;
    nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 3);
    msleep(1);

    buffer[0]=0x00;
    buffer[1]=0xCC;
    nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 2);
    msleep(500);
    
    while(1)
    {
        buffer[0]=0xFF;
        buffer[1]=0x3F;
        buffer[2]=0xE9;
        nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 3);
        msleep(1);

        buffer[0]=0x00;
        nvt_i2c_read(ts->client, I2C_FW_Address, buffer, 2);
        if(buffer[1]==0xBB){
            break;
        }else {
            msleep(1);
            timeoutcnt0++;
            if(timeoutcnt0 > 100){
                timeoutcnt1++;
                timeoutcnt0=0;
                if(timeoutcnt1 > 3){
                    return 1;
                }else{
                    goto lenter_test_mode;
                }
            }
        }
    }
    return 0;
}

void leave_test_mode(void)
{    
    uint8_t buffer[16]={0};
    buffer[0]=0xFF;
    buffer[1]=0x3F;
    buffer[2]=0xE9;
    nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 3);
    msleep(1);

    buffer[0]=0x00;
    buffer[1]=0xAA;
    nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 2);
    msleep(1);
}
    
void get_tp_cm_parameter(void)
{
    uint8_t buffer[32]={0};
    const int cf_table[8] = { 222,  327,  428,  533,  632,  737, 838, 943};
    const int cc_table[32] = { 30,  57,  82,  109,  136,  163, 188, 215,
                                    237,  264,  289,  316,  343,  370 , 395, 422,
                                    856,  883,  908,  935,  962,  989,1014,1041,
                                   1063, 109 , 1115, 1142, 1169, 1196,1221,1248};
    //Reset IC
    nvt_hw_reset();

    buffer[0]=0xFF;
    buffer[1]=0x3F;
    buffer[2]=0xB7;
    nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 3);
    msleep(1);

    buffer[0]=0x00;
    nvt_i2c_read(ts->client, I2C_FW_Address, buffer, 25);
               
    iaccum =  buffer[1];
    fcf = cf_table[buffer[9]& 0x07];
    fcc = cc_table[((buffer[9]& 0xF8)>>4) |(((buffer[9]& 0xF8)& 0x08) <<1)];

    leave_test_mode();
}

int check_fw_status(void)
{
    uint8_t buffer[16]={0};
    int i;
    for(i=0;i<100;i++){
        msleep(1);
        buffer[0]=0xFF;
        buffer[1]=0x3D;
        buffer[2]=0xFB;
        nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 3);
        msleep(1);

        buffer[0]=0x00;
        nvt_i2c_read(ts->client, I2C_FW_Address, buffer, 2);

        if(buffer[1]==0xAA){
            break;
        }
        msleep(10);
    }

    if(i==100){
        return -1;
    }else{
        return 0;
    }
}

void readraw_nt11205(void)
{
    unsigned int startAddr;
    int i, j, k, m;
    int bytecount, sec_ct, Residual_1, sec_ct2, Residual_2, temp_cnt, offsetAddr;
    uint8_t buffer[64]={0};
    int *diff_temp = NULL;
    diff_temp=(int *)kzalloc(sizeof(int)*5000,GFP_KERNEL);
    temp_cnt=0;
    bytecount=xnum*ynum*2;
    sec_ct=bytecount/244;
    Residual_1=bytecount%244;
    sec_ct2=Residual_1/61;
    Residual_2=Residual_1%61;
    startAddr=0x0800;
    
    for(m=0;m<sec_ct;m++){
        offsetAddr=0;
        buffer[0]=0xFF;
        buffer[1]=startAddr>>8;
        buffer[2]=startAddr&0xFF;
        nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 3);
        msleep(1);

        for(k=0;k<4;k++){
            buffer[0]=offsetAddr&0xFF;
            nvt_i2c_read(ts->client, I2C_FW_Address, buffer, 62);
            offsetAddr+=61;
            for(i=0;i<61;i++){
                diff_temp[temp_cnt++]=(int)buffer[1+i];
            }
        }
        startAddr+=offsetAddr;
    }
    
    if(Residual_1>0){
        offsetAddr=0;
        buffer[0]=0xFF;
        buffer[1]=startAddr>>8;
        buffer[2]=startAddr&0xFF;
        nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 3);
        msleep(1);

        for(k=0;k<sec_ct2;k++){
            buffer[0]=offsetAddr&0xFF;
            nvt_i2c_read(ts->client, I2C_FW_Address, buffer, 62);
            offsetAddr+=61;
            for(i=0;i<61;i++){
                diff_temp[temp_cnt++]=(int)buffer[1+i];
            }
        }
        startAddr+=offsetAddr;
        if(Residual_2>0){
            offsetAddr=0;
            buffer[0]=0xFF;
            buffer[1]=startAddr>>8;
            buffer[2]=startAddr&0xFF;
            nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 3);
            msleep(1);

            buffer[0]=offsetAddr&0xFF;
            nvt_i2c_read(ts->client, I2C_FW_Address, buffer, Residual_2+1);
            for(i=0;i<Residual_2;i++){
                diff_temp[temp_cnt++]=(int)buffer[1+i];
            }
        }
    }

    //we use printk here to make the table clean
    for(j = 0; j<ynum; j++){
        for(i=0; i< xnum; i++){
            mutual_data[j*xnum+i] = (unsigned short)((diff_temp[2*(j*xnum+i)]<<8)|diff_temp[2*(j*xnum+i)+1]);
            printk("%d, ", mutual_data[j*xnum+i]);
        }
        printk("\n");
    }
    printk("\n");  
    
    kfree(diff_temp);
}

void rawdata_to_cm(void)
{
    int i,j;
    int kk=0;
    int RepeatCnt = 0;
    int temp1;    
    uint8_t buffer[16]={0};
    unsigned short *rawdata = NULL;
    rawdata = (unsigned short *)kzalloc(sizeof(unsigned short )*1000,GFP_KERNEL);
    if(NULL == rawdata){
        tp_log_err("%s line%d: rawdata kmalloc failed!\n", __func__, __LINE__);
        return;
    }
againgetdata:

    //Reset IC
    nvt_hw_reset();
    
    //SendCmdToGetrawdata();
    buffer[0]=0xFF;
    buffer[1]=0x3F;
    buffer[2]=0xE8;
    nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 3);
    msleep(1);            
    buffer[0]=0x00;
    buffer[1]=0xCF;
    nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 2);
    msleep(500);    

    buffer[0]=0xFF;
    buffer[1]=0x3D;
    buffer[2]=0xFC;
    nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 3);
    msleep(1);            
    buffer[0]=0x00;
    buffer[1]=0xAA;
    buffer[2]=0x5A;
    nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 3);
    msleep(500);

    //Backup data at last time
    for(j = 0 ; j< ynum; j++){
        for(i = 0; i < xnum; i++){
            rawdata[j*xnum + i] = mutual_data[j*xnum + i];
        }
    }
    
    //Get data 
    if(check_fw_status()==-1){
        kfree(rawdata);
        return;
    }

    //get Rawdata        
    readraw_nt11205();

    kk =0;
    for(j = 0 ; j< ynum; j++) {
        for(i = 0; i < xnum; i++){
            if(abs(rawdata[j*xnum + i] - mutual_data[j*xnum + i] >1000)){
                kk++;
                if(kk > 2){
                    RepeatCnt++;
                    goto againgetdata;
                }
            }
        }
    }
    
    //\C2\u09a8CM
    temp1=(int)(((2*(long int)fcc*10/3)*10000/fcf));
    tp_log_info("iaccum=%d\n", iaccum);
    tp_log_info("temp1=%d\n", temp1);
    tp_log_info("fcc=%d\n", fcc);
    tp_log_info("fcf=%d\n", fcf);        
    tp_log_info("cm_data\n");

    //to make the table clean, we use printk here
    for(j = 0; j<ynum; j++){
        for(i=0; i< xnum; i++){
            cm_data[xnum*j +i]= (int)((((((((mutual_data[xnum*j +i]/((iaccum+1)/2))-1024)*24)+temp1)/2)*3/10)*fcf/100));
            printk("%d, ", cm_data[xnum*j +i]);
        }
        printk("\n");
    }
    printk("\n");

    kfree(rawdata);
}


//static char gpiomsg[2048]={0};

static ssize_t nvt_sysfs_sensor_short_test(char *buf, ssize_t length, bool * result)
{
    unsigned char port;    
    unsigned char fail_ain;
    unsigned char test_pin;
    unsigned char gpio_shortlist[48];
    unsigned char gpio_shortcnt=0;
    uint8_t buffer[16]={0};
    int i,j;
    int ret=0;    
    ssize_t size = 0;
    
    char *gpiomsg = NULL;
    gpiomsg = (char *)kzalloc(sizeof(char)*2048,GFP_KERNEL);

    //memset(gpiomsg, 0, sizeof(gpiomsg));
    memset(gpio_shortlist, 0xff, sizeof(gpio_shortlist)); 
    
    gpio_shortcnt = 0;

    tp_log_info("%s line%d: sensor short test:\n",__func__,__LINE__);

    //Reset IC
    nvt_hw_reset();

    //Get XY channel number
    nvt_read_touch_info();

    //Set Delay
    buffer[0]=0xFF;
    buffer[1]=0x3F;
    buffer[2]=0xF4;
    nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 3);
    msleep(1);

    buffer[0]=0x00;
    buffer[1]=255;
    nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 2);
    msleep(1);

    //All AINs Check in  ShortTest originally 
    if(gpioshort_allcheck(buffer) >0){
        tp_log_err("%s:Sensor Short Test ,FAIL=>FW time out!\n",__func__);
        size = snprintf(buf, PAGE_SIZE, "Sensor Short Test ,FAIL=>FW time out\n");
        kfree(gpiomsg);
        return 1;
    }
    for(i =0 ; i< 6; i++){
        if(buffer[i+ 2] > 0){
            port = buffer[i+2];
            for(j = 0; j < 8; j++){
                test_pin = (0x01  << j);
                if((port & test_pin) >0){
                    fail_ain = (i* 8) + j;
                    if(AIN[fail_ain]==-1){
                        continue;
                    }else {
                        gpio_shortlist[gpio_shortcnt] = fail_ain;                        
                        gpio_shortcnt++;
                    }                
                }            
            }
        }        
    }

    //Get Result 
    ret =0;
    
    //gpiomsg = "{";
    sprintf(gpiomsg, "{");
    for(j = 0 ; j < gpio_shortcnt; j++){
        i = gpio_shortlist[j];
        if(i >= 0xff){
            continue;
        }else if(AIN[i]==-1){
            continue;
        }
        memset(fail, 0, sizeof(fail));
        gpio2ain2txrx(fail, i);
        sprintf(gpiomsg,"%s%s", gpiomsg, fail);
        ret++;
    }

    if(ret <= 1){
        sprintf(gpiomsg,"%sGND,},", gpiomsg);
    }else{
        sprintf(gpiomsg,"%s},", gpiomsg);
    }

    //nvt_hw_reset();    
    if(ret > 0){
        size = snprintf(buf, length, "Short Test FAIL\n");
        size += snprintf(buf+size, length-size,"%s\n",gpiomsg);
        tp_log_err("%s line%d: short test fail, result = >\n%s\n",__func__,__LINE__,gpiomsg);
        *result = false;
        tp_log_err("Sensor Short Test, FAIL\n%s\n", gpiomsg);
    }else{
        size = snprintf(buf, length, "Short Test PASS\n");
        *result = true;
        tp_log_info("Sensor Short Test ,PASS\n");
    }
    
    kfree(gpiomsg);
    return size;
}

static ssize_t nvt_sysfs_sensor_open_test(char *buf, ssize_t length, bool * result)
{
    int i,j;
    int kk=1;
    ssize_t size = 0;
    char tmp_data[20] = { 0 };
    tp_log_info("%s line%d: sensor open test:\n",__func__,__LINE__);
    
    //Reset IC
    nvt_hw_reset();
    //Get XY channel number
    nvt_read_touch_info();

    // Get rawdata and To CM 
    get_tp_cm_parameter();
    rawdata_to_cm();
    for(j = 0; j<ynum; j++){
        for(i=0; i< xnum; i++){
            cm_data[xnum*j +i]-= FPC_CM[xnum*j +i];
        }
    }
    for(j = 0; j < ynum ;  j++){
        for(i = 0 ; i<xnum ; i++){
            if(cm_data[j*xnum + i] == 0){
                 cm_data[j*xnum + i] = 1;
            }else if(cm_data[j*xnum + i] < 0){        
                 cm_data[j*xnum + 1] = 1;
            }
        }
    }

    //  Check abs low boundary
    tp_log_info("%s line%d: begin to check abs low bound\n",__func__, __LINE__);
    for(j =0 ; j < ynum ; j++){
        for(i = 0; i <xnum; i++){
            kk = (int)((Mutual_AVG[j*xnum+i]*(1000-ITOLERANCE_S))/1000);
            tp_log_debug("(%ld * (1000 - %d))/1000 = %d\n", Mutual_AVG[j*xnum+i], ITOLERANCE_S,kk);
            if(cm_data[j*xnum + i]  < kk ){
                record_result[j*xnum + i] |= 1;
            }
            kk = (int)((Mutual_AVG[j*xnum+i]*(1000+IPOSTIVE_TOLERANCE))/1000);
            tp_log_debug("(%ld * (1000 + %d))/1000 = %d\n", Mutual_AVG[j*xnum+i], IPOSTIVE_TOLERANCE,kk);
            if(cm_data[j*xnum + i]  > kk )    {
                record_result[j*xnum + i] |= 2;
            }        
        }
    }

    // Record Test Result
    for(j=0;j<ynum;j++){
        for(i=0; i<xnum;i++){
            kk = 0;
            if((record_result[j*xnum+ i] & 0x01) > 0){
                kk++;
            }
            if((record_result[j*xnum+ i] & 0x02) > 0){
                kk++;
            }
        }
    }

    // TEST Result log
    for(j = 0; j<ynum; j++){
        for(i=0; i< xnum; i++){
           tp_log_debug("%d, ", record_result[xnum*j +i]);
        }
        tp_log_debug("\n");
    }
    tp_log_debug("\n");

    tp_log_info("%s line%d: begin to reset ic\n",__func__, __LINE__);
    nvt_hw_reset();
    
    tp_log_info("%s: begin to save open test data!\n",__func__);

    if(NULL == buf){
        tp_log_err("%s: buf is NULL\n", __func__);
        return 0;
    }
    
    if(kk >=1){ 
        //size = snprintf(buf, sizeof(tmp_data), "Sensor Open Test ,FAIL\n");             
        size = snprintf(buf, length, "Open Test FAIL\n");            
        *result = false;
        tp_log_err("%s: Sensor Open Test, FAIL\n", __func__);
    }else{ 
        //size = snprintf(tmp_data, sizeof(tmp_data), "Sensor Open Test ,PASS\n"); 
        size = snprintf(buf, length, "Open Test PASS\n"); 
        *result = true;
        tp_log_info("%s: Sensor Open Test, PASS\n", __func__);
    }

    for(j = 0; j<ynum; j++){
        for(i=0; i< xnum; i++){
            size += snprintf(tmp_data,sizeof(tmp_data), "%d,", cm_data[xnum*j +i]);
            strncat(buf, tmp_data, length-size);
        }
        size += snprintf(tmp_data, sizeof(tmp_data),"\n");
        strncat(buf, tmp_data, length-size);
    }
    size += snprintf(tmp_data, sizeof(tmp_data),"\n");
    strncat(buf, tmp_data, length-size);
    
    return size;
}


static ssize_t nvt_sysfs_scap_test_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    ssize_t size = 0;
    char * tmp_buf = NULL;
    bool short_pass = true;
    bool open_pass = true;

    tmp_buf = kmalloc(PAGE_SIZE, GFP_KERNEL);
    if(NULL == tmp_buf){
        tp_log_err("%s line%d: tmp_buf kmalloc failed!\n",__func__,__LINE__);
        return snprintf(buf,PAGE_SIZE,"FAIL\n tmp_buf kmalloc failed!\n");
        return 0;
    }

    if(!mutex_trylock(&ts->nvt_mutex)){
        tp_log_info("%s line%d: get nvt_mutex failed!\n",__func__,__LINE__);
        kfree(tmp_buf);
        return snprintf(buf,PAGE_SIZE,"FAIL\n get nvt_mutex failed!\n");
    }

    nvt_stop_esd_timer();

    flush_workqueue(ts->nvt_wq);
    disable_irq(ts->client->irq);
    nvt_pm_runtime_get(ts); 
    
    size += nvt_sysfs_sensor_short_test(tmp_buf+size, PAGE_SIZE-size, &short_pass);
    msleep(500);
    size += nvt_sysfs_sensor_open_test(tmp_buf+size, PAGE_SIZE-size, &open_pass);

    nvt_pm_runtime_put(ts);
    enable_irq(ts->client->irq);

    nvt_start_esd_timer();
    mutex_unlock(&ts->nvt_mutex);

    if(short_pass && open_pass){
        size = snprintf(buf, PAGE_SIZE, "PASS\n");
    }else{
        size = snprintf(buf, PAGE_SIZE, "FAIL\n");
    }
    size += snprintf(buf+size, PAGE_SIZE-size, tmp_buf);

    kfree(tmp_buf);

    return size;
}


static void ReadRaw_NT11205(unsigned int startAddr)
{
    uint8_t buffer[64]={0};
    int i, k, m;
    int bytecount, sec_ct, Residual_1, sec_ct2, Residual_2, temp_cnt, offsetAddr;

    temp_cnt=0;
    bytecount=xnum*ynum*2;
    sec_ct=bytecount/244;
    Residual_1=bytecount%244;
    sec_ct2=Residual_1/61;
    Residual_2=Residual_1%61;

    for(m=0;m<sec_ct;m++)
    {
        offsetAddr=0;
        buffer[0]=0xFF;
        buffer[1]=startAddr>>8;
        buffer[2]=startAddr&0xFF;
        nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 3);
        msleep(1);

        for(k=0;k<4;k++)
        {
            buffer[0]=offsetAddr&0xFF;
            nvt_i2c_read(ts->client, I2C_FW_Address, buffer, 62);
            offsetAddr+=61;
            for(i=0;i<61;i++)
            {
                DiffTemp[temp_cnt++]=(int)buffer[1+i];
            }
        }
        startAddr+=offsetAddr;
    }

    if(Residual_1>0)
    {
        offsetAddr=0;
        buffer[0]=0xFF;
        buffer[1]=startAddr>>8;
        buffer[2]=startAddr&0xFF;
        nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 3);
        msleep(1);

        for(k=0;k<sec_ct2;k++)
        {
            buffer[0]=offsetAddr&0xFF;
            nvt_i2c_read(ts->client, I2C_FW_Address, buffer, 62);
            offsetAddr+=61;
            for(i=0;i<61;i++)
            {
                DiffTemp[temp_cnt++]=(int)buffer[1+i];
            }
        }
        startAddr+=offsetAddr;
        if(Residual_2>0)
        {
            offsetAddr=0;
            buffer[0]=0xFF;
            buffer[1]=startAddr>>8;
            buffer[2]=startAddr&0xFF;
            nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 3);
            msleep(1);

            buffer[0]=offsetAddr&0xFF;
            nvt_i2c_read(ts->client, I2C_FW_Address, buffer, Residual_2+1);
            for(i=0;i<Residual_2;i++)
            {
                DiffTemp[temp_cnt++]=(int)buffer[1+i];
            }
        }
    }


    //---Unlock Frame---
    buffer[0]=0xFF;
    buffer[1]=0x3D;
    buffer[2]=0xFA;
    nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 3);
    msleep(1);

    buffer[0]=0x00;
    buffer[1]=0xBB;
    nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 2);

    msleep(100);
}

static ssize_t GetRawSample(char *buf, ssize_t length)
{
    uint8_t buffer[8]={0};
    int i, j;
    ssize_t size = 0;

    //Get XY channel number
    nvt_read_touch_info();

    //---Enter Debug Mode---
    buffer[0]=0xFF;
    buffer[1]=0x3D;
    buffer[2]=0xFC;
    nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 3);
    msleep(1);

    buffer[0]=0x00;
    buffer[1]=0xAA;
    buffer[2]=0x5B;
    nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 3);
    msleep(500);

    //---Read RawData---
    /*ReadRaw_NT11205(0x1E40);
    printk("RawData:\n");
    for(j=0; j<ynum; j++)
    {
        for(i=0; i< xnum; i++)
        {
            mutual_data[j*xnum+i] = ((DiffTemp[2*(j*xnum+i)]<<8)|DiffTemp[2*(j*xnum+i)+1]);
            size += snprintf(buf,length - size,"%5d,", mutual_data[j*xnum+i]);
            printk("%5d,", mutual_data[j*xnum+i]);
        }
        size += snprintf(buf,length - size,"\n");
        printk("\n");
    }
    size += snprintf(buf,length - size,"\n");
    printk("\n");*/

    //---Read DiffData---
    ReadRaw_NT11205(0x1200);
    printk("DiffData:\n");
    for(j=0; j<ynum; j++)
    {
        for(i=0; i< xnum; i++)
        {
            mutual_data[j*xnum+i] = ((DiffTemp[2*(j*xnum+i)]<<8)|DiffTemp[2*(j*xnum+i)+1]);
            if(mutual_data[j*xnum+i] >= 65536/2)
            {
                mutual_data[j*xnum+i] = mutual_data[j*xnum+i]-65536;
            }
            size += snprintf(buf+size, length - size,"%5d,",mutual_data[j*xnum+i]);
            printk("%5d,", mutual_data[j*xnum+i]);
        }
        size += snprintf(buf+size,length - size,"\n");
        printk("\n");
    }

    //---Quit Debug Mode---
    buffer[0]=0xFF;
    buffer[1]=0x3D;
    buffer[2]=0xFC;
    nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 3);
    msleep(1);

    buffer[0]=0x00;
    buffer[1]=0x00;
    buffer[2]=0x00;
    nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 3);

    //---Unlock Frame---
    buffer[0]=0xFF;
    buffer[1]=0x3D;
    buffer[2]=0xFA;
    nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 3);
    msleep(1);

    buffer[0]=0x00;
    buffer[1]=0xBB;
    nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 2);

    //---Set Default I2C Buffer Index---
    buffer[0]=0xFF;
    buffer[1]=0x3F;
    buffer[2]=0x00;
    nvt_i2c_write(ts->client, I2C_FW_Address, buffer, 3);

    return size;
}

//---Device Node for Driver Version Check---
static ssize_t nvt_sysfs_read_differ_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    char * tmp_buf = NULL;
    ssize_t size = 0;

    tmp_buf = kmalloc(PAGE_SIZE,GFP_KERNEL);
    if(NULL == tmp_buf){ 
        tp_log_err("%s: tmp_buf kmalloc failed!\n", __func__);
        return snprintf(buf,PAGE_SIZE,"tmp_buf kmalloc failed!\n");
    }

    nvt_stop_esd_timer();
    
    disable_irq(ts->client->irq);    
    nvt_pm_runtime_get(ts);
    size = GetRawSample(tmp_buf, PAGE_SIZE);
    enable_irq(ts->client->irq);
    nvt_pm_runtime_put(ts);

    nvt_start_esd_timer();

    size = snprintf(buf,PAGE_SIZE,tmp_buf);
    kfree(tmp_buf);

    return size;
}


/* SYSFS : Device Attributes */
static DEVICE_ATTR(reset_touch,                S_IRUGO,    nvt_sysfs_reset_touch_show,           NULL);
static DEVICE_ATTR(scap_test,              S_IRUGO,    nvt_sysfs_scap_test_show,                     NULL);
static DEVICE_ATTR(read_differ,              S_IRUGO,    nvt_sysfs_read_differ_show,       NULL);

static struct attribute *nvt_ts_attributes[] = {
    &dev_attr_reset_touch.attr,
    &dev_attr_scap_test.attr,
    &dev_attr_read_differ.attr,  
    NULL
};

static struct attribute_group nvt_ts_attribute_group = {
    .attrs = nvt_ts_attributes
};

void nvt_sysfs_test_init(void)
{
    int ret=0;
    struct kobject *kobject_ts = NULL;

    init_sample_test_parms(ts);

    kobject_ts = tp_get_touch_screen_obj();
    if (!kobject_ts) {
        tp_log_err("%s line%d: create kobjetct error!\n",__func__,__LINE__);
        return ;
    }

    ret = sysfs_create_group(kobject_ts, &nvt_ts_attribute_group);
    if (ret){
        tp_log_err("%s line%d: ret = %d,sysfs create_group failed!\n",__func__, __LINE__,ret);
    }else{
        tp_log_info("%s line%d: sysfs create_group succeed.\n", __func__, __LINE__);
    }
}

