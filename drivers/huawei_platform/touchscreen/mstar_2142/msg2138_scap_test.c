
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include "msg2138_scap_test.h"
#include "msg2138_qc.h"
#include <linux/i2c-qup.h>
#include <linux/slab.h>

static struct mutex scap_test_mutex;



extern struct dsm_client * tp_dclient;

extern struct msg21xx_ts_data * msg21xx_data;
extern struct i2c_client *captest_i2c_client;
extern unsigned int g_focal_rst_gpio;
//#define DMA_IIC //modify: MTK??¨¬¡§3?1y8byte¡À?D?¨º1¨®?DMA¡¤?¨º???DDiic¨ª¡§D?
#ifdef DMA_IIC
#include <linux/dma-mapping.h>
static unsigned char *I2CDMABuf_va = NULL;
static volatile unsigned int I2CDMABuf_pa = NULL;
static void _msg_dma_alloc(void)
{
    I2CDMABuf_va = (u8 *)dma_alloc_coherent(NULL, 4096, &I2CDMABuf_pa, GFP_KERNEL);
}

static void _msg_dma_free(void)
{
    if(NULL!=I2CDMABuf_va)
    {
        dma_free_coherent(NULL, 4096, I2CDMABuf_va, I2CDMABuf_pa);
	    I2CDMABuf_va = NULL;
	    I2CDMABuf_pa = 0;
    }
}
#endif


//modify:
#include <open_test_ANA1_OFILM.h>
#include <open_test_ANA2_OFILM.h>
#include <open_test_ANA1_B_OFILM.h>
#include <open_test_ANA2_B_OFILM.h>
#include <open_test_ANA3_OFILM.h>

#include <open_test_ANA1_OFILM_ULC02.h>
#include <open_test_ANA2_OFILM_ULC02.h>
#include <open_test_ANA1_B_OFILM_ULC02.h>
#include <open_test_ANA2_B_OFILM_ULC02.h>
#include <open_test_ANA3_OFILM_ULC02.h>
#include <open_test_ANA1_EELY_ULC02.h>
#include <open_test_ANA2_EELY_ULC02.h>
#include <open_test_ANA1_B_EELY_ULC02.h>
#include <open_test_ANA2_B_EELY_ULC02.h>
#include <open_test_ANA3_EELY_ULC02.h>

#include <open_test_ANA1_MUTTO_ULC02.h>
#include <open_test_ANA2_MUTTO_ULC02.h>
#include <open_test_ANA1_B_MUTTO_ULC02.h>
#include <open_test_ANA2_B_MUTTO_ULC02.h>
#include <open_test_ANA3_MUTTO_ULC02.h>


///////////////////////////////////////////////////////////////////////////
u8 bItoTestDebug = 1;
#define ITO_TEST_DEBUG(format, ...) \
{ \
    if(bItoTestDebug) \
    { \
        printk(KERN_ERR "ito_test ***" format "\n", ## __VA_ARGS__); \
        mdelay(5); \
    } \
}
#define ITO_TEST_DEBUG_MUST(format, ...)    \
    printk(KERN_ERR "ito_test ***" format "\n", ## __VA_ARGS__);mdelay(5)

static int g_i2c_freq = 0;

s16  s16_raw_data_1[48] = {0};
s16  s16_raw_data_2[48] = {0};
s16  s16_raw_data_3[48] = {0};
u8 ito_test_keynum = 0;
u8 ito_test_dummynum = 0;
u8 ito_test_trianglenum = 0;
u8 ito_test_2r = 0;
u8 g_LTP = 1;
uint16_t *open_1 = NULL;
uint16_t *open_1B = NULL;
uint16_t *open_2 = NULL;
uint16_t *open_2B = NULL;
uint16_t *open_3 = NULL;
u8 *MAP1 = NULL;
u8 *MAP2=NULL;
u8 *MAP3=NULL;
u8 *MAP40_1 = NULL;
u8 *MAP40_2 = NULL;
u8 *MAP40_3 = NULL;
u8 *MAP40_4 = NULL;
u8 *MAP41_1 = NULL;
u8 *MAP41_2 = NULL;
u8 *MAP41_3 = NULL;
u8 *MAP41_4 = NULL;

#define ITO_TEST_ADDR_TP  (0x4C>>1)
#define ITO_TEST_ADDR_REG (0xC4>>1)
#define REG_INTR_FIQ_MASK           0x04
#define FIQ_E_FRAME_READY_MASK      ( 1 << 8 )
#define MAX_CHNL_NUM (48)
#define BIT0  (1<<0)
#define BIT1  (1<<1)
#define BIT2  (1<<2)
#define BIT5  (1<<5)
#define BIT11 (1<<11)
#define BIT15 (1<<15)
static int ito_test_i2c_read(U8 addr, U8* read_data, U16 size)
{
    int rc;
    U8 before_addr = captest_i2c_client->addr;
    captest_i2c_client->addr = addr;

#ifdef DMA_IIC
    if(size>8&&NULL!=I2CDMABuf_va)
    {
        int i = 0;
        captest_i2c_client->ext_flag = captest_i2c_client->ext_flag | I2C_DMA_FLAG ;
        rc = i2c_master_recv(captest_i2c_client, (unsigned char *)I2CDMABuf_pa, size);
        for(i = 0; i < size; i++)
   		{
        	read_data[i] = I2CDMABuf_va[i];
    	}
    }
    else
    {
        rc = i2c_master_recv(captest_i2c_client, read_data, size);
    }
    captest_i2c_client->ext_flag = captest_i2c_client->ext_flag & (~I2C_DMA_FLAG);	
#else
    rc = i2c_master_recv(captest_i2c_client, read_data, size);
#endif
    captest_i2c_client->addr = before_addr;
    if( rc < 0 )
    {
        ITO_TEST_DEBUG_MUST("ito_test_i2c_read error %d,addr=%d,size=%d\n", rc,addr,size);

        msg21xx_report_dsm_erro(msg21xx_data, tp_dclient, DSM_TP_I2C_RW_ERROR_NO, rc);
        
    }
    return rc;
}

static int ito_test_i2c_write(u8 addr, u8* data, u16 size)//modify : ?¨´?Y????DT?? i2c_client
{
    int rc;
    u8 before_addr = captest_i2c_client->addr;
    captest_i2c_client->addr = addr;

#ifdef DMA_IIC
    //printk("hisdar debug:DMA\n");
    if(size>8&&NULL!=I2CDMABuf_va)
	{
	    int i = 0;
	    for(i=0;i<size;i++)
    	{
    		 I2CDMABuf_va[i]=data[i];
    	}
		captest_i2c_client->ext_flag = captest_i2c_client->ext_flag | I2C_DMA_FLAG ;
		rc = i2c_master_send(captest_i2c_client, (unsigned char *)I2CDMABuf_pa, size);
	}
	else
	{
		rc = i2c_master_send(captest_i2c_client, data, size);
	}
    captest_i2c_client->ext_flag = captest_i2c_client->ext_flag & (~I2C_DMA_FLAG);	
#else
    //printk("hisdar debug:nomal\n");
    rc = i2c_master_send(captest_i2c_client, data, size);
#endif
    captest_i2c_client->addr = before_addr;
    if( rc < 0 )
    {
        ITO_TEST_DEBUG_MUST("ito_test_i2c_write error %d,addr = %d,data[0]=%d,size=%d\n", rc, addr,data[0],size);
        
        msg21xx_report_dsm_erro(msg21xx_data, tp_dclient, DSM_TP_I2C_RW_ERROR_NO, rc);
        
    }

    //printk("hisdar debug:finish write\n");
    return rc;
}
static void ito_test_reset(void)//modify:?¨´?Y????DT??
{
    gpio_set_value(g_focal_rst_gpio, 1);
    mdelay(20);  /* Note that the RST must be in LOW 10ms at least */
    gpio_set_value(g_focal_rst_gpio, 0);
    mdelay(100);
    gpio_set_value(g_focal_rst_gpio, 1);
    /* Enable the interrupt service thread/routine for INT after 50ms */
    mdelay(200);/*Added by liumx 2013.11.22*/
	
}
static void ito_test_disable_irq(void)//modify:?¨´?Y????DT??
{
	msg2138_disable_irq();
}
static void ito_test_enable_irq(void)//modify:?¨´?Y????DT??
{
	msg2138_enable_irq();
}

//static void ito_test_set_iic_rate(u32 iicRate)//modify:?¨´?Y??¨¬¡§DT??,iic?¨´?¨º¨°a?¨®50K



static void ito_test_WriteReg( u8 bank, u8 addr, u16 data )
{
    u8 tx_data[5] = {0x10, bank, addr, data & 0xFF, data >> 8};
    ito_test_i2c_write( ITO_TEST_ADDR_REG, &tx_data[0], 5 );
}
static void ito_test_WriteReg8Bit( u8 bank, u8 addr, u8 data )
{
    u8 tx_data[4] = {0x10, bank, addr, data};
    ito_test_i2c_write ( ITO_TEST_ADDR_REG, &tx_data[0], 4 );
}
static unsigned short ito_test_ReadReg( u8 bank, u8 addr )
{
    u8 tx_data[3] = {0x10, bank, addr};
    u8 rx_data[2] = {0};

    ito_test_i2c_write( ITO_TEST_ADDR_REG, &tx_data[0], 3 );
    ito_test_i2c_read ( ITO_TEST_ADDR_REG, &rx_data[0], 2 );
    return ( rx_data[1] << 8 | rx_data[0] );
}

//modify:
#define TP_OF_OFILM         (1)
static u32 ito_test_choose_TpType(void)
{
    u16 tpType = msg21xx_data->vendor_id;
    open_1 = NULL;
    open_1B = NULL;
    open_2 = NULL;
    open_2B = NULL;
    open_3 = NULL;
    MAP1 = NULL;
    MAP2 = NULL;
    MAP3 = NULL;
    MAP40_1 = NULL;
    MAP40_2 = NULL;
    MAP40_3 = NULL;
    MAP40_4 = NULL;
    MAP41_1 = NULL;
    MAP41_2 = NULL;
    MAP41_3 = NULL;
    MAP41_4 = NULL;
    ito_test_keynum = 0;
    ito_test_dummynum = 0;
    ito_test_trianglenum = 0;
    ito_test_2r = 0;
    
    if(strncmp(msg21xx_data->product_name, PHONE_NAME_Y550, sizeof(PHONE_NAME_Y550)) == 0)
    {
        //scap_test range for ofilm, y550 only use ofilm
        open_1 = open_1_OFILM;
        open_1B = open_1B_OFILM;
        open_2 = open_2_OFILM;
        open_2B = open_2B_OFILM;
        open_3 = open_3_OFILM;
        MAP1 = MAP1_OFILM;
        MAP2 = MAP2_OFILM;
        MAP3 = MAP3_OFILM;
        MAP40_1 = MAP40_1_OFILM;
        MAP40_2 = MAP40_2_OFILM;
        MAP40_3 = MAP40_3_OFILM;
        MAP40_4 = MAP40_4_OFILM;
        MAP41_1 = MAP41_1_OFILM;
        MAP41_2 = MAP41_2_OFILM;
        MAP41_3 = MAP41_3_OFILM;
        MAP41_4 = MAP41_4_OFILM;
        ito_test_keynum = NUM_KEY_OFILM;
        ito_test_dummynum = NUM_DUMMY_OFILM;
        ito_test_trianglenum = NUM_SENSOR_OFILM;
        ito_test_2r = ENABLE_2R_OFILM;                    
        tp_log_info("%s %d: select range for Y550\n", __func__, __LINE__);
    }
    else if(strncmp(msg21xx_data->product_name, PHONE_NAME_ULC02, sizeof(PHONE_NAME_ULC02)) == 0)
    {
        switch(tpType)
        {
            case SWID_OFILM: //scap_test range for ofilm
                open_1 = open_1_OFILM_ULC02;
                open_1B = open_1B_OFILM_ULC02;
                open_2 = open_2_OFILM_ULC02;
                open_2B = open_2B_OFILM_ULC02;
                open_3 = open_3_OFILM_ULC02;
                MAP1 = MAP1_OFILM_ULC02;
                MAP2 = MAP2_OFILM_ULC02;
                MAP3 = MAP3_OFILM_ULC02;
                MAP40_1 = MAP40_1_OFILM_ULC02;
                MAP40_2 = MAP40_2_OFILM_ULC02;
                MAP40_3 = MAP40_3_OFILM_ULC02;
                MAP40_4 = MAP40_4_OFILM_ULC02;
                MAP41_1 = MAP41_1_OFILM_ULC02;
                MAP41_2 = MAP41_2_OFILM_ULC02;
                MAP41_3 = MAP41_3_OFILM_ULC02;
                MAP41_4 = MAP41_4_OFILM_ULC02;
                ito_test_keynum = NUM_KEY_OFILM_ULC02;
                ito_test_dummynum = NUM_DUMMY_OFILM_ULC02;
                ito_test_trianglenum = NUM_SENSOR_OFILM_ULC02;
                ito_test_2r = ENABLE_2R_OFILM_ULC02;             
                break;
            case SWID_EELY: //scap_test range for eely
                open_1 = open_1_EELY_ULC02;
                open_1B = open_1B_EELY_ULC02;
                open_2 = open_2_EELY_ULC02;
                open_2B = open_2B_EELY_ULC02;
                open_3 = open_3_EELY_ULC02;
                MAP1 = MAP1_EELY_ULC02;
                MAP2 = MAP2_EELY_ULC02;
                MAP3 = MAP3_EELY_ULC02;
                MAP40_1 = MAP40_1_EELY_ULC02;
                MAP40_2 = MAP40_2_EELY_ULC02;
                MAP40_3 = MAP40_3_EELY_ULC02;
                MAP40_4 = MAP40_4_EELY_ULC02;
                MAP41_1 = MAP41_1_EELY_ULC02;
                MAP41_2 = MAP41_2_EELY_ULC02;
                MAP41_3 = MAP41_3_EELY_ULC02;
                MAP41_4 = MAP41_4_EELY_ULC02;
                ito_test_keynum = NUM_KEY_EELY_ULC02;
                ito_test_dummynum = NUM_DUMMY_EELY_ULC02;
                ito_test_trianglenum = NUM_SENSOR_EELY_ULC02;
                ito_test_2r = ENABLE_2R_EELY_ULC02;                        
                break;
            case SWID_MUTTO: //scap_test range for mutto
                open_1 = open_1_MUTTO_ULC02;
                open_1B = open_1B_MUTTO_ULC02;
                open_2 = open_2_MUTTO_ULC02;
                open_2B = open_2B_MUTTO_ULC02;
                open_3 = open_3_MUTTO_ULC02;
                MAP1 = MAP1_MUTTO_ULC02;
                MAP2 = MAP2_MUTTO_ULC02;
                MAP3 = MAP3_MUTTO_ULC02;
                MAP40_1 = MAP40_1_MUTTO_ULC02;
                MAP40_2 = MAP40_2_MUTTO_ULC02;
                MAP40_3 = MAP40_3_MUTTO_ULC02;
                MAP40_4 = MAP40_4_MUTTO_ULC02;
                MAP41_1 = MAP41_1_MUTTO_ULC02;
                MAP41_2 = MAP41_2_MUTTO_ULC02;
                MAP41_3 = MAP41_3_MUTTO_ULC02;
                MAP41_4 = MAP41_4_MUTTO_ULC02;
                ito_test_keynum = NUM_KEY_MUTTO_ULC02;
                ito_test_dummynum = NUM_DUMMY_MUTTO_ULC02;
                ito_test_trianglenum = NUM_SENSOR_MUTTO_ULC02;
                ito_test_2r = ENABLE_2R_MUTTO_ULC02;                    
                break;
            default:  // use ofilm's range for default module.
                open_1 = open_1_OFILM_ULC02;
                open_1B = open_1B_OFILM_ULC02;
                open_2 = open_2_OFILM_ULC02;
                open_2B = open_2B_OFILM_ULC02;
                open_3 = open_3_OFILM_ULC02;
                MAP1 = MAP1_OFILM_ULC02;
                MAP2 = MAP2_OFILM_ULC02;
                MAP3 = MAP3_OFILM_ULC02;
                MAP40_1 = MAP40_1_OFILM_ULC02;
                MAP40_2 = MAP40_2_OFILM_ULC02;
                MAP40_3 = MAP40_3_OFILM_ULC02;
                MAP40_4 = MAP40_4_OFILM_ULC02;
                MAP41_1 = MAP41_1_OFILM_ULC02;
                MAP41_2 = MAP41_2_OFILM_ULC02;
                MAP41_3 = MAP41_3_OFILM_ULC02;
                MAP41_4 = MAP41_4_OFILM_ULC02;
                ito_test_keynum = NUM_KEY_OFILM_ULC02;
                ito_test_dummynum = NUM_DUMMY_OFILM_ULC02;
                ito_test_trianglenum = NUM_SENSOR_OFILM_ULC02;
                ito_test_2r = ENABLE_2R_OFILM_ULC02;                 
                break;
        }    
        tp_log_info("%s %d: select range for ULC02\n", __func__, __LINE__);
    }

    tp_log_info("%s %d: select scap range for vendor_id = %u\n", __func__, __LINE__, tpType);
    return tpType;
}
static void ito_test_EnterSerialDebugMode(void)
{
    u8 data[5];

    data[0] = 0x53;
    data[1] = 0x45;
    data[2] = 0x52;
    data[3] = 0x44;
    data[4] = 0x42;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, &data[0], 5);

    data[0] = 0x37;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, &data[0], 1);

    data[0] = 0x35;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, &data[0], 1);

    data[0] = 0x71;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, &data[0], 1);
}
static uint16_t ito_test_get_num( void )
{
    uint16_t    num_of_sensor,i;
    uint16_t 	RegValue1,RegValue2;
 
    num_of_sensor = 0;
        
    RegValue1 = ito_test_ReadReg( 0x11, 0x4A);
    ITO_TEST_DEBUG("ito_test_get_num,RegValue1=%d\n",RegValue1);
    if ( ( RegValue1 & BIT1) == BIT1 )
    {
    	RegValue1 = ito_test_ReadReg( 0x12, 0x0A);			
    	RegValue1 = RegValue1 & 0x0F;
    	
    	RegValue2 = ito_test_ReadReg( 0x12, 0x16);    		
    	RegValue2 = (( RegValue2 >> 1 ) & 0x0F) + 1;
    	
    	num_of_sensor = RegValue1 * RegValue2;
    }
	else
	{
	    for(i=0;i<4;i++)
	    {
	        num_of_sensor+=(ito_test_ReadReg( 0x12, 0x0A)>>(4*i))&0x0F;
	    }
	}
    ITO_TEST_DEBUG("ito_test_get_num,num_of_sensor=%d\n",num_of_sensor);
    return num_of_sensor;        
}
static void ito_test_polling( void )
{
    uint16_t    reg_int = 0x0000;
    uint8_t     dbbus_tx_data[5];
    uint8_t     dbbus_rx_data[4];
    uint16_t    reg_value;

    reg_int = 0;

    ito_test_WriteReg( 0x13, 0x0C, BIT15 );       
    ito_test_WriteReg( 0x12, 0x14, (ito_test_ReadReg(0x12,0x14) | BIT0) );         
            
    ITO_TEST_DEBUG("polling start\n");
    while( ( reg_int & BIT0 ) == 0x0000 )
    {
        dbbus_tx_data[0] = 0x10;
        dbbus_tx_data[1] = 0x3D;
        dbbus_tx_data[2] = 0x18;
        ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 3);
        ito_test_i2c_read(ITO_TEST_ADDR_REG,  dbbus_rx_data, 2);
        reg_int = dbbus_rx_data[1];
    }
    ITO_TEST_DEBUG("polling end\n");
    reg_value = ito_test_ReadReg( 0x3D, 0x18 ); 
    ito_test_WriteReg( 0x3D, 0x18, reg_value & (~BIT0) );      
}
static uint16_t ito_test_get_data_out( int16_t* s16_raw_data )
{
    uint8_t     i,dbbus_tx_data[8];
    uint16_t    raw_data[48]={0};
    uint16_t    num_of_sensor;
    uint16_t    reg_int;
    uint8_t		dbbus_rx_data[96]={0};
  
    num_of_sensor = ito_test_get_num();
    if(num_of_sensor*2>96)
    {
        ITO_TEST_DEBUG("danger,num_of_sensor=%d\n",num_of_sensor);
        return num_of_sensor;
    }

    reg_int = ito_test_ReadReg( 0x3d, REG_INTR_FIQ_MASK<<1 ); 
    ito_test_WriteReg( 0x3d, REG_INTR_FIQ_MASK<<1, (reg_int & (uint16_t)(~FIQ_E_FRAME_READY_MASK) ) ); 
    ito_test_polling();
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x13;
    dbbus_tx_data[2] = 0x40;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 3);
    mdelay(20);
    ito_test_i2c_read(ITO_TEST_ADDR_REG, &dbbus_rx_data[0], (num_of_sensor * 2));
    mdelay(100);
    for(i=0;i<num_of_sensor * 2;i++)
    {
        ITO_TEST_DEBUG("dbbus_rx_data[%d]=%d\n",i,dbbus_rx_data[i]);
    }
 
    reg_int = ito_test_ReadReg( 0x3d, REG_INTR_FIQ_MASK<<1 ); 
    ito_test_WriteReg( 0x3d, REG_INTR_FIQ_MASK<<1, (reg_int | (uint16_t)FIQ_E_FRAME_READY_MASK ) ); 

    for( i = 0; i < num_of_sensor; i++ )
    {
        raw_data[i] = ( dbbus_rx_data[ 2 * i + 1] << 8 ) | ( dbbus_rx_data[2 * i] );
        s16_raw_data[i] = ( int16_t )raw_data[i];
    }
    
    return(num_of_sensor);
}


static void ito_test_send_data_in( uint8_t step )
{
    uint16_t	i;
    uint8_t 	dbbus_tx_data[512];
    uint16_t 	*Type1=NULL;        

    ITO_TEST_DEBUG("ito_test_send_data_in step=%d\n",step);
	if( step == 4 )
    {
        Type1 = &open_1[0];        
    }
    else if( step == 5 )
    {
        Type1 = &open_2[0];      	
    }
    else if( step == 6 )
    {
        Type1 = &open_3[0];      	
    }
    else if( step == 9 )
    {
        Type1 = &open_1B[0];        
    }
    else if( step == 10 )
    {
        Type1 = &open_2B[0];      	
    } 
    else
    {
        tp_log_err("%s %d: ilegal parm: step = %d\n", __func__, __LINE__, step);
        return;
    }
     
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0x00;    
    for( i = 0; i <= 0x3E ; i++ )
    {
        dbbus_tx_data[3+2*i] = Type1[i] & 0xFF;
        dbbus_tx_data[4+2*i] = ( Type1[i] >> 8 ) & 0xFF;    	
    }
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 3+0x3F*2);
 
    dbbus_tx_data[2] = 0x7A * 2;
    for( i = 0x7A; i <= 0x7D ; i++ )
    {
        dbbus_tx_data[3+2*(i-0x7A)] = 0;
        dbbus_tx_data[4+2*(i-0x7A)] = 0;    	    	
    }
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 3+8);  
    
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x12;
      
    dbbus_tx_data[2] = 5 * 2;
    dbbus_tx_data[3] = Type1[128+5] & 0xFF;
    dbbus_tx_data[4] = ( Type1[128+5] >> 8 ) & 0xFF;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 5);
    
    dbbus_tx_data[2] = 0x0B * 2;
    dbbus_tx_data[3] = Type1[128+0x0B] & 0xFF;
    dbbus_tx_data[4] = ( Type1[128+0x0B] >> 8 ) & 0xFF;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 5);
    
    dbbus_tx_data[2] = 0x12 * 2;
    dbbus_tx_data[3] = Type1[128+0x12] & 0xFF;
    dbbus_tx_data[4] = ( Type1[128+0x12] >> 8 ) & 0xFF;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 5);
    
    dbbus_tx_data[2] = 0x15 * 2;
    dbbus_tx_data[3] = Type1[128+0x15] & 0xFF;
    dbbus_tx_data[4] = ( Type1[128+0x15] >> 8 ) & 0xFF;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 5);        
    //for AC mod --showlo
    dbbus_tx_data[1] = 0x13;
    dbbus_tx_data[2] = 0x12 * 2;
    dbbus_tx_data[3] = 0X30;
    dbbus_tx_data[4] = 0X30;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 5);        

    
    dbbus_tx_data[2] = 0x14 * 2;
    dbbus_tx_data[3] = 0X30;
    dbbus_tx_data[4] = 0X30;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 5);     

    
    dbbus_tx_data[1] = 0x12;
    for (i = 0x0D; i <= 0x10;i++ )//for AC noise(++)
    {
        dbbus_tx_data[2] = i * 2;
        dbbus_tx_data[3] = Type1[128+i] & 0xFF;
        dbbus_tx_data[4] = ( Type1[128+i] >> 8 ) & 0xFF;
        ito_test_i2c_write( ITO_TEST_ADDR_REG,  dbbus_tx_data,5 );  
    }

    for (i = 0x16; i <= 0x18; i++)//for AC noise
    {
        dbbus_tx_data[2] = i * 2;
        dbbus_tx_data[3] = Type1[128+i] & 0xFF;
        dbbus_tx_data[4] = ( Type1[128+i] >> 8 ) & 0xFF;
        ito_test_i2c_write( ITO_TEST_ADDR_REG, dbbus_tx_data,5 );  
    }
}

static void ito_test_set_v( uint8_t Enable, uint8_t Prs)	
{
    uint16_t    u16RegValue;        
    
    
    u16RegValue = ito_test_ReadReg( 0x12, 0x08);   
    u16RegValue = u16RegValue & 0xF1; 							
    if ( Prs == 0 )
    {
    	ito_test_WriteReg( 0x12, 0x08, u16RegValue| 0x0C); 		
    }
    else if ( Prs == 1 )
    {
    	ito_test_WriteReg( 0x12, 0x08, u16RegValue| 0x0E); 		     	
    }
    else
    {
    	ito_test_WriteReg( 0x12, 0x08, u16RegValue| 0x02); 			
    }    
    
    if ( Enable )
    {
        u16RegValue = ito_test_ReadReg( 0x11, 0x06);    
        ito_test_WriteReg( 0x11, 0x06, u16RegValue| 0x03);   	
    }
    else
    {
        u16RegValue = ito_test_ReadReg( 0x11, 0x06);    
        u16RegValue = u16RegValue & 0xFC;					
        ito_test_WriteReg( 0x11, 0x06, u16RegValue);         
    }

}

static void ito_test_set_c( uint8_t Csub_Step )
{
    uint8_t i;
    uint8_t dbbus_tx_data[MAX_CHNL_NUM+3];
    uint8_t HighLevel_Csub = false;
    uint8_t Csub_new;
     
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;        
    dbbus_tx_data[2] = 0x84;        
    for( i = 0; i < MAX_CHNL_NUM; i++ )
    {
		Csub_new = Csub_Step;        
        HighLevel_Csub = false;   
        if( Csub_new > 0x1F )
        {
            Csub_new = Csub_new - 0x14;
            HighLevel_Csub = true;
        }
           
        dbbus_tx_data[3+i] =    Csub_new & 0x1F;        
        if( HighLevel_Csub == true )
        {
            dbbus_tx_data[3+i] |= BIT5;
        }
    }
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, MAX_CHNL_NUM+3);

    dbbus_tx_data[2] = 0xB4;        
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, MAX_CHNL_NUM+3);
}

static void ito_test_sw( void )
{
    ito_test_WriteReg( 0x11, 0x00, 0xFFFF );
    ito_test_WriteReg( 0x11, 0x00, 0x0000 );
    mdelay( 50 );
}
void Disable_noise_detect( void )
{
    ito_test_WriteReg8Bit( 0x13, 0x02, (ito_test_ReadReg(0x13,0x01) & ~( BIT2|BIT0|BIT1 ) ) );
}

static void ito_test_first( uint8_t item_id , int16_t* s16_raw_data)		
{
	//uint8_t     result = 0,loop;
	//uint8_t     dbbus_tx_data[9];
	uint8_t loop;
	uint8_t     i,j;
    int16_t     s16_raw_data_tmp[48]={0};
	uint8_t     num_of_sensor, num_of_sensor2,total_sensor;
	uint16_t	u16RegValue;
    uint8_t 	*pMapping=NULL;
    
    	total_sensor=0;
	num_of_sensor = 0;
	num_of_sensor2 = 0;	
	
    ITO_TEST_DEBUG("ito_test_first item_id=%d\n",item_id);
	ito_test_WriteReg( 0x0F, 0xE6, 0x01 );

	ito_test_WriteReg( 0x1E, 0x24, 0x0500 );
	ito_test_WriteReg( 0x1E, 0x2A, 0x0000 );
	ito_test_WriteReg( 0x1E, 0xE6, 0x6E00 );
	ito_test_WriteReg( 0x1E, 0xE8, 0x0071 );
	    
    if ( item_id == 40 )    			
    {
        pMapping = &MAP1[0];
        if ( ito_test_2r )
		{
			total_sensor = ito_test_trianglenum/2; 
		}
		else
		{
		    total_sensor = ito_test_trianglenum/2 + ito_test_keynum + ito_test_dummynum;
		}
    }
    else if( item_id == 41 )    		
    {
        pMapping = &MAP2[0];
        if ( ito_test_2r )
		{
			total_sensor = ito_test_trianglenum/2; 
		}
		else
		{
		    total_sensor = ito_test_trianglenum/2 + ito_test_keynum + ito_test_dummynum;
		}
    }
    else if( item_id == 42 )    		
    {
        pMapping = &MAP3[0];      
        total_sensor =  ito_test_trianglenum + ito_test_keynum+ ito_test_dummynum; 
    }
        	    
	    
	loop = 1;
	if ( item_id != 42 )
	{
	    if(total_sensor>11)
        {
            loop = 2;
        }
	}	
    ITO_TEST_DEBUG("loop=%d\n",loop);
	for ( i = 0; i < loop; i++ )
	{
		if ( i == 0 )
		{
			ito_test_send_data_in( item_id - 36 );
		}
		else
		{ 
			if ( item_id == 40 ) 
				ito_test_send_data_in( 9 );
			else 		
				ito_test_send_data_in( 10 );
		}
		Disable_noise_detect();
		ito_test_set_v(1,0);    
		u16RegValue = ito_test_ReadReg( 0x11, 0x0E);    			
		ito_test_WriteReg( 0x11, 0x0E, u16RegValue | BIT11 );				 		
	
		if ( g_LTP == 1 )
	    	ito_test_set_c( 32 );	    	
		else	    	
	    	ito_test_set_c( 0 );
	    
		ito_test_sw();
		
		if ( i == 0 )	 
        {      
            num_of_sensor=ito_test_get_data_out(  s16_raw_data_tmp );
            ITO_TEST_DEBUG("num_of_sensor=%d;\n",num_of_sensor);
        }
		else	
        {      
            num_of_sensor2=ito_test_get_data_out(  &s16_raw_data_tmp[num_of_sensor] );
            ITO_TEST_DEBUG("num_of_sensor=%d;num_of_sensor2=%d\n",num_of_sensor,num_of_sensor2);
        }
	}
    for ( j = 0; j < total_sensor ; j ++ )
	{
		if ( g_LTP == 1 )
			s16_raw_data[pMapping[j]] = s16_raw_data_tmp[j] + 4096;
		else
			s16_raw_data[pMapping[j]] = s16_raw_data_tmp[j];	
	}	

	return;
}

typedef enum
{
	ITO_TEST_OK = 0,
	ITO_TEST_FAIL,
	ITO_TEST_GET_TP_TYPE_ERROR,
} ITO_TEST_RET;

ITO_TEST_RET ito_test_second (u8 item_id)
{
	u8 i = 0;
    
	s32  s16_raw_data_jg_tmp1 = 0;
	s32  s16_raw_data_jg_tmp2 = 0;
	s32  jg_tmp1_avg_Th_max =0;
	s32  jg_tmp1_avg_Th_min =0;
	s32  jg_tmp2_avg_Th_max =0;
	s32  jg_tmp2_avg_Th_min =0;

	u8  Th_Tri = 25;        
	u8  Th_bor = 30;        

	if ( item_id == 40 )    			
    {
        for (i=0; i<(ito_test_trianglenum/2)-2; i++)
        {
			s16_raw_data_jg_tmp1 += s16_raw_data_1[MAP40_1[i]];
		}
		for (i=0; i<2; i++)
        {
			s16_raw_data_jg_tmp2 += s16_raw_data_1[MAP40_2[i]];
		}
    }
    else if( item_id == 41 )    		
    {
        for (i=0; i<(ito_test_trianglenum/2)-2; i++)
        {
			s16_raw_data_jg_tmp1 += s16_raw_data_2[MAP41_1[i]];
		}
		for (i=0; i<2; i++)
        {
			s16_raw_data_jg_tmp2 += s16_raw_data_2[MAP41_2[i]];
		}
    }

	    jg_tmp1_avg_Th_max = (s16_raw_data_jg_tmp1 / ((ito_test_trianglenum/2)-2)) * ( 100 + Th_Tri) / 100 ;
	    jg_tmp1_avg_Th_min = (s16_raw_data_jg_tmp1 / ((ito_test_trianglenum/2)-2)) * ( 100 - Th_Tri) / 100 ;
        jg_tmp2_avg_Th_max = (s16_raw_data_jg_tmp2 / 2) * ( 100 + Th_bor) / 100 ;
	    jg_tmp2_avg_Th_min = (s16_raw_data_jg_tmp2 / 2 ) * ( 100 - Th_bor) / 100 ;
	
        ITO_TEST_DEBUG("item_id=%d;sum1=%d;max1=%d;min1=%d;sum2=%d;max2=%d;min2=%d\n",item_id,s16_raw_data_jg_tmp1,jg_tmp1_avg_Th_max,jg_tmp1_avg_Th_min,s16_raw_data_jg_tmp2,jg_tmp2_avg_Th_max,jg_tmp2_avg_Th_min);

	if ( item_id == 40 ) 
	{
		for (i=0; i<(ito_test_trianglenum/2)-2; i++)
	    {
			if (s16_raw_data_1[MAP40_1[i]] > jg_tmp1_avg_Th_max || s16_raw_data_1[MAP40_1[i]] < jg_tmp1_avg_Th_min) 
				return ITO_TEST_FAIL;
		}
		for (i=0; i<(ito_test_trianglenum/2)-3; i++)
        {
            if (s16_raw_data_1[MAP40_1[i]] > s16_raw_data_1[MAP40_1[i+1]] ) 
                return ITO_TEST_FAIL;
        }
		for (i=0; i<2; i++)
	    {
			if (s16_raw_data_1[MAP40_2[i]] > jg_tmp2_avg_Th_max || s16_raw_data_1[MAP40_2[i]] < jg_tmp2_avg_Th_min) 
				return ITO_TEST_FAIL;
		} 
	}

	if ( item_id == 41 ) 
	{
		for (i=0; i<(ito_test_trianglenum/2)-2; i++)
	    {
			if (s16_raw_data_2[MAP41_1[i]] > jg_tmp1_avg_Th_max || s16_raw_data_2[MAP41_1[i]] < jg_tmp1_avg_Th_min) 
				return ITO_TEST_FAIL;
		}
        for (i=0; i<(ito_test_trianglenum/2)-3; i++)
        {
            if (s16_raw_data_2[MAP41_1[i]] < s16_raw_data_2[MAP41_1[i+1]] ) 
                return ITO_TEST_FAIL;
        }

		for (i=0; i<2; i++)
	    {
			if (s16_raw_data_2[MAP41_2[i]] > jg_tmp2_avg_Th_max || s16_raw_data_2[MAP41_2[i]] < jg_tmp2_avg_Th_min) 
				return ITO_TEST_FAIL;
		} 
	}

	return ITO_TEST_OK;
	
}
ITO_TEST_RET ito_test_second_2r (u8 item_id)
{
	u8 i = 0;
    
	s32  s16_raw_data_jg_tmp1 = 0;
	s32  s16_raw_data_jg_tmp2 = 0;
	s32  s16_raw_data_jg_tmp3 = 0;
	s32  s16_raw_data_jg_tmp4 = 0;
	
	s32  jg_tmp1_avg_Th_max =0;
	s32  jg_tmp1_avg_Th_min =0;
	s32  jg_tmp2_avg_Th_max =0;
	s32  jg_tmp2_avg_Th_min =0;
	s32  jg_tmp3_avg_Th_max =0;
	s32  jg_tmp3_avg_Th_min =0;
	s32  jg_tmp4_avg_Th_max =0;
	s32  jg_tmp4_avg_Th_min =0;

	u8  Th_Tri = 25;    // non-border threshold    
	u8  Th_bor = 40;    // border threshold    

	if ( item_id == 40 )    			
    {
        for (i=0; i<(ito_test_trianglenum/4)-2; i++)
        {
			s16_raw_data_jg_tmp1 += s16_raw_data_1[MAP40_1[i]];  //first region: non-border 
		}
		for (i=0; i<2; i++)
        {
			s16_raw_data_jg_tmp2 += s16_raw_data_1[MAP40_2[i]];  //first region: border
		}

		for (i=0; i<(ito_test_trianglenum/4)-2; i++)
        {
			s16_raw_data_jg_tmp3 += s16_raw_data_1[MAP40_3[i]];  //second region: non-border
		}
		for (i=0; i<2; i++)
        {
			s16_raw_data_jg_tmp4 += s16_raw_data_1[MAP40_4[i]];  //second region: border
		}
    }



	
    else if( item_id == 41 )    		
    {
        for (i=0; i<(ito_test_trianglenum/4)-2; i++)
        {
			s16_raw_data_jg_tmp1 += s16_raw_data_2[MAP41_1[i]];  //first region: non-border
		}
		for (i=0; i<2; i++)
        {
			s16_raw_data_jg_tmp2 += s16_raw_data_2[MAP41_2[i]];  //first region: border
		}
		for (i=0; i<(ito_test_trianglenum/4)-2; i++)
        {
			s16_raw_data_jg_tmp3 += s16_raw_data_2[MAP41_3[i]];  //second region: non-border
		}
		for (i=0; i<2; i++)
        {
			s16_raw_data_jg_tmp4 += s16_raw_data_2[MAP41_4[i]];  //second region: border
		}
    }

	    jg_tmp1_avg_Th_max = (s16_raw_data_jg_tmp1 / ((ito_test_trianglenum/4)-2)) * ( 100 + Th_Tri) / 100 ;
	    jg_tmp1_avg_Th_min = (s16_raw_data_jg_tmp1 / ((ito_test_trianglenum/4)-2)) * ( 100 - Th_Tri) / 100 ;
        jg_tmp2_avg_Th_max = (s16_raw_data_jg_tmp2 / 2) * ( 100 + Th_bor) / 100 ;
	    jg_tmp2_avg_Th_min = (s16_raw_data_jg_tmp2 / 2) * ( 100 - Th_bor) / 100 ;
		jg_tmp3_avg_Th_max = (s16_raw_data_jg_tmp3 / ((ito_test_trianglenum/4)-2)) * ( 100 + Th_Tri) / 100 ;
	    jg_tmp3_avg_Th_min = (s16_raw_data_jg_tmp3 / ((ito_test_trianglenum/4)-2)) * ( 100 - Th_Tri) / 100 ;
        jg_tmp4_avg_Th_max = (s16_raw_data_jg_tmp4 / 2) * ( 100 + Th_bor) / 100 ;
	    jg_tmp4_avg_Th_min = (s16_raw_data_jg_tmp4 / 2) * ( 100 - Th_bor) / 100 ;
		
	
        ITO_TEST_DEBUG("item_id=%d;sum1=%d;max1=%d;min1=%d;sum2=%d;max2=%d;min2=%d;sum3=%d;max3=%d;min3=%d;sum4=%d;max4=%d;min4=%d;\n",item_id,s16_raw_data_jg_tmp1,jg_tmp1_avg_Th_max,jg_tmp1_avg_Th_min,s16_raw_data_jg_tmp2,jg_tmp2_avg_Th_max,jg_tmp2_avg_Th_min,s16_raw_data_jg_tmp3,jg_tmp3_avg_Th_max,jg_tmp3_avg_Th_min,s16_raw_data_jg_tmp4,jg_tmp4_avg_Th_max,jg_tmp4_avg_Th_min);




	if ( item_id == 40 ) 
	{
		for (i=0; i<(ito_test_trianglenum/4)-2; i++)
	    {
			if (s16_raw_data_1[MAP40_1[i]] > jg_tmp1_avg_Th_max || s16_raw_data_1[MAP40_1[i]] < jg_tmp1_avg_Th_min) 
				return ITO_TEST_FAIL;
		}
		
		for (i=0; i<2; i++)
	    {
			if (s16_raw_data_1[MAP40_2[i]] > jg_tmp2_avg_Th_max || s16_raw_data_1[MAP40_2[i]] < jg_tmp2_avg_Th_min) 
				return ITO_TEST_FAIL;
		} 
		
		for (i=0; i<(ito_test_trianglenum/4)-2; i++)
	    {
			if (s16_raw_data_1[MAP40_3[i]] > jg_tmp3_avg_Th_max || s16_raw_data_1[MAP40_3[i]] < jg_tmp3_avg_Th_min) 
				return ITO_TEST_FAIL;
		}
		
		for (i=0; i<2; i++)
	    {
			if (s16_raw_data_1[MAP40_4[i]] > jg_tmp4_avg_Th_max || s16_raw_data_1[MAP40_4[i]] < jg_tmp4_avg_Th_min) 
				return ITO_TEST_FAIL;
		} 
	}

	if ( item_id == 41 ) 
	{
		for (i=0; i<(ito_test_trianglenum/4)-2; i++)
	    {
			if (s16_raw_data_2[MAP41_1[i]] > jg_tmp1_avg_Th_max || s16_raw_data_2[MAP41_1[i]] < jg_tmp1_avg_Th_min) 
				return ITO_TEST_FAIL;
		}
		
		for (i=0; i<2; i++)
	    {
			if (s16_raw_data_2[MAP41_2[i]] > jg_tmp2_avg_Th_max || s16_raw_data_2[MAP41_2[i]] < jg_tmp2_avg_Th_min) 
				return ITO_TEST_FAIL;
		}
		
		for (i=0; i<(ito_test_trianglenum/4)-2; i++)
	    {
			if (s16_raw_data_2[MAP41_3[i]] > jg_tmp3_avg_Th_max || s16_raw_data_2[MAP41_3[i]] < jg_tmp3_avg_Th_min) 
				return ITO_TEST_FAIL;
		}
		
		for (i=0; i<2; i++)
	    {
			if (s16_raw_data_2[MAP41_4[i]] > jg_tmp4_avg_Th_max || s16_raw_data_2[MAP41_4[i]] < jg_tmp4_avg_Th_min) 
				return ITO_TEST_FAIL;
		} 

	}

	return ITO_TEST_OK;
	
}

static ITO_TEST_RET ito_test_interface(void)
{
    ITO_TEST_RET ret = ITO_TEST_OK;
    uint16_t i = 0;
    
    mutex_lock(&scap_test_mutex);

    msg21xx_stop_esd_timer(msg21xx_data);
    
#ifdef DMA_IIC
    _msg_dma_alloc();
#endif
	g_i2c_freq = qup_get_clk_freq(captest_i2c_client->adapter);
    qup_set_clk_freq(captest_i2c_client->adapter, I2C_FREQUENCY_50000);
    ITO_TEST_DEBUG("start\n");
    ito_test_disable_irq();
	ito_test_reset();
    if(ito_test_choose_TpType() >= SWID_NULL)
    {
        ITO_TEST_DEBUG("choose tpType fail\n");
        ret = ITO_TEST_GET_TP_TYPE_ERROR;
        goto ITO_TEST_END;
    }
    ito_test_EnterSerialDebugMode();
    mdelay(100);
    ITO_TEST_DEBUG("EnterSerialDebugMode\n");
    ito_test_WriteReg8Bit ( 0x0F, 0xE6, 0x01 );
    ito_test_WriteReg ( 0x3C, 0x60, 0xAA55 );
    ITO_TEST_DEBUG("stop mcu and disable watchdog V.005\n");   
    mdelay(50);
    
	for(i = 0;i < 48;i++)
	{
		s16_raw_data_1[i] = 0;
		s16_raw_data_2[i] = 0;
		s16_raw_data_3[i] = 0;
	}	
	
    ito_test_first(40, s16_raw_data_1);
    ITO_TEST_DEBUG("40 get s16_raw_data_1\n");

    ito_test_first(41, s16_raw_data_2);
    ITO_TEST_DEBUG("41 get s16_raw_data_2\n");

    /* delete key test */

    if(ito_test_2r)
    {
        ret=ito_test_second_2r(40);
    }
    else
    {
        ret=ito_test_second(40);
    }
    if(ITO_TEST_FAIL==ret)
    {
        goto ITO_TEST_END;
    }
    
   
    if(ito_test_2r)
    {
        ret=ito_test_second_2r(41);
    }
    else
    {
        ret=ito_test_second(41);
    }
    if(ITO_TEST_FAIL==ret)
    {
        goto ITO_TEST_END;
    }
    /* move to up to test */
    
    ITO_TEST_END:
#ifdef DMA_IIC
    _msg_dma_free();
#endif
    qup_set_clk_freq(captest_i2c_client->adapter, g_i2c_freq);
	ito_test_reset();
    ito_test_enable_irq();
    ITO_TEST_DEBUG("end\n");

    msg21xx_start_esd_timer(msg21xx_data);
    
    mutex_unlock(&scap_test_mutex);

    return ret;
}

#include <linux/proc_fs.h>

ITO_TEST_RET g_ito_test_ret = ITO_TEST_OK;

static int msg_scap_test_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int error= 0;

    /*--coverity--spintf-->snprintf--*/
    
    g_ito_test_ret = ito_test_interface();
    
    if(ITO_TEST_OK==g_ito_test_ret)
    {
        error = snprintf(buf, PAGE_SIZE, "PASS\n");
        ITO_TEST_DEBUG_MUST("ITO_TEST_OK");
    }
    else if(ITO_TEST_FAIL==g_ito_test_ret)
    {
        error = snprintf(buf, PAGE_SIZE, "FAIL\n");
        ITO_TEST_DEBUG_MUST("ITO_TEST_FAIL");
    }
    else if(ITO_TEST_GET_TP_TYPE_ERROR==g_ito_test_ret)
    {
        error = snprintf(buf, PAGE_SIZE, "FAIL\n");
        ITO_TEST_DEBUG_MUST("ITO_TEST_GET_TP_TYPE_ERROR");
    }
    
    return error;
}

static int msg_scap_test_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{    
    u16 i = 0;
    mdelay(5);
    ITO_TEST_DEBUG_MUST("ito_test_ret = %d",g_ito_test_ret);
    mdelay(5);
    for(i=0;i<48;i++)
    {
        ITO_TEST_DEBUG_MUST("data_1[%d]=%d;\n",i,s16_raw_data_1[i]);
    }
    mdelay(5);
    for(i=0;i<48;i++)
    {
        ITO_TEST_DEBUG_MUST("data_2[%d]=%d;\n",i,s16_raw_data_2[i]);
    }
    mdelay(5);
    for(i=0;i<48;i++)
    {
        ITO_TEST_DEBUG_MUST("data_3[%d]=%d;\n",i,s16_raw_data_3[i]);
    }
    mdelay(5);
    return count;
}
static int msg_scap_test_switch_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int error= 0;
    
    bItoTestDebug = 1;
    
    /*--coverity--spintf-->snprintf--*/
    error =  snprintf(buf, PAGE_SIZE, "[MSG]open debug log\n");
    
    ITO_TEST_DEBUG_MUST("on debug bItoTestDebug = %d",bItoTestDebug);

    return error;
}

static int msg_scap_test_switch_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{    
	int error = 0;
    bItoTestDebug = 0;
	//error =  sprintf(buf, "[MSG]close debug log\n");
    ITO_TEST_DEBUG_MUST("off debug bItoTestDebug = %d",bItoTestDebug);
    return error;
}
static int msg_scap_test_sample_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int error= 0;
    char *data=NULL;
    char tmp_buf[20];
    int length = 0;
    u16 i = 0;
    unsigned int data_len = 4096;

    data = (char *)kmalloc(data_len, GFP_ATOMIC);
	if (NULL == data)
	{
		return -ENOMEM;
	}
    mdelay(5);
    ITO_TEST_DEBUG_MUST("ito_test_ret = %d",g_ito_test_ret);
	memset(data, 0, data_len);
	memset(tmp_buf, 0, sizeof(tmp_buf));
    //mdelay(5);
    for(i=0;i<48;i++)
    {
        ITO_TEST_DEBUG_MUST("data_1[%d]=%d;\n",i,s16_raw_data_1[i]);

    /*--coverity--spintf-->snprintf--*/
    /*--coverity--strcat-->snstrcat--*/
        length = snprintf(tmp_buf, sizeof(tmp_buf), "data_1[%d]=%d;\n",i,s16_raw_data_1[i]);
        strncat(data, tmp_buf, sizeof(tmp_buf));
    }
    mdelay(5);
    for(i=0;i<48;i++)
    {
        ITO_TEST_DEBUG_MUST("data_2[%d]=%d;\n",i,s16_raw_data_2[i]);
        length = snprintf(tmp_buf, sizeof(tmp_buf), "data_2[%d]=%d;\n",i,s16_raw_data_2[i]);      
        strncat(data, tmp_buf, sizeof(tmp_buf));
    }
    mdelay(5);
    for(i=0;i<48;i++)
    {
        ITO_TEST_DEBUG_MUST("data_3[%d]=%d;\n",i,s16_raw_data_3[i]);
        length = snprintf(tmp_buf, sizeof(tmp_buf), "data_3[%d]=%d;\n",i,s16_raw_data_3[i]);  
        strncat(data, tmp_buf, sizeof(tmp_buf));
    }
    mdelay(5);
    error = snprintf(buf, PAGE_SIZE, data);
    kfree(data);
    
    return error;
}
static int msg_scap_test_sample_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{    
    return -1;
}
static DEVICE_ATTR(scap_test, S_IRUGO|S_IWUSR, msg_scap_test_show, msg_scap_test_store);
static DEVICE_ATTR(scap_test_sample,S_IRUGO|S_IWUSR, msg_scap_test_sample_show, msg_scap_test_sample_store);
static DEVICE_ATTR(scap_test_log_switch, S_IRUGO|S_IWUSR, msg_scap_test_switch_show, msg_scap_test_switch_store);


static struct attribute *msg_attributes[] = {
	&dev_attr_scap_test.attr,
	&dev_attr_scap_test_sample.attr,
	&dev_attr_scap_test_log_switch.attr,
	NULL
};

static struct attribute_group msg_attribute_group = {
	.attrs = msg_attributes
};
static struct kobject *touch_screen_kobject_ts = NULL;
int ito_test_create_entry(void)
{
	int error = 0;

    //printk("%s %d:enterd!\n", __func__, __LINE__);
	if( NULL == touch_screen_kobject_ts )
	{
		touch_screen_kobject_ts = kobject_create_and_add("touch_screen", NULL);
		if (!touch_screen_kobject_ts)
		{
			error=-EIO;
			return error;
		}
	}
	
	error = sysfs_create_group(touch_screen_kobject_ts, &msg_attribute_group);
	if (0 != error) {
		
		sysfs_remove_group(touch_screen_kobject_ts, &msg_attribute_group);
		error=-EIO;
		return error;
	}

    mutex_init(&scap_test_mutex);

	return error;   
}


void msg_release_sysfs(struct i2c_client *client)
{
	sysfs_remove_group(touch_screen_kobject_ts, &msg_attribute_group);
	touch_screen_kobject_ts = NULL;	
}
