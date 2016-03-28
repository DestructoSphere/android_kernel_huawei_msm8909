#include <linux/config_interface.h>
#include <misc/app_info.h>

const void* dt_get_property_for_fac (char* key, int* pvalue_len)
{
    struct device_node *dp = NULL;
    
    if(key == NULL)
    {
        printk(KERN_ERR "param is NULL!\n");
        return NULL;
    }
    dp = of_find_node_by_path("/huawei_fac_info");
    if(!of_device_is_available(dp))
    {
       printk(KERN_ERR "device is not available!\n");
       return NULL;
    }

   return  of_get_property(dp, key, pvalue_len);
}


int get_product_name(char* product_name, int name_len)
{
   int product_name_len = 0;
   const char* temp = NULL;
   if(product_name == NULL)
   {
       printk(KERN_ERR " param is NULL\n");
       return DT_PROP_ERROR;
   }
   memset(product_name,0,name_len);
   temp = dt_get_property_for_fac("fac,product_name",&product_name_len);
   if(temp == NULL)
   {
       printk(KERN_ERR " get product name fail!\n");
       return DT_PROP_ERROR;
   }
   memcpy(product_name, temp, product_name_len);
   return DT_PROP_SUUCESS;
}


int get_hardware_ver(char* hardware_ver, int name_len)
{

   int hardware_ver_len = 0;
   const char* temp = NULL;
   if(hardware_ver == NULL)
   {
       printk(KERN_ERR " param is NULL\n");
       return DT_PROP_ERROR;
   }
   memset(hardware_ver,0,name_len);
   temp = dt_get_property_for_fac("fac,hardware_ver",&hardware_ver_len);
   if(temp == NULL)
   {
       printk(KERN_ERR " get hardware version fail!\n");
       return DT_PROP_ERROR;
   }
   memcpy(hardware_ver, temp, hardware_ver_len);
   return DT_PROP_SUUCESS;
}



int get_software_ver(char* software_ver, int name_len)
{

   int software_ver_len = 0;
   const char* temp = NULL;
   if(software_ver == NULL)
   {
       printk(KERN_ERR " param is NULL\n");
       return DT_PROP_ERROR;
   }
   memset(software_ver,0,name_len);
   temp = dt_get_property_for_fac("fac,software_ver",&software_ver_len);
   if(temp == NULL)
   {
       printk(KERN_ERR " get software version fail!\n");
       return DT_PROP_ERROR;
   }
   memcpy(software_ver, temp, software_ver_len);
   return DT_PROP_SUUCESS;
}
int set_sbl1_ver_to_appinfo()
{

   int sbl1_ver_len = 0;
   const char* sbl_ver = NULL;

   sbl_ver = dt_get_property_for_fac("fac,sbl1_ver",&sbl1_ver_len);
   if(sbl_ver == NULL)
   {
       printk(KERN_ERR " get sbl1 version fail!\n");
       return DT_PROP_ERROR;
   }  
   if( -1 == app_info_set("huawei_sbl1_version", sbl_ver))
   {
       printk(KERN_ERR " set sbl1 version to app info fail!\n");
       return DT_PROP_ERROR;
   }
   return DT_PROP_SUUCESS;
}

int set_hardware_ver_to_appinfo()
{

   int hardware_ver_len = 0;
   const char* hardware_ver = NULL;

   hardware_ver = dt_get_property_for_fac("fac,hardware_ver",&hardware_ver_len);
   if(hardware_ver == NULL)
   {
       printk(KERN_ERR " get hardware version fail!\n");
       return DT_PROP_ERROR;
   }  
   if( -1 == app_info_set("huawei_hardware_version", hardware_ver))
   {
       printk(KERN_ERR " set hardware version to app info fail!\n");
       return DT_PROP_ERROR;
   }
   return DT_PROP_SUUCESS;
}

int set_appboot_ver_to_appinfo()
{

   int appboot_ver_len = 0;
   const char* appboot_ver = NULL;

   appboot_ver = dt_get_property_for_fac("fac,appboot_ver",&appboot_ver_len);
   if(appboot_ver == NULL)
   {
       printk(KERN_ERR " get appboot version fail!\n");
       return DT_PROP_ERROR;
   }  
   if( -1 == app_info_set("huawei_appboot_version", appboot_ver))
   {
       printk(KERN_ERR " set appboot version to app info fail!\n");
       return DT_PROP_ERROR;
   }
   return DT_PROP_SUUCESS;
}

int set_product_name_to_appinfo()
{

   int product_name_len = 0;
   const char* product_name = NULL;

   product_name = dt_get_property_for_fac("fac,product_name",&product_name_len);
   if(product_name == NULL)
   {
       printk(KERN_ERR " get product name fail!\n");
       return DT_PROP_ERROR;
   }  
   if( -1 == app_info_set("huawei_fac_product_name", product_name))
   {
       printk(KERN_ERR " set product name to app info fail!\n");
       return DT_PROP_ERROR;
   }
   return DT_PROP_SUUCESS;
}
int set_software_ver_to_appinfo(void)
{

   int software_ver_len = 0;
   const char* software_ver = NULL;

   software_ver = dt_get_property_for_fac("fac,software_ver",&software_ver_len);
   if(software_ver == NULL)
   {
       printk(KERN_ERR " get software version fail!\n");
       return DT_PROP_ERROR;
   }
   if( -1 == app_info_set("board_id", software_ver))
   {
       printk(KERN_ERR " set software ver to app info fail!\n");
       return DT_PROP_ERROR;
   }
   return DT_PROP_SUUCESS;
}
