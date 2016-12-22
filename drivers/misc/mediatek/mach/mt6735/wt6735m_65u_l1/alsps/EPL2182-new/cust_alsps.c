#include <linux/types.h>
#include <mach/mt_pm_ldo.h>
#include <cust_alsps.h>
#include <mach/upmu_common.h>

static struct alsps_hw cust_alsps_hw = {
  .i2c_num    = 2,
	//.i2c_addr = {0x92,0,0,0},
	.polling_mode_ps =0,
	.polling_mode_als =1,
    .power_id   = MT65XX_POWER_NONE,    /*LDO is not used*/
    .power_vol  = VOL_DEFAULT,          /*LDO is not used*/
   #ifdef WT_ALS_SWITCH_AUTO
	.als_level_high	= { 4, 100, 200, 300, 400, 500, 600, 700, 800, 1200,  3000,  5000, 10000, 30000, 50000},
	.als_level_low	= { 1, 100, 200, 300, 400, 500, 600, 700, 800, 1000,  3000,  5000, 10000, 30000, 50000},
    #endif
	.als_level	= { 1, 100, 200, 300, 400, 500, 600, 700, 800, 1200,  3000,  5000, 10000, 30000, 50000},
	.als_value	= { 0, 320, 320, 320, 320, 320, 320, 320, 320,  320, 10240, 10240, 10240, 10240, 10240, 10240},
	.ps_threshold_high = 1500,
	.ps_threshold_low = 600,
};
struct alsps_hw *get_cust_alsps_hw(void) {
    return &cust_alsps_hw;
}



