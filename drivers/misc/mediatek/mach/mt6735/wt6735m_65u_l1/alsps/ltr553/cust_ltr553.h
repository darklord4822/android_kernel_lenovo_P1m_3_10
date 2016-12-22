

#ifndef __CUST_LTR553_H__
#define __CUST_LTR553_H__


/*******************************************/

/* define setting value */
#define ALS_AVR_COUNT 		  5		// als do average N times
#define ALS_COUNT_FOR_SMOOTH  32	// als switch value after N times
#define CUST_ALS_SKIP_COUNT   5		// als skip N count after enable 

/*******************************************/

//#define WT_DO_ALS_SMOOTH	// enable als mooth 
#define WT_ALS_SWITCH_AUTO	// enable als auto switch 
#define WT_ALS_ENABLE_SKIP  // enable als skip first N count

/*******************************************/

/* For ps calibration */
#define CUST_LTR553_PS_THRES_CLOSE		90	 // Close 
#define CUST_LTR553_PS_THRES_FAR		40	 // Far away 

/*LTR553 ps cust setting*/
#define CUST_LTR553_PS_CONTR_ENABLE		0x03 // PS operation mode control		//PS Gain X16
#define CUST_LTR553_PS_LED				0x7F // PS LED setting					//100mA, 100%, 60kHz 
#define CUST_LTR553_PS_N_PULSES			0x04 // PS number of pulses				//Number of pulses = 4 
#define CUST_LTR553_PS_MEAS_RATE		0x02 // PS measurement rate in active mode	 //100 ms	   
#define CUST_LTR553_INTERRUPT_PERSIST	0x00 // ALS / PS Interrupt persist setting	 //every 	


/*******************************************/

/*LTR553 als cust setting*/
#define CUST_LTR553_ALS_CONTR_ENABLE	0x05 // ALS operation mode control SW reset	 //Gain 2X -> 0.5lux to 32k lux
#define CUST_LTR553_ALS_MEAS_RATE		0x03 // ALS measurement rate in active mode  //rate=500ms, time=100ms 

#endif
