#ifndef _CCCONFIG_H
#define _CCCONFIG_H

#include <c_types.h>

#define HPABUFFSIZE 512

#define CCEMBEDDED
#define NUM_LIN_LEDS 255
#define DFREQ 16000

#define memcpy ets_memcpy
#define memset ets_memset

#define ROOT_NOTE_OFFSET	CCS.gROOT_NOTE_OFFSET
#define DFTIIR				CCS.gDFTIIR
#define FUZZ_IIR_BITS  		CCS.gFUZZ_IIR_BITS
#define MAXNOTES  12 //MAXNOTES cannot be changed dynamically.
#define FILTER_BLUR_PASSES	CCS.gFILTER_BLUR_PASSES
#define SEMIBITSPERBIN		CCS.gSEMIBITSPERBIN
#define MAX_JUMP_DISTANCE	CCS.gMAX_JUMP_DISTANCE
#define MAX_COMBINE_DISTANCE CCS.gMAX_COMBINE_DISTANCE
#define AMP_1_IIR_BITS		CCS.gAMP_1_IIR_BITS
#define AMP_2_IIR_BITS		CCS.gAMP_2_IIR_BITS
#define MIN_AMP_FOR_NOTE	CCS.gMIN_AMP_FOR_NOTE
#define MINIMUM_AMP_FOR_NOTE_TO_DISAPPEAR CCS.gMINIMUM_AMP_FOR_NOTE_TO_DISAPPEAR
#define NOTE_FINAL_AMP		CCS.gNOTE_FINAL_AMP
#define NERF_NOTE_PORP		CCS.gNERF_NOTE_PORP
#define USE_NUM_LIN_LEDS	CCS.gUSE_NUM_LIN_LEDS
#define COLORCHORD_OUTPUT_DRIVER	CCS.gCOLORCHORD_OUTPUT_DRIVER
#define COLORCHORD_ACTIVE	CCS.gCOLORCHORD_ACTIVE
#define INITIAL_AMP	CCS.gINITIAL_AMP

//We are not enabling these for the ESP8266 port.
#define LIN_WRAPAROUND 0 
#define SORT_NOTES 0


struct CCSettings
{
	uint8_t gSETTINGS_KEY;
	uint8_t gROOT_NOTE_OFFSET; //Set to define what the root note is.  0 = A.
	uint8_t gDFTIIR;                            //=6
	uint8_t gFUZZ_IIR_BITS;                     //=1
	uint8_t gFILTER_BLUR_PASSES;                //=2
	uint8_t gSEMIBITSPERBIN;                    //=3
	uint8_t gMAX_JUMP_DISTANCE;                 //=4
	uint8_t gMAX_COMBINE_DISTANCE;              //=7
	uint8_t gAMP_1_IIR_BITS;                    //=4
	uint8_t gAMP_2_IIR_BITS;                    //=2
	uint8_t gMIN_AMP_FOR_NOTE;                  //=80
	uint8_t gMINIMUM_AMP_FOR_NOTE_TO_DISAPPEAR; //=64
	uint8_t gNOTE_FINAL_AMP;                    //=12
	uint8_t gNERF_NOTE_PORP;                    //=15
	uint8_t gUSE_NUM_LIN_LEDS;                  // = NUM_LIN_LEDS
	uint8_t gCOLORCHORD_ACTIVE;
	uint8_t gCOLORCHORD_OUTPUT_DRIVER;
	uint8_t gINITIAL_AMP;
};

extern struct CCSettings CCS;




#endif

