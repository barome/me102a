//==============================================================//
//			Include Files				//
//==============================================================//
#include <linux/unistd.h>
#include "NVT_Config.h"


//==============================================================//
//			Declaration Functions			//
//==============================================================//
extern void NVT_Touch_Main(unsigned int X_max, unsigned int Y_max, unsigned char Boundary_Type);




//==============================================================//
//			Constant Definitions			//
//==============================================================//

#define Boundary_I  	0
#define Boundary_II	    1

#define AF_Filter_1	    0
#define AF_Filter_2	    0
#define Self_Mode	    0
#define Self_Eng_Mode	0
	

//==============================================================//
//			Extern Gloubal Variables		//
//==============================================================//
struct PointState {
    int points;
    unsigned char Status[20];
    unsigned char ID[20];
    int PosX[20];
    int PosY[20];
    int Area[20];
    int Pressure[20];
};

struct TouchConfig {
    unsigned char X_Num;
    unsigned char Y_Num;

};

extern struct TouchConfig NVT_Config;

extern struct PointState ReportPointState;
extern struct PointState Boundary_ReportPointState;
extern struct PointState preReportPointState;
extern struct PointState mPointState;
extern struct PointState prePointState;
extern struct PointState IIRPointState;
extern struct PointState preIIRPointState;


extern int X_Num, Y_Num;

 				
extern unsigned char ID_Table[20];

extern unsigned char Enter_Debounce[20];
extern unsigned char Break_Debounce[20];
extern unsigned char ID_Valid[20];
extern unsigned char ID_Valid_pre[20];
extern unsigned char Large_Move_Cnt[20];
extern unsigned char FinDown_TH[20];
extern unsigned char NoTouchCnt;

//Tracking added by TY Chan 2013.07.16 
extern unsigned long int maxTrackDis;
extern unsigned int Velocity[20];
extern unsigned char ReUseIDCnt[20];
extern unsigned char trackedIDList[20];
extern unsigned char trackedNum;
//Report Buffer, added by TY Chan 2013.07.29
extern unsigned int Report_Buffer_X[20][10];
extern unsigned int Report_Buffer_Y[20][10];

//Boundary Cheat, added by TY Chan 2013.08.07
extern unsigned int var_temp1;
extern unsigned int var_temp2;
extern unsigned int var_temp3;
extern unsigned int var_temp4;
extern unsigned char direction_X[20];
extern unsigned char direction_Y[20];
extern unsigned char sameDirCnt_X[20];
extern unsigned char sameDirCnt_Y[20];
extern signed int lastVector_X[20];
extern signed int lastVector_Y[20];

//FIR, added by TY Chan 2013.08.08
extern unsigned int FIR_X[20][10];
extern unsigned int FIR_Y[20][10];
extern unsigned char FIR_point[20];
extern unsigned char FIR_weight[20];
extern unsigned int FIRPointPosX[20];
extern unsigned int FIRPointPosY[20];

//Median Filter
extern unsigned short Median_X[20][8];
extern unsigned short Median_Y[20][8];
extern unsigned int MedianPointPosX[20];
extern unsigned int MedianPointPosY[20];

//FW update parameter
extern unsigned char FinUP_Update;


//Parameter from FW
extern unsigned char FW_PWR_State;					//[61][2:1]	
extern unsigned char FW_IIR_Distance_L;			//[62][7:0]
extern unsigned char FW_IIR_Weight;				//[63][3:0]
extern unsigned char FW_IIR_Start;					//[63][7:4]
extern unsigned char FW_FinUP_TH;					//[64][2:0]
extern unsigned char FW_TrackThreshold_H;			//[64][7:3]
extern unsigned char FW_TrackThreshold_L;			//[65][7:0]
extern unsigned char FW_Tapping_Ratio;				//[66][5:0]
extern unsigned char FW_Tapping_Dis_H;				//[66][7:6]
extern unsigned char FW_Tapping_Dis_L;				//[67][7:0]	
extern unsigned char FW_Boundary_DY_TH;			//[68][7:0]
extern unsigned char FW_Boundary_DisX_TH;			//[69][7:0]
extern unsigned char FW_Same_Dir_Cnt_X;			//[70][2:0]
extern unsigned char FW_Boundary_Range_H;			//[70][7:3]
extern unsigned char FW_Boundary_Range_L;			//[71][7:0]
extern unsigned char FW_FIR_EN;                    //[61][3]
extern unsigned char FW_Min_Fir_Cnt;               //[72][3:0]
extern unsigned char FW_Max_Fir_Cnt;               //[72][7:4]
extern unsigned char FW_Fir_Step_L;                //[73][7:0]
extern unsigned char FW_IIR_Distance_H;            //[74][3:0]
extern unsigned char FW_Fir_Step_H;                //[74][7:4]
extern unsigned char FW_Median_Order;              //[61][6:4]
//extern unsigned char FW_EnterDebounce[20] = {Drp_Frm_NC_ID1,Drp_Frm_NC_ID2,Drp_Frm_NC_ID3,Drp_Frm_NC_ID4,Drp_Frm_NC_ID5,Drp_Frm_NC_ID6,Drp_Frm_NC_ID7,Drp_Frm_NC_ID8,Drp_Frm_NC_ID9,Drp_Frm_NC_ID10,Drp_Frm_NC_ID11,Drp_Frm_NC_ID12,Drp_Frm_NC_ID13,Drp_Frm_NC_ID14,Drp_Frm_NC_ID15,Drp_Frm_NC_ID16,Drp_Frm_NC_ID17,Drp_Frm_NC_ID18,Drp_Frm_NC_ID19,Drp_Frm_NC_ID20};
extern unsigned char FW_Tracking_Anti_Cross_En;    //[61][7]
extern unsigned char FW_X_Jitter_Offset;           //[80][3:0]
extern unsigned char FW_X_Jitter_Maximun;          //[80][7:4]
extern unsigned char FW_Y_Jitter_Offset;           //[81][3:0]
extern unsigned char FW_Y_Jitter_Maximun;          //[81][7:4]
extern unsigned char FW_CrossFinNum;          		//[82][3:0]


//Parameter defualt value (overwrite by FW)
#define ENABLE                      1
#define DISABLE                     0
#define TRACK_TH_L					0xA0
#define TRACK_TH_H					0x0F	//4000
#define FIN_DOWN_TH					2
#define FIN_UP_TH               	3
#define TAPPING_DIS_L				0
#define TAPPING_DIS_H				0x01	//256
#define TAPPING_RATIO           	24
#define MAX_ENTER_DB            	30
#define LARGE_MOVE_CNT_TH       	4
#define LARGE_MOVE_MAX          	20  
#define TAPPING_LARGE_MOVE_TH   	150
#define REPORT_BUFFER_NUM       	FIN_UP_TH
#define	NO_TOUCH_CNT_MAX			30
#define BOUNDARY_CHEAT_DX_TH        100
#define BOUNDARY_CHEAT_DY_TH        100
#define BOUNDARY_CHEAT_DIS_X_TH     30
#define BOUNDARY_CHEAT_DIS_Y_TH     30
#define SAME_DIR_CNT_X_TH           5
#define SAME_DIR_CNT_Y_TH           5
#define BOUNDARY_CHEAT_RANGE_H		0x01		
#define BOUNDARY_CHEAT_RANGE_L		0x2C	//300
#define MIN_FIR_CNT                 1
#define MAX_FIR_CNT                 5       //max = 10
#define FIR_STEP_L                  200
#define FIR_STEP_H                  0
#define AREA_OFFSET					10
#define PRESSURE_OFFSET				10
#define MEDIAN_ORDER                3
#define EnterDB_ID0                 2
#define EnterDB_ID1                 2
#define EnterDB_ID2                 2
#define EnterDB_ID3                 3
#define EnterDB_ID4                 3
#define EnterDB_ID5                 3
#define EnterDB_ID6                 3
#define EnterDB_ID7                 3
#define EnterDB_ID8                 3
#define EnterDB_ID9                 3
#define ID_REUSE_CNT                0
#define	MAX_CROSS_FIN_NUM			5



