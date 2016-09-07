/*
 * tef6657.c NXP TEF6657 Car Radio Enhanced Selectivity Tuner
 * Copyright (c) 2014 Intel Corporation
 *
 *I2C device address 0xC8>>1
 */
 
#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/device.h>
#include <linux/sysfs.h>


#include "Tef665x.h"

#define DRIVER_NAME "tef6657"


struct tef6657_t
{
	struct i2c_client *tef6657_client;
	struct mutex mu;
	void (*PowerSwitch)(struct tef6657_t* nxp_dev,bool OnOff);
	void (*ModeSwitch)(struct tef6657_t * nxp_dev,bool mode_switch,AR_TuningAction_t mode,u16 frequency);
	//int (*Radio_Signal)(struct tef6657_t* nxp_dev,int16_t *ret,u8 data);
} ;

static struct tef6657_t *tef6657_dev;

enum {
	TEF6657_CMD_tune = 0,
	TEF6657_CMD_open = 1,
	TEF6657_CMD_close = 2,
	TEF6657_CMD_am = 3,
	TEF6657_CMD_fm = 4,
	TEF6657_CMD_GETamSTAUS  = 5,
	TEF6657_CMD_GETfmSTAUS  = 6,
	TEF6657_CMD_GEToirtSTAUS
};

enum
{
	RADIO_BOOT_STATE = 0,
	RADIO_IDLE_STATE,
	RADIO_STANBDY_STATE,
	RADIO_FM_STATE,
	RADIO_AM_STATE
	
};


typedef struct
{
    uint16_t Status;
    int16_t Level;
    int16_t FreqOffset;
    uint8_t IFBandwidth;
    uint16_t Modulation;
} AM_QData_t;

typedef struct
{
    uint16_t Status;
    int16_t Level;
    uint16_t UltraSonicNoise;
    uint16_t WidebandAMMultipath;
    int16_t FreqOffset;
    uint8_t IFBandwidth;
    uint16_t Modulation;
} FM_QData_t;

typedef struct
{
    bool Stereo;
    bool DigSignal;
} SignalSts_t;

typedef enum
{
	BAND_OIRT,
	BAND_FM,
	BAND_AM
} RADIO_BAND; 

typedef enum
{
	AREA_EUROPE,
	AREA_EAST_EUROPE,
	AREA_USA,
	AREA_ASIA,
	AREA_JAPAN
} RADIO_AREA; 

typedef struct
{
	RADIO_BAND		Band;
	uint32_t		FreqFM;
	uint32_t		FreqAM;
	uint32_t		FreqOIRT;
	RADIO_AREA		Area;
	uint8_t			ScanState;
	uint8_t			FM_SignalLevel;
	uint8_t			AM_SignalLevel;
	uint16_t		Timer;
}RADIO_T;

RADIO_T Radio_t;

static const unsigned char devTEF665x_Patch_CmdTab1[] = {3,	0x1c,0x00,0x00};
static const unsigned char devTEF665x_Patch_CmdTab2[] = {3,	0x1c,0x00,0x74};
static const unsigned char devTEF665x_Patch_CmdTab3[] = {3,	0x1c,0x00,0x75};

#define High_16bto8b(a)	((uint8_t)((a) >> 8))
#define Low_16bto8b(a) 	((uint8_t)(a )) 
#define Convert8bto16b(a)	((uint16_t)(((uint16_t)(*(a))) << 8 |((uint16_t)(*(a+1)))))

int APPL_Get_Operation_Status(struct i2c_client *client,TEF665x_STATE *status);
static int devTEF665x_Write(struct i2c_client *client,unsigned char * buf,u8 len);
int devTEF665x_APPL_Get_Operation_Status(struct i2c_client *client,u8 *status);
static int devTEF665x_Read(struct i2c_client *client, u8 reg,unsigned char * buf,u32 len);
int devTEF665x_APPL_Set_ReferenceClock(struct i2c_client *client,u16 frequency_high,u16 frequency_low,u16 type);
int devTEF665x_Patch_Init(struct i2c_client *client);
static int devTEF665x_WriteTab(struct i2c_client *client,const unsigned char * tab);

int devTEF665x_Set_Cmd(struct i2c_client *client,TEF665x_MODULE module, u8 cmd, int len,...)
{
	int i;
	u8 buf[TEF665x_CMD_LEN_MAX];
	u16 temp;
    va_list     vArgs;

    va_start(vArgs, len);
		
	buf[0]= module;			//module,		FM/AM/APP
	buf[1]= cmd;		//cmd,		1,2,10,... 
	buf[2]= 0x01;	//index, 		always 1

//fill buffer with 16bits one by one
	for(i=3;i<len;i++)
	{
		temp = va_arg(vArgs,int);	//the size only int valid for compile
		
		buf[i++]=High_16bto8b(temp);		
		buf[i]=Low_16bto8b(temp);		
	}
	
	va_end(vArgs); 
	
	return devTEF665x_Write(client,buf,len);
}

static int devTEF665x_Get_Cmd(struct i2c_client *client,TEF665x_MODULE module, u8 cmd, u8 *receive,int len)
{
	u8 buf[3];

	buf[0]= module;			//module,		FM/AM/APP
	buf[1]= cmd;		//cmd,		1,2,10,... 
	buf[2]= 1;	//index, 		always 1

	//printk( "devTEF665x_Get_Cmd buf[0]=%d  buf[1]=%d buf[2]=%d\n",buf[0],buf[1],buf[2]);

	devTEF665x_Write(client,buf, 3);

	return devTEF665x_Read(client,buf[0],receive,len);
}

/*
module 64 APPL
cmd 128 Get_Operation_Status | status
index 
1 status
	Device operation status
	0 = boot state; no command support
	1 = idle state
	2 = active state; radio standby
	3 = active state; FM
	4 = active state; AM
*/
int devTEF665x_APPL_Get_Operation_Status(struct i2c_client *client,u8 *status)
{
	u8 buf[2];
	int ret;
	
	ret = devTEF665x_Get_Cmd(client,TEF665X_MODULE_APPL,
			TEF665X_Cmd_Get_Operation_Status,
			buf,sizeof(buf));

	if(ret == SET_SUCCESS)
	{
		printk( "Status buf[0]=%d  buf[1]=%d\n",buf[0],buf[1]);
		*status = Convert8bto16b(buf);
	}

	return ret;
}

int devTEF665x_APPL_Get_GPIO_Status(struct i2c_client *client)
{
	u8 buf[2];
	int ret;

	u16 status;
	
	ret = devTEF665x_Get_Cmd(client,TEF665X_MODULE_APPL,
			TEF665X_Cmd_Get_GPIO_Status,
			buf,sizeof(buf));

	if(ret == SET_SUCCESS){
		status = Convert8bto16b(buf);
		
		printk("GPIO_Status status=%d \n",status);
	}

	return ret;
}


int devTEF665x_APPL_Get_Identification(struct i2c_client *client)
{
	u8 buf[6];
	int ret;

	u16 device;
	u16 hw_version;
	u16 sw_version;
	
	ret = devTEF665x_Get_Cmd(client,TEF665X_MODULE_APPL,
			TEF665X_Cmd_Get_Identification,
			buf,sizeof(buf));

	if(ret == SET_SUCCESS){
		device = Convert8bto16b(buf);
		hw_version = Convert8bto16b(buf+2);
		sw_version = Convert8bto16b(buf+4);

		printk("Identification device=%d hw_version=%d sw_version=%d\n",device,hw_version,sw_version);
	}

	return ret;
}


static int DevTEF665x_Power_on(struct i2c_client *client)
{
	int ret;

	TEF665x_STATE status;
	
	//mdelay(5);
	
	if(SET_SUCCESS == (ret=APPL_Get_Operation_Status(client,&status)))   //[ w 40 80 01 [ r 0000 ]
	{

		printk( "DevTEF665x_Power_on \n");
	}
	
	return ret;
}

//Command start will bring the device into? idle state’: [ w 14 0001 ]
int devTEF665x_StartCmd(struct i2c_client *client)
{

	int ret;
	unsigned char  buf[3];
	
	buf[0] = 0x14;
	buf[1] = 0;
	buf[2] = 1;

	ret = i2c_master_send(client,buf,3);

	printk( "devTEF665x_StartCmd =%d \n",ret);
	
	return ret;
}

/*
module 64 APPL
cmd 4 Set_ReferenceClock frequency

index 
1 frequency_high
	[ 15:0 ]
	MSB part of the reference clock frequency
	[ 31:16 ]
2 frequency_low
	[ 15:0 ]
	LSB part of the reference clock frequency
	[ 15:0 ]
	frequency [*1 Hz] (default = 9216000)
3 type
	[ 15:0 ]
	clock type
	0 = crystal oscillator operation (default)
	1 = external clock input operation
*/
int devTEF665x_APPL_Set_ReferenceClock(struct i2c_client *client,u16 frequency_high,u16 frequency_low,u16 type)
{
	return devTEF665x_Set_Cmd(client,TEF665X_MODULE_APPL,
			TEF665X_Cmd_Set_ReferenceClock, 
			9,
			frequency_high,frequency_low,type);
}

int APPL_Set_ReferenceClock(struct i2c_client *client,u32 frequency, bool is_ext_clk)  //0x3d 0x900
{
	int ret;
	ret = devTEF665x_APPL_Set_ReferenceClock(client,(u16)(frequency >> 16), (u16)frequency, is_ext_clk);
	
	return ret;
}

/*
module 64 APPL
cmd 5 Activate mode

index 
1 mode
	[ 15:0 ]
	1 = goto ‘active’ state with operation mode of ‘radio standby’
*/
int devTEF665x_APPL_Activate(struct i2c_client *client,u16 mode)
{
	return devTEF665x_Set_Cmd(client,TEF665X_MODULE_APPL,
			TEF665X_Cmd_Activate, 
			5,
			mode);
}

int APPL_Activate(struct i2c_client *client)
{
	return devTEF665x_APPL_Activate(client,1);
}

int APPL_Get_Operation_Status(struct i2c_client *client,TEF665x_STATE *status)
{
	u8 data;
	int ret;

	if(SET_SUCCESS ==(ret = devTEF665x_APPL_Get_Operation_Status(client,&data)))
	{
		printk( "APPL_Get_Operation_Status1 data= %d \n",data);
		switch(data)
		{
			case 0:
				printk( "status = eDevTEF665x_Boot_state \n");
				*status = eDevTEF665x_Boot_state;
				break;
			case 1:
				printk( "status = eDevTEF665x_Idle_state \n");
				*status = eDevTEF665x_Idle_state;
				break;
			default:
				printk( "status = eDevTEF665x_Active_state \n");
				*status = eDevTEF665x_Active_state;
				break;
		}
	}
	
	return ret;
}

int devTEF665x_Para_Load(struct i2c_client *client)
{
	int i;
	int r;
	const u8 *p = init_para;

	for(i=0;i<sizeof(init_para);i+=(p[i]+1))
	{
		if(SET_SUCCESS != (r=devTEF665x_WriteTab(client,p+i)))
			break;
	}

	return r;
}

/*
module 32 / 33 FM / AM
cmd 1 Tune_To mode, frequency

index 
1 mode
	[ 15:0 ]
	tuning actions
	0 = no action (radio mode does not change as function of module band)
	1 = Preset Tune to new program with short mute time
	2 = Search Tune to new program and stay muted
	FM 3 = AF-Update Tune to alternative frequency, store quality
	and tune back with inaudible mute
	4 = Jump Tune to alternative frequency with short
	inaudible mute
	5 = Check Tune to alternative frequency and stay
	muted
	AM 3 … 5 = reserved
	6 = reserved
	7 = End Release the mute of a Search or Check action
	(frequency is not required and ignored)
2 frequency
[ 15:0 ]
	tuning frequency
	FM 6500 … 10800 65.00 … 108.00 MHz / 10 kHz step size
	AM LW 144 … 288 144 … 288 kHz / 1 kHz step size
	MW 522 … 1710 522 … 1710 kHz / 1 kHz step size
	SW 2300 … 27000 2.3 … 27 MHz / 1 kHz step size
*/
int devTEF665x_Radio_Tune_To (struct i2c_client *client,bool fm,u16 mode,u16 frequency )
{
	return devTEF665x_Set_Cmd(client,fm ? TEF665X_MODULE_FM: TEF665X_MODULE_AM,
			TEF665X_Cmd_Tune_To, 
			(mode<=5)? 7 : 5,
			mode, frequency);
}

int FM_Tune_To(struct i2c_client *client,AR_TuningAction_t mode,u16 frequency)
{
	return devTEF665x_Radio_Tune_To(client,1, (u16)mode, frequency);
}

int AM_Tune_To(struct i2c_client *client,AR_TuningAction_t mode,u16 frequency)
{
	return devTEF665x_Radio_Tune_To(client,0, (u16)mode, frequency);
}

/*
module 48 AUDIO
cmd 11 Set_Mute mode

index 
1 mode
	[ 15:0 ]
	audio mute
	0 = mute disabled
	1 = mute active (default)
*/
int devTEF665x_Audio_Set_Mute(struct i2c_client *client,u16 mode)
{
	return devTEF665x_Set_Cmd(client,TEF665X_MODULE_AUDIO,
			TEF665X_Cmd_Set_Mute, 
			5,
			mode);
}

/*
module 48 AUDIO
cmd 10 Set_Volume volume

index 
1 volume
	[ 15:0 ] (signed)
	audio volume
	-599 … +240 = -60 … +24 dB volume
	0 = 0 dB (default)
*/
int devTEF665x_Audio_Set_Volume(struct i2c_client *client,int16_t volume)
{
	return devTEF665x_Set_Cmd(client,TEF665X_MODULE_AUDIO,
			TEF665X_Cmd_Set_Volume,
			5,
			volume*10);
}

//mute=1, unmute=0
int AUDIO_Set_Mute(struct i2c_client *client,bool mute)
{
	return devTEF665x_Audio_Set_Mute(client,mute);//AUDIO_Set_Mute mode = 0 : disable mute
}
//-60 … +24 dB volume
int AUDIO_Set_Volume(struct i2c_client *client,int vol)
{
	return devTEF665x_Audio_Set_Volume(client,(int16_t)vol);
}

/*
module 64 APPL
cmd 1 Set_OperationMode mode

index 
1 mode
	[ 15:0 ]
	device operation mode
	0 = normal operation
	1 = radio standby mode (low-power mode without radio functionality)
	(default)
*/
int devTEF665x_Audio_Set_OperationMode(struct i2c_client *client,u16 mode)
{
	return devTEF665x_Set_Cmd(client,TEF665X_MODULE_APPL,
			TEF665X_Cmd_Set_OperationMode, 
			5,
			mode);
}

//TRUE = ON;
//FALSE = OFF
void Radio_PowerSwitch(struct tef6657_t* nxp_dev,bool OnOff)
{
	devTEF665x_Audio_Set_OperationMode(nxp_dev->tef6657_client,OnOff? 0 : 1);//standby mode = 1
}

void Radio_ModeSwitch(struct tef6657_t* nxp_dev,bool mode_switch,AR_TuningAction_t mode,u16 frequency)
{
	if(mode_switch)	//FM
	{
		FM_Tune_To(nxp_dev->tef6657_client,mode,frequency);	
	}
	else //AM
	{
		AM_Tune_To(nxp_dev->tef6657_client,mode,frequency);
	}
}


static int DevTEF665x_Wait_Active(struct i2c_client *client)
{
	TEF665x_STATE status;

	//mdelay(50);
	printk("======DevTEF665x_Wait_Active1====\n");

	if(SET_SUCCESS == APPL_Get_Operation_Status(client,&status)){
		printk("======DevTEF665x_Wait_Active2====\n");
		if((status != eDevTEF665x_Boot_state) &&(status != eDevTEF665x_Idle_state))
		{
			printk("======DevTEF665x_Wait_Active3====\n");
			devTEF665x_Para_Load(client);

			FM_Tune_To(client,eAR_TuningAction_Preset,8980);// tune to 89.8MHz

			AUDIO_Set_Mute(client,0);//unmute
			AUDIO_Set_Volume(client,0);//set to 0db

			return 1;
		}
	}

	return 0;
}

static int DevTEF665x_Idle_state(struct i2c_client *client)
{

	TEF665x_STATE status;
	
	//mdelay(50);

	if(SET_SUCCESS == APPL_Get_Operation_Status(client,&status))
	{
		printk("APPL_Get_Operation_Status success!\n");
		if(status != eDevTEF665x_Boot_state)
		{
			printk("status != eDevTEF665x_Boot_state\n");
			//Set reference frequency
			if(SET_SUCCESS == APPL_Set_ReferenceClock(client,TEF665x_REF_CLK, TEF665x_IS_CRYSTAL_CLK)) //TEF665x_IS_EXT_CLK
			{
				printk("APPL_Set_ReferenceClock success!\n");
				if(SET_SUCCESS == APPL_Activate(client))//Activate : APPL_Activate mode = 1.[ w 40 05 01 0001 ]
				{
					//mdelay(100); //Wait 100 ms
					printk("APPL_Activate success!\n");
					return 1;
				}
			}
		}
	}

	return 0;
	
}

static int DevTEF665x_Boot_state(struct i2c_client *client)
{
	
	devTEF665x_Patch_Init(client);

	msleep(50);

	devTEF665x_StartCmd(client);
	
	msleep(50);
	
	return 0;
}

static int devTEF665x_WriteTab(struct i2c_client *client,const unsigned char * tab)
{
	unsigned char buf[32];
	int len;
	int i;
	int ret;

	len = tab[0];

	for(i=0;i<len;i++)
		buf[i] = tab[i+1];

	//printk("devTEF665x_WriteTab\n");
		
	ret = i2c_master_send(client,buf,len);
	//printk("devTEF665x_WriteTab buf[0]= %d len=%d ret=%d ,ret_test=%d\n",buf[0],len,ret,ret_test);
	if(ret < 0)
	{
		printk("sends command devTEF665x_WriteTab error!!\n");
		return 0;
	}

	
	return 1;
}

int devTEF665x_Patch_Load(struct i2c_client *client,const unsigned char * pLutBytes, int16_t size)
{
	unsigned char buf[TEF665X_SPLIT_SIZE+1];
	int16_t len;
	int ret;
	
	buf[0] = 0x1b;

	while(size)
	{
		len = (size>TEF665X_SPLIT_SIZE)? TEF665X_SPLIT_SIZE : size;
		size -= len;

		memcpy(buf+1,pLutBytes,len);
		pLutBytes+=len;

		ret = i2c_master_send(client,buf,len+1);

		//printk("devTEF665x_Patch_Load buf[0]= %d len=%d ret=%d \n",buf[0],len,ret);
		
		if(ret < 0)
		{
			printk("sends command devTEF665x_Patch_Load error!!\n");
			return 0;
		}		
	
		udelay(50); //delay for debug output
	
	}

	return 1;
}

static int devTEF665x_Read(struct i2c_client *client, u8 reg,unsigned char * buf,u32 len)
{ 
	int ret;

	ret = i2c_master_recv(client,buf,len);

	if(ret < 0)
	{
		printk("recv command error!!\n");
		return 0;
	}
	
	return 1;
}

static int devTEF665x_Write(struct i2c_client *client,unsigned char * buf,u8 len)
{
	int ret;

	ret = i2c_master_send(client,buf,len);

	if(ret < 0)
	{
		printk("sends command error!!\n");
		return 0;
	}
	
	return 1;
	
}

int devTEF665x_Patch_Init(struct i2c_client *client)
{
	int ret;
	
	ret = devTEF665x_WriteTab(client,devTEF665x_Patch_CmdTab1);  //[ w 1C 0000 ]
	if(!ret)
	{
		printk("int tab1 error !\n");
		goto tab_failed;
	}

	ret = devTEF665x_WriteTab(client,devTEF665x_Patch_CmdTab2);  //[ w 1C 0074 ]
	if(!ret)
	{
		printk("int tab2 error !\n");
		goto tab_failed;
	}

	ret= devTEF665x_Patch_Load(client,pPatchBytes,PatchSize); //table1	
	if(!ret)
	{
		printk("devTEF665x_Patch_Load failed!\n");
		goto tab_failed;
	}		
	
	ret = devTEF665x_WriteTab(client,devTEF665x_Patch_CmdTab1); //[ w 1C 0000 ]	
	if(!ret)
	{
		printk("devTEF665x_Patch_CmdTab1 failed!\n");
		goto tab_failed;
	}	

	ret = devTEF665x_WriteTab(client,devTEF665x_Patch_CmdTab3); //[ w 1C 0075 ]	
	if(!ret)
	{
		printk("devTEF665x_Patch_CmdTab3 failed!\n");
		goto tab_failed;
	}	

	ret = devTEF665x_Patch_Load(client,pLutBytes,LutSize); //table2	
	if(!ret)
	{
		printk("devTEF665x table2 failed!\n");
		goto tab_failed;
	}		

	ret = devTEF665x_WriteTab(client,devTEF665x_Patch_CmdTab1); //[ w 1C 0000 ]	
	if(!ret)
	{
		printk("devTEF665x_Patch_CmdTab1 failed!\n");
	}	
	
tab_failed:
	return ret;	
}

int devTEF665x_Radio_Get_Quality_Data (struct i2c_client *client,bool fm,u8 *status,int16_t *level,u8 *usn,u8 *wam,int16_t *offset,int16_t *bandwidth,u8 *modulation)
{
	u8 buf[14];
	int ret;
	
	ret = devTEF665x_Get_Cmd(client,fm ? TEF665X_MODULE_FM : TEF665X_MODULE_AM,
			TEF665X_Cmd_Get_Quality_Data,
			buf,sizeof(buf));

	if(ret == SET_SUCCESS)
	{
		*status = ((0x3fff&Convert8bto16b(buf))/10);
		*level = (((int)Convert8bto16b(buf+2))/10);
		*usn = (Convert8bto16b(buf+4)/10);
		*wam = (Convert8bto16b(buf+6)/10);
		*offset = (((int)Convert8bto16b(buf+8))/1);
		*bandwidth = (Convert8bto16b(buf+10)/1);
		*modulation = (Convert8bto16b(buf+12)/10);
	}

	return ret;
}


int AM_Get_Quality_Data (struct i2c_client *client,AM_QData_t *pAMQ )
{
    int ret;
	s16 level,offset,bandwidth;
	u8 	status,modulation,wam,usn;
    ret = devTEF665x_Radio_Get_Quality_Data(client,0,&status,&level,&usn,&wam,&offset,&bandwidth,&modulation);
	if(ret==SET_SUCCESS)
	{
      pAMQ->FreqOffset=offset;
	  pAMQ->IFBandwidth=bandwidth;
	  pAMQ->Level=level;
	  pAMQ->Status=status;

	}
	return ret;
}
int FM_Get_Quality_Data (struct i2c_client *client,FM_QData_t *pFMQ )
{
    int ret;
	s16 level,offset,bandwidth;
	u8 	status,modulation,wam,usn;
    ret = devTEF665x_Radio_Get_Quality_Data(client,1,&status,&level,&usn,&wam,&offset,&bandwidth,&modulation);

	if(ret==SET_SUCCESS)
	{
      pFMQ->FreqOffset=offset;
	  pFMQ->IFBandwidth=bandwidth;
	  pFMQ->Level=level;
	  pFMQ->Status=status;
	  pFMQ->Modulation=modulation;
	  pFMQ->UltraSonicNoise=usn;
	  pFMQ->WidebandAMMultipath=wam;
	}

	return ret;
}

int devTEF665x_Radio_Get_Quality_Level (struct i2c_client *client,bool fm,u8 *status,int16_t *level)
{
	u8 buf[4];
	int ret;
	
	ret = devTEF665x_Get_Cmd(client,fm ? TEF665X_MODULE_FM : TEF665X_MODULE_AM,
			TEF665X_Cmd_Get_Quality_Data,
			buf,sizeof(buf));

	if(ret == SET_SUCCESS)
	{
		*status = (int16_t)((0x3fff&Convert8bto16b(buf))/10);
		*level = (int16_t)(((int)Convert8bto16b(buf+2))/10);
	}

	return ret;
}

int Get_Level_Data (struct i2c_client *client,bool fm,s16 *pLev )
{
    int ret;
	u8 status;
	
	ret = devTEF665x_Radio_Get_Quality_Level(client,fm,&status,pLev);
	
	return ret;		
}

int devTEF665x_Radio_Get_Quality_Status (struct i2c_client *client,bool fm,u8 *status)
{
	u8 buf[2];
	int ret;
	
	ret = devTEF665x_Get_Cmd(client,fm ? TEF665X_MODULE_FM : TEF665X_MODULE_AM,
			TEF665X_Cmd_Get_Operation_Status,
			buf,sizeof(buf));

	if(ret == SET_SUCCESS){
		*status = ((0x3fff&Convert8bto16b(buf))/10);
	}

	return ret;
}

int Radio_Get_QRS(struct i2c_client *client,bool fm)
{
	u8 status;

	devTEF665x_Radio_Get_Quality_Status(client,fm,&status);
	
	return status;
}

int devTEF665x_Radio_Get_Signal_Status(struct i2c_client *client,bool fm,u16 *status)
{
	u8 buf[2];
	int r;
	
	r = devTEF665x_Get_Cmd(client,fm ? TEF665X_MODULE_FM : TEF665X_MODULE_AM,
			TEF665X_Cmd_Get_Signal_Status,
			buf,sizeof(buf));

	if(r == SET_SUCCESS){
		*status = Convert8bto16b(buf);
	}

	return r;
}

int Get_Signal_Status(struct i2c_client *client,bool fm,SignalSts_t *pST)
{
	int ret;
    u16 status;
	ret = devTEF665x_Radio_Get_Signal_Status(client,fm,&status);
	if(ret==SET_SUCCESS)
	{
      pST->Stereo=( status & bit15)? true :false;
	  pST->DigSignal=( status & bit14)? true :false;
	}
	return ret;
			
}

int Radio_Get_Level(struct i2c_client *client,bool fm)
{
	int16_t level;
	u8 status;
	
		if(SET_SUCCESS == devTEF665x_Radio_Get_Quality_Level(client,fm,&status,&level))
		{
			return level;
		}

	return -255;
}

int Radio_Signal_FM(struct i2c_client *client,FM_QData_t *pFMQ)
{
	bool stereo = false;
	SignalSts_t status;
	int nstereo;
	
    Get_Signal_Status(client,1,&status);
	stereo=status.Stereo;
				
	//*ret=Radio_Get_Level(client,1);
	  
	FM_Get_Quality_Data(client,pFMQ);

	printk("============pFMQ->FreqOffset=%d=======\n",pFMQ->FreqOffset);
	printk("============pFMQ->IFBandwidth=%d=====\n",pFMQ->IFBandwidth);
	printk("============pFMQ->Level=%d======\n",pFMQ->Level);
	printk("============pFMQ->Status=%d=======\n",pFMQ->Status);
	printk("============pFMQ->Modulation=%d=======\n",pFMQ->Modulation);
	printk("============pFMQ->UltraSonicNoise=%d=======\n",pFMQ->UltraSonicNoise);
	printk("============pFMQ->WidebandAMMultipath=%d=======\n",pFMQ->WidebandAMMultipath);
	
	nstereo = (stereo?1:0);
		
	return nstereo;
	
}

int Radio_Signal_AM(struct i2c_client *client,AM_QData_t *pAMQ)
{
	bool stereo = false;
	SignalSts_t status;
	int nstereo;
	
    Get_Signal_Status(client,1,&status);
	stereo=status.Stereo;

	AM_Get_Quality_Data(client,pAMQ);
	
	printk("============pAMQ->FreqOffset=%d=======\n",pAMQ->FreqOffset);
	printk("============pAMQ->IFBandwidth=%d=====\n",pAMQ->IFBandwidth);
	printk("============pAMQ->Level=%d======\n",pAMQ->Level);
	printk("============pAMQ->Status=%d=======\n",pAMQ->Status);

	nstereo = (stereo?1:0);
		
	return nstereo;
	
}


void TEF665x_dev_init(struct tef6657_t* nxp_dev)
{
	mutex_init(&nxp_dev->mu);
	nxp_dev->PowerSwitch=Radio_PowerSwitch;
	nxp_dev->ModeSwitch=Radio_ModeSwitch;
	//nxp_dev->Radio_Signal=Radio_Signal_tef665x;
}

void DevTEF665x_chip_init(struct i2c_client *client)
{
	DevTEF665x_Power_on(client);
	msleep(50);
	DevTEF665x_Boot_state(client);
	msleep(100);
	DevTEF665x_Idle_state(client);
	msleep(200);
	DevTEF665x_Wait_Active(client);
}

long tef6657_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct tef6657_t *dev = (struct tef6657_t *)filp->private_data;

	mutex_lock(&dev->mu);

	switch(cmd)
	{
		case TEF6657_CMD_fm:
			dev->ModeSwitch(dev,1,eAR_TuningAction_Preset,arg); //8980
			break;
		case TEF6657_CMD_am:
			dev->ModeSwitch(dev,0,eAR_TuningAction_Preset,arg);  //990
			break;
		case TEF6657_CMD_open:
			dev->PowerSwitch(dev,1);
			break;	
		case TEF6657_CMD_close:
			dev->PowerSwitch(dev,0);
			break;
		case TEF6657_CMD_GETamSTAUS:
			
			//Radio_t.Band=BAND_AM;
				
			break;		
		case TEF6657_CMD_GETfmSTAUS:

			//Radio_t.Band=BAND_FM;

			break;	
		case TEF6657_CMD_GEToirtSTAUS:

			//Radio_t.Band=BAND_OIRT;
			
			break;					
		default:
			printk("unknown ioctl command: %d\n", cmd);
			ret = -EINVAL;
			break;
	}
	
	mutex_unlock(&dev->mu);
	
	return ret;
}

/***********************************************************************
 * sysfs interface.
 ***********************************************************************/
static ssize_t show_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	int reg_val;
	int len;

	char *str_ledon = "ledon";

	struct i2c_client* client = container_of(dev,struct i2c_client, dev);

	devTEF665x_APPL_Get_Identification(client);

	reg_val = 12;

	len = snprintf(buf, 100,"%d\n", reg_val);

	printk("=============len= %d=================\n",len);


	return len;

}

static ssize_t set_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{


	printk("=============buf = %s count= %d=================\n",buf,count);

	struct i2c_client* client = container_of(dev,struct i2c_client, dev);
	
	unsigned long reg_val = 0;
	char messages[256];

	if (count > 256)
		count = 256;
	
	strncpy(messages, buf, count);

	reg_val = simple_strtoul(messages, NULL, 10);
	
	printk(KERN_INFO "simple_strtoul  %d \n", reg_val); 

	//reg_val = i2c_smbus_read_byte_data(client, reg_val);

	//printk(KERN_INFO "i2c_smbus_read_byte_data  %d \n", reg_val); 
	
	return 0;

}

static DEVICE_ATTR(systef6657, S_IWUSR | S_IRUGO,
		   show_mode, set_mode);  //show_mode ---read  set_mode---write
	   
static struct attribute *tef6657_attributes[] = {
	&dev_attr_systef6657.attr,
	NULL
};

static const struct attribute_group tef6657_attr_group = {
	.attrs = tef6657_attributes,
};


static ssize_t tef6657_read(struct file *filp, char __user *buf, size_t count, loff_t *ppos)
{
	struct tef6657_t *dev = (struct tef6657_t *)filp->private_data;
	size_t len;
	//int16_t leve;
	int m_nstatus;
	int str[12];
	u8 data;
	int ret;

	FM_QData_t fm_quality;
	AM_QData_t am_quality;


	devTEF665x_APPL_Get_Operation_Status(dev->tef6657_client,&data);

	switch(data)
	{
		case RADIO_BOOT_STATE:
			 str[0] = data;
			 len = sizeof(str);
			 if (count >= len) 
			 {
				 ret = copy_to_user(buf, str, len);  //&str
				 return ret ? ret : len;
			 } 
			 else 
			 {
				 return -EINVAL;
			 }			 
			 break;
			
		case RADIO_IDLE_STATE:
			 str[0] = data;
			 len = sizeof(str);
			 if (count >= len) 
			 {
				 ret = copy_to_user(buf, str, len);  //&str
				 return ret ? ret : len;
			 } 
			 else 
			 {
				 return -EINVAL;
			 }				
			 break;
			
		case RADIO_STANBDY_STATE:
			 str[0] = data;
			 len = sizeof(str);
			 if (count >= len) 
			 {
				 ret = copy_to_user(buf, str, len);  //&str
				 return ret ? ret : len;
			 } 
			 else 
			 {
				 return -EINVAL;
			 }				
			 break;
			
		case RADIO_FM_STATE:

			 m_nstatus = Radio_Signal_FM(dev->tef6657_client,&fm_quality);

			 str[0] = data;
			 str[1] = m_nstatus;
			 str[2] = fm_quality.FreqOffset;  //offset
			 str[3] = fm_quality.IFBandwidth;  //bandwidth
			 str[4] = fm_quality.Level;
			 str[5] = fm_quality.Modulation;
			 str[6] = fm_quality.Status;
			 str[7] = fm_quality.UltraSonicNoise; //usn
			 str[8] = fm_quality.WidebandAMMultipath;  //wam
			 
			 len = sizeof(str);
			 if (count >= len) 
			 {
				 ret = copy_to_user(buf, str, len);  //&str
				 return ret ? ret : len;
			 } 
			 else 
			 {
				 return -EINVAL;
			 }
				
			 break;
			
		case RADIO_AM_STATE:

			 m_nstatus = Radio_Signal_AM(dev->tef6657_client,&am_quality);

			 str[0] = data;
			 str[1] = m_nstatus;
			 str[2] = am_quality.FreqOffset;
			 str[3] = am_quality.IFBandwidth;
			 str[4] = am_quality.Level;
			 str[5] = am_quality.Modulation;
			 str[6] = am_quality.Status;

			 len = sizeof(str);
			 if (count >= len) 
			 {
				ret = copy_to_user(buf, str, len);  //&str
				return ret ? ret : len;
			 } 
			 else 
			 {
				return -EINVAL;
			 }			
			break;
		default:
				printk("Get_Operation_Status failed!\n");
			break;	
	}
	
	return -EINVAL;

	//m_nstatus = dev->Radio_Signal(dev,&leve,data);
	
}

int tef6657_open (struct inode *node, struct file *file)
{

	file->private_data = tef6657_dev;

	return 0;
}

static struct file_operations tef6657_fops = 
{
	.owner=THIS_MODULE,
	.open=tef6657_open,
	.read=tef6657_read,
	.unlocked_ioctl=tef6657_ioctl,
};

/*struct miscdevice misc_tef6657 = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DRIVER_NAME,
	.fops = &tef6657_fops,
}; */

static int tef6657_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int ret = 0;

	printk("=============tef6657_probe===============\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) 
	{
		ret = -ENODEV;
		goto alloc_failed;
	}

	
	tef6657_dev = (struct tef6657_t *)kzalloc(sizeof(struct tef6657_t), GFP_KERNEL);
	
	if(NULL == tef6657_dev)
	{
		printk("alloc tef6657_dev failed!\n");
		ret = -ENOMEM;
		goto alloc_failed;	
	}
	
	TEF665x_dev_init(tef6657_dev);

	/*ret = misc_register(&misc_tef6657);
	if(ret)
	{
		printk("misc_register failed!\n");
		goto misc_failed;
	} */

	tef6657_dev->tef6657_client = client;

	DevTEF665x_chip_init(tef6657_dev->tef6657_client);

	sysfs_create_group(&client->dev.kobj, &tef6657_attr_group);


	return 0;

misc_failed:
	printk("=============misc_failed===============\n");
	kfree(tef6657_dev);
	
alloc_failed:
	printk("=============alloc_failed===============\n");
	return ret;

}

static int tef6657_remove(struct i2c_client *client)
{
	int ret = 0;
	
	//misc_deregister(&misc_tef6657);

	kfree(tef6657_dev);
	
	return ret;
}

static const struct i2c_device_id tef6657_id[] = {
	{DRIVER_NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, tef6657_id);

static struct i2c_driver tef6657_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= DRIVER_NAME,
	},
	.probe		= tef6657_probe,
	.remove		= tef6657_remove,
	.id_table	= tef6657_id,
};

//module_i2c_driver(tef6657_driver);

static int __init tef6657_init(void)
{
	printk("=============tef6657_init=================\n");
	return i2c_add_driver(&tef6657_driver);
}

static void __exit tef6657_exit(void)
{
	i2c_del_driver(&tef6657_driver);
}

module_init(tef6657_init);
module_exit(tef6657_exit);


MODULE_DESCRIPTION("TEF665X Car Radio Enhanced Selectivity Tuner");
MODULE_AUTHOR("Andy Wang <wangliang@yeamax.com>");
MODULE_LICENSE("GPL v2");

