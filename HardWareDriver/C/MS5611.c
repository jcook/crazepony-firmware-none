#include "config.h"
#include "MS5611.h"
#include <math.h>
#include "stm32f10x_it.h"

#undef ALTI_SPEED

#define MS5611Press_OSR  MS561101BA_OSR_4096  //气压采样精度
#define MS5611Temp_OSR   MS561101BA_OSR_4096  //温度采样精度

// 气压计状态机
#define SCTemperature  0x01	  //开始 温度转换
#define CTemperatureing  0x02  //正在转换温度
#define SCPressure  0x03	  //开始转换 气压
#define SCPressureing  0x04	  //正在转换气压值

/* store the recent data */
#define MOVAVG_SIZE  	5
#define START_PA_LOOP	1
static uint8_t  Now_doing = SCTemperature;	//当前转换状态
static uint16_t PROM_C[MS561101BA_PROM_REG_COUNT]; //标定值存放
static uint32_t Current_delay=0;	    //转换延时时间 us 
static uint32_t Start_Convert_Time; //启动转换时的 时间 us 
static uint32_t  D1, D2;

static float Alt_Offset_m = 0;

//存放着0米(离起飞所在平面)时 对应的气压值  这个值存放上电时的气压值
// !0 means is initialized
float Alt_offset_Pa = 0;

//interface for outside 
uint8_t Baro_ALT_Updated = 0; //气压计高度更新完成标志。
//units (Celsius degrees*100, mbar*100  ).
//单位 [温度 度] [气压 帕]  [高度 米] 
volatile float MS5611_VerticalSpeed;

// 延时表单位 us 	  不同的采样精度对应不同的延时值
uint32_t MS5611_Delay_us[9] = {
	1500,//MS561101BA_OSR_256 0.9ms  0x00
	1500,//MS561101BA_OSR_256 0.9ms  
	2000,//MS561101BA_OSR_512 1.2ms  0x02
	2000,//MS561101BA_OSR_512 1.2ms
	3000,//MS561101BA_OSR_1024 2.3ms 0x04
	3000,//MS561101BA_OSR_1024 2.3ms
	5000,//MS561101BA_OSR_2048 4.6ms 0x06
	5000,//MS561101BA_OSR_2048 4.6ms
	11000,//MS561101BA_OSR_4096 9.1ms 0x08
};

// FIFO 队列					
static float Temp_buffer[MOVAVG_SIZE], Press_buffer[MOVAVG_SIZE];
static uint8_t array_index = 0;

#define MS5611_RESULT_TEMP	0
#define MS5611_RESULT_PRESS	1
#define MS5611_RESULT_ALT	2
#define MS5611_RESULT_NUM	3
float MS5611_Result[MS5611_RESULT_NUM];

float MS561101BA_getAvg(float * buff, int size);
void MS561101BA_setResult(void);
void MS561101BA_clearResult(void);
float MS561101BA_get_altitude(void);
/* return D1 or D2 depneds on timing. */
uint32_t MS561101BA_GetD(void);

/* insert temp & pressure group. */
void MS561101BA_NewElement(float temp, float pressure)
{
	if (array_index == 0) {
		//Baro_ALT_Updated = 0;
	}
	Temp_buffer[array_index] = temp;	
	Press_buffer[array_index] = pressure;
	array_index = (array_index + 1) % MOVAVG_SIZE;
	
	if (array_index == 0) {
		MS561101BA_setResult();
	}
}

void MS561101BA_setResult(void)
{
	MS5611_Result[MS5611_RESULT_TEMP] = MS561101BA_getAvg(Temp_buffer, MOVAVG_SIZE);
	MS5611_Result[MS5611_RESULT_PRESS] = MS561101BA_getAvg(Press_buffer, MOVAVG_SIZE);
	MS5611_Result[MS5611_RESULT_ALT] = MS561101BA_get_altitude();
	
	Baro_ALT_Updated = 0xff; 	//高度更新 完成。
	
	//printf("PA: %5.2f %5.2f %5.2f\r\n", MS5611_Result[MS5611_RESULT_TEMP], MS5611_Result[MS5611_RESULT_PRESS], MS5611_Result[MS5611_RESULT_ALT]);
}

//读取队列的平均值
float MS561101BA_getAvg(float * buff, int size) 
{
	float sum = 0.0;
	int i;
	for(i=0; i < size; i++) 
	{
		sum += buff[i];
	}
	return sum / size;
}


/**************************实现函数********************************************
*函数原型:		void MS561101BA_readPROM(void)
*功　　能:	    读取 MS561101B 的工厂标定值
读取 气压计的标定值  用于修正温度和气压的读数
*******************************************************************************/
void MS561101BA_readPROM(void) 
{
	u8  inth,intl;
	uint8_t i2cret[2];
	int i;
	for (i=0;i<MS561101BA_PROM_REG_COUNT;i++) 
	{
		#ifdef DEBUG_HW_I2C
			i2cRead(MS5611_ADDR, MS561101BA_PROM_BASE_ADDR + (i * MS561101BA_PROM_REG_SIZE),2, i2cret); 
			PROM_C[i]=i2cret[0]<<8 | i2cret[1];
		#else
			IIC_Start();
			IIC_Send_Byte(MS5611_ADDR);
			IIC_Wait_Ack();
			IIC_Send_Byte(MS561101BA_PROM_BASE_ADDR + (i * MS561101BA_PROM_REG_SIZE));
			IIC_Wait_Ack();	
			IIC_Stop();
			delay_us(5);
			IIC_Start();
			IIC_Send_Byte(MS5611_ADDR+1);  //进入接收模式	
			delay_us(1);
			IIC_Wait_Ack();
			inth = IIC_Read_Byte(1);  //带ACK的读数据
			delay_us(1);
			intl = IIC_Read_Byte(0);	 //最后一个字节NACK
			IIC_Stop();
			
			PROM_C[i] = (((uint16_t)inth << 8) | intl);
		#endif
	}
}

/**************************实现函数********************************************
*函数原型:		void MS561101BA_reset(void)
*功　　能:	    发送复位命令到 MS561101B 
*******************************************************************************/
void MS561101BA_reset(void) 
{
	#ifdef DEBUG_HW_I2C
		i2cWrite(MS5611_ADDR, MS561101BA_RESET, 1); 
	#else
	IIC_Start();
    IIC_Send_Byte(MS5611_ADDR); //写地址
	IIC_Wait_Ack();
    IIC_Send_Byte(MS561101BA_RESET);//发送复位命令
	IIC_Wait_Ack();	
    IIC_Stop();
	#endif
}

/**************************实现函数********************************************
*函数原型:		void MS561101BA_startConversion(uint8_t command)
*功　　能:	    发送启动转换命令到 MS561101B
可选的 转换命令为 MS561101BA_D1  转换气压
				  MS561101BA_D2  转换温度	 
*******************************************************************************/
void MS561101BA_startConversion(uint8_t command) 
{
#ifdef DEBUG_HW_I2C
	i2cWrite(MS5611_ADDR, command, 1); 
#else
	// initialize pressure conversion
	IIC_Start();
	IIC_Send_Byte(MS5611_ADDR); //写地址
	IIC_Wait_Ack();
	IIC_Send_Byte(command); //写转换命令
	IIC_Wait_Ack();	
	IIC_Stop();
#endif
}
#define CMD_ADC_READ            0x00 // ADC read command
/**************************实现函数********************************************
*函数原型:		unsigned long MS561101BA_getConversion(void)
*功　　能:	    读取 MS561101B 的转换结果	 
*******************************************************************************/
uint32_t MS561101BA_getConversion(void) 
{
	uint32_t conversion = 0;
	u8 temp[3];
	#ifdef DEBUG_HW_I2C
		i2cRead(MS5611_ADDR,CMD_ADC_READ ,3, temp); 
		conversion=temp[0] << 16 | temp[0] <<8 | temp[2];
	#else
	IIC_Start();
	IIC_Send_Byte(MS5611_ADDR); //写地址
	IIC_Wait_Ack();
	IIC_Send_Byte(0);// start read sequence
	IIC_Wait_Ack();	
	IIC_Stop();
	
	IIC_Start();
	IIC_Send_Byte(MS5611_ADDR+1);  //进入接收模式	
	IIC_Wait_Ack();
	temp[0] = IIC_Read_Byte(1);  //带ACK的读数据  bit 23-16
	temp[1] = IIC_Read_Byte(1);  //带ACK的读数据  bit 8-15
	temp[2] = IIC_Read_Byte(0);  //带NACK的读数据 bit 0-7
	IIC_Stop();
	conversion = (unsigned long)temp[0] * 65536 + (unsigned long)temp[1] * 256 + (unsigned long)temp[2];
	#endif
	return conversion;
}

/**************************实现函数********************************************
*函数原型:		void MS561101BA_init(void)
*功　　能:	    初始化 MS561101B 
*******************************************************************************/
void MS5611_Init(void) 
{  
	MS561101BA_reset(); // 复位 MS561101B 
	delay_ms(100); // 延时 
	MS561101BA_readPROM(); // 读取EEPROM 中的标定值 待用	
}

/**************************实现函数********************************************
*函数原型:		void MS561101BA_GetD2(void)
*功　　能:	    读取 温度转换结果	 
*******************************************************************************/
uint32_t MS561101BA_GetD(void)
{	
	return MS561101BA_getConversion();	
}

/**************************实现函数********************************************
*函数原型:		float MS561101BA_get_altitude(void)
*功　　能:	    将当前的气压值转成 高度。	 
*******************************************************************************/
float MS561101BA_get_altitude(void)
{
	float Altitude;
	static uint8_t cnt = 0;
#ifdef ALTI_SPEED
	static float AltPre;
	float dz,dt;
	uint32_t current=0;
	static uint32_t tp=0;
#endif
	// 是否初始化过0米气压值？
	// FIXME: if cnt is small, first PA is not suitable.
	
	if(Alt_offset_Pa == 0) {
		if (MS5611_Result[MS5611_RESULT_PRESS]) {			
			cnt++;
			if (cnt == START_PA_LOOP) {
				Alt_offset_Pa = MS5611_Result[MS5611_RESULT_PRESS];
				//printf("firstPa: %f\r\n", Alt_offset_Pa);
			}
		}
		Altitude = 0; //高度 为 0		
		return Altitude;
	}
	
	//计算相对于上电时的位置的高度值 。单位为m
	Altitude = 4433000.0 * (1 - pow((MS5611_Result[MS5611_RESULT_PRESS] / Alt_offset_Pa), 0.1903))*0.01f;
	Altitude = Altitude + Alt_Offset_m ;  //加偏置

#ifdef ALTI_SPEED
	current=micros();
	dt=(tp>0)?((current - tp)/1000000.0f):0;
	tp=current;
	dz=(Altitude-AltPre);
	AltPre=Altitude;	//m
	if(dt>0)
		MS5611_VerticalSpeed =  dz / dt;
#endif
	printf("hPA: %f / %f / %.2f\r\n", MS5611_Result[MS5611_RESULT_PRESS], Alt_offset_Pa, Altitude);
	return Altitude; 
}

/* Calculate temperature and pressure. */
void MS561101BA_Calculate(uint32_t D1, uint32_t D2)
{
	int32_t dT, TEMP, P;
	int64_t OFF, SENS;
	
	dT  = D2 - (((int32_t)PROM_C[4]) << 8);
	TEMP = 2000 + ((dT * (int64_t)PROM_C[5]) >> 23);
#if 0	
	if (TEMP < 2000)
	{   // second order temperature compensation

	}
#endif	
	OFF  = (((int64_t)PROM_C[1]) << 16) + ((((int64_t)PROM_C[3]) * dT) >> 7);
	SENS = (((int64_t)PROM_C[0]) << 15) + ((((int64_t)PROM_C[2]) * dT) >> 8);
	
	P = (int32_t)((((((int64_t)D1) * SENS) >> 21) - OFF) >> 15);
	
	/* To display value */
	MS561101BA_NewElement(TEMP * 0.01f, P * 0.01f);
}
#if 1
/* for testing only */
void MS5611_ReadTemperature(void)
{
	int32_t dT, TEMP, i;
	
	MS561101BA_startConversion(MS561101BA_D2 + MS5611Temp_OSR);
	Current_delay = MS5611_Delay_us[MS5611Temp_OSR] ;//转换时间
	Start_Convert_Time = micros(); //计时开始
	while (!((micros()-Start_Convert_Time) > Current_delay))
		;
	
	D2 = MS561101BA_GetD();
	dT  = D2 - (((int32_t)PROM_C[4]) << 8);
	TEMP = 2000 + ((dT * (int64_t)PROM_C[5]) >> 23);
	printf("temp: %5.2f ", TEMP * 0.01f);
	
	i = 50000;
	
	while (i--)
		;
}
/* for testing only */
void MS5611_ReadTemperatureAndPressure(void)
{
	float Alt;
	int32_t dT, TEMP, P, i;
	int64_t OFF, SENS;
	static int32_t P_min = 0, P_max = 0, T_min = 0, T_max = 0, dP = 0, dTEMP = 0, firstP = 0;
	
	
	MS561101BA_startConversion(MS561101BA_D2 + MS5611Temp_OSR);
	Current_delay = MS5611_Delay_us[MS5611Temp_OSR] ;//转换时间
	Start_Convert_Time = micros(); //计时开始
	
	
	while (!((micros()-Start_Convert_Time) > Current_delay))
		;
	D2 = MS561101BA_GetD();
	
	MS561101BA_startConversion(MS561101BA_D1 + MS5611Press_OSR);
	Current_delay = MS5611_Delay_us[MS5611Press_OSR];//转换时间
	Start_Convert_Time = micros();//计时开始	
	while (!((micros()-Start_Convert_Time) > Current_delay))
		;
	
	D1 = MS561101BA_GetD();
	
	dT  = D2 - (((int32_t)PROM_C[4]) << 8);
	TEMP = 2000 + ((dT * (int64_t)PROM_C[5]) >> 23);

#if 0	
	if (TEMP < 2000)
	{   // second order temperature compensation

	}
#endif	
	OFF  = (((int64_t)PROM_C[1]) << 16) + ((((int64_t)PROM_C[3]) * dT) >> 7);
	SENS = (((int64_t)PROM_C[0]) << 15) + ((((int64_t)PROM_C[2]) * dT) >> 8);
	
	P = (int32_t)((((((int64_t)D1) * SENS) >> 21) - OFF) >> 15);

	if (P_min == 0) {
		P_min = P_max = P;
		T_min = T_max = TEMP;
		firstP = P;
	}
	
	if (P > P_max)
		P_max = P;
	if (P < P_min)
		P_min = P;
	if (TEMP > T_max)
		T_max = TEMP;
	if (TEMP < T_min)
		T_min = TEMP;
#if 0
	if (dP < (P_max - P_min)) {
		dP = P_max - P_min;
		if (dP != 0)
			printf("***** dP %d ******\r\n", dP);
	}
	if (dTEMP < (T_max - T_min)) {
		dTEMP = T_max - T_min;
		if (dTEMP != 0)
			printf(">>>>>> dT %d <<<<<<<\r\n", dTEMP);
	}
#endif
	
	Alt = 4433000.0 * (1 - pow(((float)P / (float)firstP), 0.1903))*0.01f;
	
	printf("hPA: %d / %d / %.2f", P, firstP, Alt);
	
	i = 0xFFFFF;
	
	while (i--)
		;
}
#endif

void MS5611_Thread(void) 
{
	switch(Now_doing) {
 	case SCTemperature:
next_loop:
		MS561101BA_startConversion(MS561101BA_D2 + MS5611Temp_OSR);
		Current_delay = MS5611_Delay_us[MS5611Temp_OSR];
		Start_Convert_Time = micros();
		Now_doing = CTemperatureing;
		break;
		
	case CTemperatureing:
		if((micros()-Start_Convert_Time) > Current_delay) {
			D2 = MS561101BA_GetD();	
			MS561101BA_startConversion(MS561101BA_D1 + MS5611Press_OSR);
			Current_delay = MS5611_Delay_us[MS5611Press_OSR];
			Start_Convert_Time = micros();
			Now_doing = SCPressureing;
		}
		break;
 
	case SCPressureing:
		if((micros()-Start_Convert_Time) > Current_delay) {
			D1 = MS561101BA_GetD();
			/* D1 & D2 are ready for calculatation. */
			MS561101BA_Calculate(D1, D2);
			goto next_loop;
		}
		break;
	default:
		Now_doing = CTemperatureing;
		break;
	}
}
//注意，使用前确保
uint8_t WaitBaroInitOffset(void)
{
	while(!Alt_offset_Pa) {
		MS5611_Thread();
		/* assert error??? */
	}
	return 1;
}

//------------------End of File----------------------------
