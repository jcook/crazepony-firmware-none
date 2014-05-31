#include "config.h"
#include "extern_variable.h"

uint32_t IMU_CYCTIME;

 
void NVIC_INIT(void)
{
    TimerNVIC_Configuration();//��ʱ���ж�����
    UART1NVIC_Configuration();//����1�ж�����
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		
*��������:		
*******************************************************************************/
void SYSTICK_INIT(void)
{
    SysTick->LOAD = 0xFFFFFF;
    SysTick->VAL = 0x0;
    SysTick->CTRL = 0x01;//����systick,8��Ƶ(9M),���ж�,���Ĵ�������λΪ���ر�־,=1˵��������,��ȡ���Ĵ�������λ�Զ�����
    
}


uint32_t GET_NOWTIME(uint32_t * lasttime)//���ص�ǰsystick������ֵ,32λ
{
    uint32_t temp,temp1,temp2;
    
    temp1 = SysTick->VAL;
    temp = SysTick->CTRL;
    if(temp&(1<<16)) 
            temp2 = *lasttime + 0xffffff - temp1;//��������
    else
            temp2 = *lasttime - temp1;
    *lasttime = temp1;
    if(temp2>100000)	return 0;
    return temp2;
}


