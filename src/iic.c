#include "F28x_Project.h"
#include "inc/hw_types.h"
#include "iic.h"
void I2C_Ack3(void)
{
    resetSDA;
    DELAY_US(1);
    setSCL;
    DELAY_US(1);
    resetSCL;
    DELAY_US(1);
}


void I2C_NAck3(void)
{
    setSDA;
    DELAY_US(1);
    setSCL;
    DELAY_US(1);
    resetSCL;
    DELAY_US(1);
}


void I2C_Start3(void)
{
    setSDA;
    DELAY_US(1);
    setSCL;
    DELAY_US(1);
    resetSDA;
    DELAY_US(1);
    resetSCL;
    DELAY_US(1);
    OLED_WriteByte(0x78);
    I2C_Ack3();//*这句很重要
}

void I2C_Stop3(void)
{
    setSCL;
    DELAY_US(1);
    resetSDA;
    DELAY_US(1);
    setSDA;
    DELAY_US(1);
}


void OLED_WriteByte(uint8_t mcmd)
{
    uint8_t length = 8;           // Send Command

    while(length--)
    {
        if(mcmd & 0x80)
        {
            setSDA;
        }
        else
        {
            resetSDA;
        }
        DELAY_US(1);
        setSCL;
        DELAY_US(1);
        resetSCL;
        DELAY_US(1);
        mcmd = mcmd << 1;
    }
}

//2024.3.15 由搁浅飞云改进
