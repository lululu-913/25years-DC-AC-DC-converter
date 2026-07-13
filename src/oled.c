#include "oled.h"
#include "oledfont.h"
#include "string.h"
#include <math.h>
#include <stdbool.h>
//备注：注释内的为老代码
void OLED_WriteCommand(Uchar Data)
{
    I2C_Start3();
    OLED_WriteByte(0x00);
    I2C_Ack3();
    OLED_WriteByte(Data);
    I2C_Ack3();
    I2C_Stop3();
}


void OLED_WriteData(Uchar Data)
{
    I2C_Start3();
    OLED_WriteByte(0x40);
    I2C_Ack3();
    OLED_WriteByte(Data);
    I2C_Ack3();
    I2C_Stop3();
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void OLED_Init(void)
{

    EALLOW;
    GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO28 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO29 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO29 = 1;
    EDIS;

    DELAY_US(2);

    OLED_WriteCommand(0xae);    //--turn off oled panel

    OLED_WriteCommand(0x00);    //--set low column address
    OLED_WriteCommand(0x10);    //--set high column address

    OLED_WriteCommand(0x40);    //--set start line address

    OLED_WriteCommand(0xb0);    //--set page address

    OLED_WriteCommand(0x81);    //--set contrast control register
    OLED_WriteCommand(0x00);    //调节亮度，0X00-0XFF

    OLED_WriteCommand(0xa1);    //--set segment re-map 127 to 0   a0:0 to seg127
    OLED_WriteCommand(0xa6);    //--set normal display

    OLED_WriteCommand(0xc8);    //--set com(N-1)to com0  c0:com0 to com(N-1)

    OLED_WriteCommand(0xa8);    //--set multiples ratio(1to64)
    OLED_WriteCommand(0x3f);    //--1/64 duty

    OLED_WriteCommand(0xd3);    //--set display offset
    OLED_WriteCommand(0x00);    //--not offset

    OLED_WriteCommand(0xd5);    //--set display clock divide ratio/oscillator frequency
    OLED_WriteCommand(0x80);    //--set divide ratio

    OLED_WriteCommand(0xd9);    //--set pre-charge period
    OLED_WriteCommand(0xf1);

    OLED_WriteCommand(0xda);    //--set com pins hardware configuration
    OLED_WriteCommand(0x12);

    OLED_WriteCommand(0xdb);    //--set vcomh
    OLED_WriteCommand(0x40);

    OLED_WriteCommand(0x8d);    //--set chare pump enable/disable
    OLED_WriteCommand(0x14);    //--set(0x10) disable
    //OLED_WriteCommand(0x10);  //--set(0x10) disable
    OLED_WriteCommand(0xaf);    //--turn on oled panel

    OLED_Clean();

}

/*void OLED_SetPos(Uchar x,Uchar page){
    OLED_WriteCommand(0xb0+page);
    OLED_WriteCommand(((x&0xf0)>>4)|0x10);
    OLED_WriteCommand((x&0x0f)|0x01);
}*/
void OLED_SetPos(Uchar x,Uchar page)
{
    OLED_WriteCommand(0xb0+page);
    OLED_WriteCommand(((x&0xf0)>>4)|0x10);
    OLED_WriteCommand(x&0x0f);
}

void OLED_ShowChar(Uchar x,Uchar y,char chr){//字体大小为8*16像素 X{0-15} Y{0-3}
    unsigned char c=chr-' ',i=0;
    unsigned char page=y*2;
    x=x*8;
    if(x>127){
        x=0;
        page=page+2;
    }
    OLED_SetPos(x,page);
    for(i=0;i<8;i++)
    {
        OLED_WriteData(F8X16[c*16+i]);
    }
    OLED_SetPos(x,page+1);
    for(i=0;i<8;i++)
    {
        OLED_WriteData(F8X16[c*16+i+8]);
    }
}

void OLED_ShowString(Uchar x,Uchar y,char *s){//字体大小为8*16像素 X{0-15} Y{0-3} 同一行放不下的会放到下一行
    int p=0;
    while(*(s+p)!='\0'){
        OLED_ShowChar(x+p,y,*(s+p));
        p++;
    }
}

void OLED_ShowNum(Uchar x,Uchar y,int n,Uchar len){
    int p=0;
    if(n<0){
        OLED_ShowChar(x,y,'-');
        n=-n;
    }else{
        OLED_ShowChar(x,y,' ');
    }
    if(n==0){
        OLED_ShowChar(x+len-1,y,'0');
        p++;
    }
    while(n>0&&p<len){
        OLED_ShowChar(x+len-p-1,y,n%10+48);
        p++;
        n=n/10;
    }
    while(p<len-1){
        OLED_ShowChar(x+len-p-1,y,' ');
        p++;
    }

}

/*void OLED_Clean(void){
    Uchar i,j;
    Uint k=0;
    for(j=0;j<8;j++){
        OLED_WriteCommand(0x22);
        OLED_WriteCommand(j);
        OLED_WriteCommand(0x07);
        for(i=0;i<128;i++){
            OLED_WriteData(0);
            k=k+1;
        }
    }
}*/
void OLED_Clean(void)
{
    Uchar i,j;
    for(j=0;j<8;j++){
        OLED_WriteCommand(0xb0+j);
        OLED_WriteCommand(0x00);
        OLED_WriteCommand(0x10);
        for(i=0;i<128;i++)
        {
            OLED_WriteData(0);
        }
    }
}

void OLED_Fill(void)//全屏填充
{
    Uchar m,n;
    for(m=0;m<8;m++)
    {
        OLED_WriteCommand(0xb0+m);       //page0-page1
        OLED_WriteCommand(0x00);     //low column start address
        OLED_WriteCommand(0x10);     //high column start address
        for(n=0;n<128;n++)
        {
        OLED_WriteData(1);
        }
    }
}

//打印浮点数，x{0-15}，y{0-3}，n为浮点数的数据位数 如3.29时n=3
void OLED_ShowFloat(uint8_t x,uint8_t y,float num,uint8_t n){
    uint8_t a,cnt;
    float b;
    cnt=0;
    b=num-(uint8_t)num; //小数部分
    a=(uint8_t)num;     //整数部分
    while(a){
        if(a<10)
        {
            OLED_ShowChar(x,y,((uint8_t)num)+'0');
            a/=10;
            cnt++;
        }
        else
        {
            OLED_ShowChar(x,y,((uint8_t)(a/10))+'0');
            cnt++;
            OLED_ShowChar(x+cnt,y,((uint8_t)(a%10))+'0');
            cnt++;
            a=0;
        }
    }    //a是一个大于0的值时， 上面这个if else用来写整数部分a并且写完后把a置0，并用cnt记录a的位数
    if(cnt==0){
        OLED_ShowChar(x,y,'0');
        cnt++;
    }//上面这个if是a为0时，直接打印0并且位数记为1

    OLED_ShowChar(x+cnt,y,'.');//在整数部分后打印一个小数点
    cnt++;//小数点也让计数位加一
    while(1){
        OLED_ShowChar(x+cnt,y,((int)(b*10))+'0');
        cnt++;
        b=b*10-(uint8_t)(b*10);
        if(cnt==n+1){
            break;
        }
    }//上面这个while用来打印小数部分
}

/*数字转换为字符串*/

static char table[]={'0', '1', '2', '3', '4', '5', '6', '7', '8', '9'};

void num2char(unsigned char *str, double number, uint8_t g, uint8_t l)
{
    uint8_t i;
    int temp = number/1;
    double t2 = 0.0;
    for (i = 1; i<=g; i++)
    {
        if (temp==0)
            str[g-i] = table[0];
        else
            str[g-i] = table[temp%10];
        temp = temp/10;
    }
    *(str+g) = '.';
    temp = 0;
    t2 = number;
    for(i=1; i<=l; i++)
    {
        temp = t2*10;
        str[g+i] = table[temp%10];
        t2 = t2*10;
    }
    *(str+g+l+1) = '\0';
}

//--------------------------------------------------------------
// Prototype      : void OLED_ON(void)
// Calls          :
// Parameters     : none
// Description    : 将OLED从休眠中唤醒
//--------------------------------------------------------------
void OLED_ON(void)
{
    OLED_WriteCommand(0X8D);  //设置电荷泵
    OLED_WriteCommand(0X14);  //开启电荷泵
    OLED_WriteCommand(0XAF);  //OLED唤醒
}

//--------------------------------------------------------------
// Prototype      : void OLED_OFF(void)
// Calls          :
// Parameters     : none
// Description    : 让OLED休眠 -- 休眠模式下,OLED功耗不到10uA
//--------------------------------------------------------------
void OLED_OFF(void)
{
    OLED_WriteCommand(0X8D);  //设置电荷泵
    OLED_WriteCommand(0X10);  //关闭电荷泵
    OLED_WriteCommand(0XAE);  //OLED休眠
}

//显示汉字 X{0-7}  Y{0-4}
//hzk 用取模软件得出的数组
void OLED_ShowCHinese(Uchar x,Uchar y,Uchar no)
{
    uint8_t t;
    x *= 16;
    y *= 2;
    OLED_SetPos(x,y);
    for(t=0;t<8;t++)
    {
        OLED_WriteData(Hzk[2*no][t]);
    }
    OLED_SetPos(x+8,y);
    for(t=8;t<16;t++)
    {
        OLED_WriteData(Hzk[2*no][t]);
    }
    OLED_SetPos(x,y+1);
    for(t=0;t<8;t++)
    {
        OLED_WriteData(Hzk[2*no+1][t]);
    }
    OLED_SetPos(x+8,y+1);
    for(t=8;t<16;t++)
    {
        OLED_WriteData(Hzk[2*no+1][t]);
    }
}

/*
 *oled画点函数，x:0-127,y:0-63
 *只能在y/8的1*8一列中显示一个点
 */
void OLED_DrawPoint(Uchar x,Uchar y)
{
    if(x >= 128)
        x -= 128;
    if(y >= 64)
        y -= 64;
    OLED_SetPos(x, y/8);
    switch(y%8)
    {
        case 0:OLED_WriteData(0x01);break;
        case 1:OLED_WriteData(0x02);break;
        case 2:OLED_WriteData(0x04);break;
        case 3:OLED_WriteData(0x08);break;
        case 4:OLED_WriteData(0x10);break;
        case 5:OLED_WriteData(0x20);break;
        case 6:OLED_WriteData(0x40);break;
        case 7:OLED_WriteData(0x80);break;
    }
}

/*
 *oled去点函数，x:0-127,y:0-63
 */
void OLED_ErasePoint(Uchar x,Uchar y)
{
    if(x >= 128)
        x -= 128;
    if(y >= 64)
        y -= 64;
    OLED_SetPos(x, y/8);
    OLED_WriteData(0x00);
}

/*
 * 绘画sin，改变Phi能使得图形左右滚动，默认Hz为50，phi不可大于255，最高不能超过100hz
 */
//by:fan
void OLED_DrawSin(float32 FF, Uchar Phi)
{
    uint8_t jj,jjj = 0;
    FF /= 50;
    FF = 2- FF;
//    if(Phi > 0)
//    {
//        for(jj=Phi-1;jj<128+Phi-1;jj++)
//        {
//            OLED_ErasePoint(jjj*FF,(Uint8)((sinf((float32)(jj)/16)+1)*32));  //32/50
//            jjj++;
//        }
//    }
//    else
//    {
//        for(jj=101;jj<227;jj++)
//        {
//            OLED_ErasePoint(jjj*FF,(Uint8)((sinf((float32)(jj)/16)+1)*32));  //32/50
//            jjj++;
//        }
//    }
//    jjj = 0;
    for(jj=Phi;jj<128+Phi;jj++)
    {
        OLED_DrawPoint(jjj*FF,(uint8_t)((sinf((float32)(jj)/16)+1)*32));  //32/50
        jjj++;
    }
}
