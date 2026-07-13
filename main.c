//#############################################################################
//
// TITLE:  能量回馈变流器负载试验装置 (F280049)
//
// 移植自 F28069，实现：
//  变流器1 (DC-AC): EPWM4/5/6 三相 SPWM 逆变
//  变流器2 (AC-DC): EPWM1/2/3 同步整流 → 能量回馈
//  EPWM7/8 未使用
//  3 种模式：关断 / 变频(20-100Hz) / 能量回馈
//  控制算法：电压外环 PID + 电流内环 PI×3（PLL/PR 保留备用）
//
//#############################################################################

#include "F28x_Project.h"
#include "driverlib.h"
#include "math.h"
#include "oled.h"

//********** 函数声明 **********//
__interrupt void adcA1ISR(void);
__interrupt void epwm1_isr(void);

void initADC(void);
void initEPWM(void);
void initADCSOC(void);
void Init_KEY(void);
char KEY_Scan(char mode);
void KEY_Control(int key);
void OLED_output(float freq_val, float M_val, float iout_val, float iref_val, int tag_val, float uref_val, int flag_val);
void Rectifier_ForceOff(void);

void PLL1(float UI);
void PID1_Init(void);
void PID2_Init(void);
void PIDa_Init(void);
void PIDb_Init(void);
void PIDc_Init(void);
float PID1_Cal(float u);
float PID2_Cal(float u);
float PIDa_Cal(float u);
float PIDb_Cal(float u);
float PIDc_Cal(float u);
float PR_Control(float err, float *err1, float *err2, float *res1, float *res2);

void EPWM1_Init(void);
void EPWM2_Init(void);
void EPWM3_Init(void);
void EPWM4_Init(void);
void EPWM5_Init(void);
void EPWM6_Init(void);
void EPWM7_Init(void);
void EPWM8_Init(void);

//********** 按键宏定义 **********//
#define KEY_H1  (GpioDataRegs.GPADAT.bit.GPIO27)   // KEY1 → GPIO27
#define KEY_H2  (GpioDataRegs.GPADAT.bit.GPIO25)   // KEY2 → GPIO25
#define KEY_H3  (GpioDataRegs.GPADAT.bit.GPIO17)   // KEY3 → GPIO17
#define KEY_H4  (GpioDataRegs.GPADAT.bit.GPIO26)   // KEY4 → GPIO26
#define KEY_H5  (GpioDataRegs.GPADAT.bit.GPIO16)   // KEY5 → GPIO16
#define KEY_H6  (GpioDataRegs.GPBDAT.bit.GPIO39)   // KEY6 → GPIO39

#define KEY1_PRESS  1
#define KEY2_PRESS  2
#define KEY3_PRESS  3
#define KEY4_PRESS  4
#define KEY5_PRESS  5
#define KEY6_PRESS  6
#define KEY_UNPRESS 0

//********** 基础参数 **********//
#define EPWM_TIMER_TBPRD  2500                          // PWM 周期值，开关频率 = 100MHz/2/2500 = 20kHz
#define RECT_M_INIT        0.435f                        // 整流初始调制: 60V×0.435=26.1V, 匹配逆变26.13V
#define THIRD_HARMONIC_GAIN 0.16666667f                  // 三次谐波注入系数1/6, 提高直流电压利用率
#define OVERCURRENT_PROTECTION_ENABLED 1                 // 临时关闭软件过流保护: 恢复时改为1
#define OVERCURRENT_CONFIRM_COUNT 5                      // 连续5个20kHz采样过流后才触发保护
#define PI                 3.14159265f

float ADC_FREQ = 100000000.0f / EPWM_TIMER_TBPRD / 2.0f; // ADC 采样频率 = 20kHz

//********** PID 配置 **********//
typedef struct {
    float ref;
    float Xin;
    float Err;
    float Err_last;
    float Kp, Ki, Kd;
    float result;
    float result_last;
    float Integral;
} pidsettings;

pidsettings pid1;                                        // 电压外环 PID
pidsettings pid2;                                        // 备用
pidsettings pida;                                        // A 相电流内环 PI
pidsettings pidb;                                        // B 相电流内环 PI
pidsettings pidc;                                        // C 相电流内环 PI

//********** 系统参考值 **********//
float U_BUS_REF = 32.0f;                                 // 线电压目标值 (Vrms)
float I_PID_REF = 2.0f;                                  // 电流内环参考值 (A)

//********** ADC 采样值 **********//
Uint16 adc_Uab = 0;                                      // ADCA SOC0 → ADCINA0 线电压 Uab
Uint16 adc_Ubc = 0;                                      // ADCA SOC1 → ADCINA1 线电压 Ubc
Uint16 adc_Uout = 0;                                     // ADCA SOC2 → ADCINA2 输出电压
Uint16 adc_A3 = 0;                                       // ADCA SOC3 → ADCINA3 备用采样
Uint16 adc_A4 = 0;                                       // ADCA SOC4 → ADCINA4 备用采样

Uint16 adc_Ioa = 0;                                      // ADCB SOC0 → ADCINB0 A 相电流
Uint16 adc_Iob = 0;                                      // ADCB SOC1 → ADCINB1 B 相电流
Uint16 adc_Ioc = 0;                                      // ADCB SOC2 → ADCINB2 C 相电流

//********** 三相电压电流变量 **********//
float U_oa = 0, U_ob = 0, U_oc = 0;
float I_oa = 0, I_ob = 0, I_oc = 0;
float U_ab = 0, U_bc = 0;
float U_out = 0;
float middle_a = 0, middle_b = 0, middle_c = 0;
float theta_a = 0, theta_b = 0, theta_c = 0;
float Da = 0.4f, Db = 0.4f, Dc = 0.4f;
float U_av = 0, I_av = 0;

float U_bus_rms = 0;
float U_out_rms = 0;
float I_bus_rms = 0;
float pid_out = 2.0f;
float dbg_vcm_mod = 0.0f;                                // 三次谐波共模调制量: (mod_a+mod_b+mod_c)/3
//********** 模式控制 **********//
#define TAG_LIST_LEN 3
#define TAG2_FREQ_HZ    50.0f
#define TAG2_U_BUS_REF  32.0f
#define TAG2_I_PID_REF  2.0f
int tag = 0;
float i_ref_ramp = 0.0f;                                     // 闭环电流参考斜坡值
int rect_start_delay = 0;                                    // 整流启动延迟计数器
int rect_ctl_mode = 2;                                       // 0=开环, 1=PI闭环, 2=PR闭环
int rect_overcurrent_latch = 0;                              // 过流后闭锁同步整流, 逆变侧继续运行
int overcurrent_count = 0;                                   // 连续过流采样计数
int tag_last = 0;
int tag_num = 0;
int tag_list[TAG_LIST_LEN] = {0, 1, 2};                  // 关断→变频→回馈
int flag = 0;
int N = 200;
int N_c1 = 0;
float M = 2.2f;                                          // 调制比（反比：越大输出电压越小）
float freq = 50.0f;                                      // 当前输出频率

//********** 有效值累加 **********//
float sum1 = 0, sum2 = 0, sum3 = 0;

//********** PLL 锁相环变量 **********//
float W0 = 314.1593f;                                    // 目标角速度 50Hz
float W1 = 314.1593f;                                    // 实际角速度
float W1_last = 0;
float Ts = 0.00005f;                                     // 控制周期 1/20kHz
float U_alpha = 0, U_beta = 0;
float Ud = 0, Uq = 0;
float theta = 0;

float kp_pll = 100.0f;                                   // PLL PI 参数（参考 0612）
float ki_pll = 10.0f;
float err_pll = 0;
float err_pll_last = 0;
float result_pll_last = 0;
float result_pll = 0;

//********** PR 控制器变量 (三相独立状态) **********//
#define PR_KP      0.05f                                 // PR比例系数 (60V)
#define PR_KR      0.00030f                              // PR谐振系数 (994×0.0003≈0.30/A @50Hz)
#define PR_A1     -1.99875f                              // -2×r×cos(ω₀Ts), r=√PR_A2=0.9995
#define PR_A2      0.99900f                              // r²=0.999, 极点 |z|=0.9995<1 ✅

float pr_err1_a=0, pr_err2_a=0, pr_res1_a=0, pr_res2_a=0;
float pr_err1_b=0, pr_err2_b=0, pr_res1_b=0, pr_res2_b=0;
float pr_err1_c=0, pr_err2_c=0, pr_res1_c=0, pr_res2_c=0;
float pr_out_a=0, pr_out_b=0, pr_out_c=0;

//********** 按键变量 **********//
int N_key = 0;

//********** OLED 轮播 **********//
int oled_step = 0;

//********** 通用计数器 **********//
// (已清理未使用变量)

//********** Flash 运行相关 **********//
#ifdef _FLASH
extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsRunStart;
extern Uint16 RamfuncsLoadSize;
#endif


//******************* 主函数 *******************//
void main(void)
{
    #ifdef _FLASH
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);
    #endif

    InitSysCtrl();
    InitGpio();

    // F280049 GPIO 引脚默认锁定，须先解锁才能配置 MUX（F28069 无此限制）
    GPIO_unlockPortConfig(GPIO_PORT_A, 0xFFFFFFFF);
    GPIO_unlockPortConfig(GPIO_PORT_B, 0xFFFFFFFF);
    GPIO_unlockPortConfig(GPIO_PORT_H, 0xFFFFFFFF);

    EALLOW;
    // 前级同步整流（EPWM1/2/3）+ 后级 SPWM（EPWM4/5/6）
    GpioCtrlRegs.GPAGMUX1.bit.GPIO0 = 0;  GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;  // EPWM1A
    GpioCtrlRegs.GPAGMUX1.bit.GPIO1 = 0;  GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;  // EPWM1B
    GpioCtrlRegs.GPAGMUX1.bit.GPIO2 = 0;  GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;  // EPWM2A
    GpioCtrlRegs.GPAGMUX1.bit.GPIO3 = 0;  GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;  // EPWM2B
    GpioCtrlRegs.GPAGMUX1.bit.GPIO4 = 0;  GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;  // EPWM3A
    GpioCtrlRegs.GPAGMUX1.bit.GPIO5 = 0;  GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;  // EPWM3B
    GpioCtrlRegs.GPAGMUX1.bit.GPIO6 = 0;  GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;  // EPWM4A
    GpioCtrlRegs.GPAGMUX1.bit.GPIO7 = 0;  GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;  // EPWM4B
    GpioCtrlRegs.GPAGMUX1.bit.GPIO8 = 0;  GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1;  // EPWM5A
    GpioCtrlRegs.GPAGMUX1.bit.GPIO9 = 0;  GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 1;  // EPWM5B
    GpioCtrlRegs.GPAGMUX1.bit.GPIO10 = 0; GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1; // EPWM6A
    GpioCtrlRegs.GPAGMUX1.bit.GPIO11 = 0; GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1; // EPWM6B
    // EPWM7/8 未使用，GPIO12-15 保持默认 GPIO 状态
    AnalogSubsysRegs.DCDCCTL.bit.DCDCEN = 0;             // 禁用内部 DC-DC（LaunchPad 使用外部 LDO）
    EDIS;

    DINT;
    InitPieCtrl();
    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable();

    EALLOW;
    PieVectTable.ADCA1_INT = &adcA1ISR;
    EDIS;

    OLED_Init();
    Init_KEY();
    initADC();
    initEPWM();
    initADCSOC();

    PID1_Init();
    PID2_Init();
    PIDa_Init();
    PIDb_Init();
    PIDc_Init();

    OLED_ShowString(0, 0, "freq");
    OLED_ShowString(0, 1, "M");
    OLED_ShowString(0, 2, "Vcm");
    OLED_ShowString(0, 3, "TAG");

    IER |= M_INT1;
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;               // 统一释放所有 EPWM 时基，同步启动
    EDIS;

    EINT;
    ERTM;

//******************* 主循环 *******************//
    for(;;)
    {
        // 快照 ISR 共享变量（32 位浮点在 C28x 上非原子，需关中断读取）
        float snap_freq, snap_M, snap_ious, snap_iref, snap_uref;
        int snap_tag, snap_flag;
        DINT;
        snap_freq = freq;
        snap_M    = M;
        snap_ious = dbg_vcm_mod;
        snap_iref = I_PID_REF;
        snap_tag  = tag;
        snap_uref = U_BUS_REF;
        snap_flag = flag;
        EINT;
        OLED_output(snap_freq, snap_M, snap_ious, snap_iref, snap_tag, snap_uref, snap_flag);

        if (U_BUS_REF > 65.0f) U_BUS_REF = 65.0f;
        if (U_BUS_REF < 10.0f) U_BUS_REF = 10.0f;
        if (freq > 101.0f)     freq = 101.0f;
        if (freq < 19.0f)      freq = 19.0f;

        pid1.ref = U_BUS_REF / 1.73205f;
    }
}


//******************* ADC 中断服务程序（20kHz，控制核心）*******************//
__interrupt void adcA1ISR(void)
{
    int overcurrent_now = 0;

    // ===== Step 1: 读取 6 路 ADC 结果 =====
    adc_Uab  = AdcaResultRegs.ADCRESULT0;                // ADCA SOC0→A0, 线电压 Uab
    adc_Ubc  = AdcaResultRegs.ADCRESULT1;                // ADCA SOC1→A1, 线电压 Ubc
    adc_Uout = AdcaResultRegs.ADCRESULT2;                // ADCA SOC2→A2, 输出电压 U_out
    adc_A3   = AdcaResultRegs.ADCRESULT3;                // ADCA SOC3→A3, 备用
    adc_A4   = AdcaResultRegs.ADCRESULT4;                // ADCA SOC4→A4, 备用
    adc_Ioa  = AdcbResultRegs.ADCRESULT0;                // ADCB SOC0→B0, A相电流 I_oa
    adc_Iob  = AdcbResultRegs.ADCRESULT1;                // ADCB SOC1→B1, B相电流 I_ob
    adc_Ioc  = AdcbResultRegs.ADCRESULT2;                // ADCB SOC2→B2, C相电流 I_oc

    // ===== Step 2: ADC原始值→实际电压/电流 =====
    // 校准公式: ADC = k×实际值 + b → 实际值 = (ADC - b)/k
    U_ab  = ((float)adc_Uab  - 2043.6f) / 36.499f;      // A0: y=36.499x+2043.6 → 线电压Uab (V)
    U_bc  = ((float)adc_Ubc  - 2047.3f) / 36.323f;      // A1: y=36.323x+2047.3 → 线电压Ubc (V)
    U_out = ((float)adc_Uout - 2046.5f) / 36.167f;      // A2: y=36.167x+2046.5 → 输出电压U_out (V)
    // A3: y=36.391x+2042.4, 备用通道暂不转换
    // A4: 未标定, 原始值
    I_oa  = ((float)adc_Ioa  - 2050.2f) / 304.04f;      // B0: y=304.04x+2050.2 → A相电流 (A)
    I_ob  = ((float)adc_Iob  - 2032.1f) / 324.40f;      // B1: y=324.4x+2032.1  → B相电流 (A)
    I_oc  = ((float)adc_Ioc  - 2049.5f) / 327.40f;      // B2: y=327.4x+2049.5  → C相电流 (A)

    // ===== Step 3: 过流保护 (当前临时关闭; 恢复时打开OVERCURRENT_PROTECTION_ENABLED) =====
#if OVERCURRENT_PROTECTION_ENABLED
    overcurrent_now = ((fabsf(I_oa) >= 6.0f) || (fabsf(I_ob) >= 6.0f) || (fabsf(I_oc) >= 6.0f));
    if (overcurrent_now)
    {
        if (overcurrent_count < OVERCURRENT_CONFIRM_COUNT)
        {
            overcurrent_count++;
        }
    }
    else
    {
        overcurrent_count = 0;
    }

    if ((tag == 2) && (overcurrent_count >= OVERCURRENT_CONFIRM_COUNT))
    {
        tag = 1;                                         // 退回变频模式, 保持逆变侧运行
        tag_num = 1;                                     // 同步按键循环状态
        rect_overcurrent_latch = 1;                      // 闭锁同步整流, 避免电感电流硬截断
        flag = 1;                                        // 过流标志置位
        Rectifier_ForceOff();                            // 仅关闭EPWM1/2/3, 保持EPWM4/5/6运行
    }
#endif

    // ===== Step 4: 两路线电压→三相相电压重构 =====
    U_ob = (U_bc - U_ab) * 0.33333f;                    // U_b = (U_bc - U_ab)/3
    U_oa = U_ob + U_ab;                                  // U_a = U_b + U_ab
    U_oc = -(U_ob + U_oa);                               // U_c = -(U_a + U_b), 三相平衡

    // ===== Step 5: 三相有效值计算 (RMS=√(ΣU²/N)) =====
    U_av = sqrtf((U_oa * U_oa + U_ob * U_ob + U_oc * U_oc) * 0.33333f); // 相电压均方根值
    I_av = sqrtf((I_oa * I_oa + I_ob * I_ob + I_oc * I_oc) * 0.33333f); // 相电流均方根值

    // ===== Step 6: 模式切换→互补输出开关 (DBCTL受EALLOW保护) =====
    if (tag != tag_last)                                 // 仅在模式变化时执行
    {
        if ((tag == 2) && (tag_last == 1) && (overcurrent_now == 0)) // 变频→回馈: 开启前级EPWM1/2/3互补
        {
            float rect_theta_b = theta_a - 4.0f * PI * 0.33333f;
            float rect_theta_c = theta_a + 4.0f * PI * 0.33333f;
            if (rect_theta_b < 0.0f) rect_theta_b += 2.0f * PI;
            if (rect_theta_c >= 2.0f * PI) rect_theta_c -= 2.0f * PI;

            {
                float rect_sin_3 = sinf(3.0f * theta_a);
                float rect_mod_a = sinf(theta_a) + THIRD_HARMONIC_GAIN * rect_sin_3;
                float rect_mod_b = sinf(rect_theta_b) + THIRD_HARMONIC_GAIN * rect_sin_3;
                float rect_mod_c = sinf(rect_theta_c) + THIRD_HARMONIC_GAIN * rect_sin_3;
                Da = 0.5f + RECT_M_INIT * rect_mod_a;
                Db = 0.5f + RECT_M_INIT * rect_mod_b;
                Dc = 0.5f + RECT_M_INIT * rect_mod_c;
            }
            if (Da > 0.98f) Da = 0.98f;
            if (Da < 0.02f) Da = 0.02f;
            if (Db > 0.98f) Db = 0.98f;
            if (Db < 0.02f) Db = 0.02f;
            if (Dc > 0.98f) Dc = 0.98f;
            if (Dc < 0.02f) Dc = 0.02f;

            // 先按当前逆变相位预装整流侧占空比, 再释放强制输出
            EALLOW;
            EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_IMMEDIATE;
            EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_IMMEDIATE;
            EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_IMMEDIATE;
            EDIS;
            EPwm1Regs.CMPA.bit.CMPA = (Uint16)((float)EPWM_TIMER_TBPRD * Da);
            EPwm2Regs.CMPA.bit.CMPA = (Uint16)((float)EPWM_TIMER_TBPRD * Db);
            EPwm3Regs.CMPA.bit.CMPA = (Uint16)((float)EPWM_TIMER_TBPRD * Dc);
            { volatile Uint16 _d = EPwm3Regs.CMPA.bit.CMPA; (void)_d; } // 读回屏障
            EALLOW;
            EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
            EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
            EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
            EPwm1Regs.DBCTL.bit.POLSEL = 2;              // EPWM1 A/B互补输出 (AHC模式)
            EPwm2Regs.DBCTL.bit.POLSEL = 2;
            EPwm3Regs.DBCTL.bit.POLSEL = 2;
            EDIS;
            // CMPA=1250已生效, 安全释放整流侧强制
            EPwm1Regs.AQCSFRC.bit.CSFA = 0;
            EPwm1Regs.AQCSFRC.bit.CSFB = 0;
            EPwm2Regs.AQCSFRC.bit.CSFA = 0;
            EPwm2Regs.AQCSFRC.bit.CSFB = 0;
            EPwm3Regs.AQCSFRC.bit.CSFA = 0;
            EPwm3Regs.AQCSFRC.bit.CSFB = 0;
            pida.Integral = 0.0f;
            pidb.Integral = 0.0f;
            pidc.Integral = 0.0f;
            // 清零PR状态
            pr_err1_a=0; pr_err2_a=0; pr_res1_a=0; pr_res2_a=0;
            pr_err1_b=0; pr_err2_b=0; pr_res1_b=0; pr_res2_b=0;
            pr_err1_c=0; pr_err2_c=0; pr_res1_c=0; pr_res2_c=0;
            i_ref_ramp = 0.1f;
        }
        if (tag == 1)                                    // →变频: 开启后级EPWM4/5/6互补
        {
            // 先设逆变侧CMPA安全值→立即生效, 再释放强制 (同回馈模式防毛刺逻辑)
            EALLOW;
            EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_IMMEDIATE;
            EPwm5Regs.CMPCTL.bit.SHDWAMODE = CC_IMMEDIATE;
            EPwm6Regs.CMPCTL.bit.SHDWAMODE = CC_IMMEDIATE;
            EDIS;
            EPwm4Regs.CMPA.bit.CMPA = (EPWM_TIMER_TBPRD >> 1); // 50%占空比安全值
            EPwm5Regs.CMPA.bit.CMPA = (EPWM_TIMER_TBPRD >> 1);
            EPwm6Regs.CMPA.bit.CMPA = (EPWM_TIMER_TBPRD >> 1);
            { volatile Uint16 _d = EPwm6Regs.CMPA.bit.CMPA; (void)_d; } // 读回屏障
            EALLOW;
            EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;  // 恢复影子模式
            EPwm5Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
            EPwm6Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
            EPwm4Regs.DBCTL.bit.POLSEL = 2;              // EPWM4 A/B互补输出 (AHC模式)
            EPwm5Regs.DBCTL.bit.POLSEL = 2;
            EPwm6Regs.DBCTL.bit.POLSEL = 2;
            EDIS;
            // CMPA=1250已生效, 安全释放逆变侧强制
            EPwm4Regs.AQCSFRC.bit.CSFA = 0;
            EPwm4Regs.AQCSFRC.bit.CSFB = 0;
            EPwm5Regs.AQCSFRC.bit.CSFA = 0;
            EPwm5Regs.AQCSFRC.bit.CSFB = 0;
            EPwm6Regs.AQCSFRC.bit.CSFA = 0;
            EPwm6Regs.AQCSFRC.bit.CSFB = 0;
            Rectifier_ForceOff();                        // 整流侧强制输出全低: 仅体二极管被动整流
            M = 8.0f;                                    // 软启动: M最大→输出电压最低, PID逐步下调
        }
        // 模式切换时设定默认参数（仅执行一次，后续由按键调整）
        if (tag == 1)
        {
            N = (int)(ADC_FREQ / freq + 0.5f);           // 每周期采样点数
            U_BUS_REF = 32.0f;                            // 默认线电压目标32V
            if (overcurrent_now == 0)
            {
                rect_overcurrent_latch = 0;
                flag = 0;                                // 正常切换, 清除过流标志
            }
        }
        else if (tag == 2)
        {
            freq = TAG2_FREQ_HZ;                         // 回馈模式固定50Hz
            N = (int)(ADC_FREQ / TAG2_FREQ_HZ + 0.5f);    // 50Hz完整周期
            U_BUS_REF = TAG2_U_BUS_REF;                  // 回馈模式固定线电压32V
            I_PID_REF = TAG2_I_PID_REF;                  // 回馈模式固定线电流2A
            pid1.Integral = 0.0f;
            sum1 = 0.0f;
            sum2 = 0.0f;
            sum3 = 0.0f;
            N_c1 = 0;
            if (overcurrent_now == 0)
            {
                rect_overcurrent_latch = 0;
                flag = 0;
            }
        }
        // 注意: 过流时flag不清零, OLED可显示过流状态
    }
    tag_last = tag;                                      // 更新上一次模式状态

    // ===== Step 7: 变频模式下动态调整RMS窗口大小 =====
    if (tag == 1)
    {
        N = (int)(ADC_FREQ / freq + 0.5f);               // freq变化时N跟随更新
    }

    // ===== Step 8: 运行状态处理 =====
    if (tag != 0)                                        // 非关断模式
    {
        // DDS浮点相位累加: 每ISR执行, 确保精确频率 (Δθ = 2π×freq/ADC_FREQ)
        theta_a += 2.0f * PI * freq / ADC_FREQ;                     // A相角度累加
        if (theta_a >= 2.0f * PI)  theta_a -= 2.0f * PI;           // 归一到[0, 2π)
        theta_b = theta_a - 4.0f * PI * 0.33333f;                  // B相滞后120°
        if (theta_b < 0.0f) theta_b += 2.0f * PI;
        theta_c = theta_a + 4.0f * PI * 0.33333f;                  // C相超前120°
        if (theta_c >= 2.0f * PI) theta_c -= 2.0f * PI;

        // 缓存三相正弦值, 避免重复调用 sinf()
        float sin_a = sinf(theta_a);
        float sin_b = sinf(theta_b);
        float sin_c = sinf(theta_c);
        float sin_3 = sinf(3.0f * theta_a);
        float mod_a = sin_a + THIRD_HARMONIC_GAIN * sin_3;
        float mod_b = sin_b + THIRD_HARMONIC_GAIN * sin_3;
        float mod_c = sin_c + THIRD_HARMONIC_GAIN * sin_3;
        dbg_vcm_mod = (mod_a + mod_b + mod_c) * 0.33333f;

        // 后级SPWM: 每ISR更新占空比 → 变流器1 DC-AC
        EPwm4Regs.CMPA.bit.CMPA = (Uint16)((float)EPWM_TIMER_TBPRD * mod_a / M + (float)EPWM_TIMER_TBPRD * 0.5f);
        EPwm5Regs.CMPA.bit.CMPA = (Uint16)((float)EPWM_TIMER_TBPRD * mod_b / M + (float)EPWM_TIMER_TBPRD * 0.5f);
        EPwm6Regs.CMPA.bit.CMPA = (Uint16)((float)EPWM_TIMER_TBPRD * mod_c / M + (float)EPWM_TIMER_TBPRD * 0.5f);

        // tag=2: 能量回馈—前级EPWM1/2/3整流
        if ((tag == 2) && (rect_overcurrent_latch == 0))
        {
            // 电流参考软启动: 从实测值斜坡过渡到目标值, 避免PI积分冲击
            if (i_ref_ramp > I_PID_REF) {
                i_ref_ramp -= 0.0005f;                   // 慢速下降 (0.0005×20kHz=10A/s, ~0.2s从3A→1A)
            } else if (i_ref_ramp < I_PID_REF) {
                i_ref_ramp += 0.0005f;                   // 慢速上升 (如需增大电流)
            } else {
                i_ref_ramp = I_PID_REF;
            }
            pid_out = i_ref_ramp * 1.4142f;              // RMS→峰值转换
            if (pid_out > 5.6f)  pid_out = 5.6f;
            if (pid_out < -5.6f) pid_out = -5.6f;

            pida.ref = pid_out * sin_a;
            pidb.ref = pid_out * sin_b;
            pidc.ref = pid_out * sin_c;

            if (rect_ctl_mode == 0) {
                Da = 0.5f + RECT_M_INIT * mod_a;
                Db = 0.5f + RECT_M_INIT * mod_b;
                Dc = 0.5f + RECT_M_INIT * mod_c;
            } else if (rect_ctl_mode == 1) {
                middle_a = PIDa_Cal(I_oa);
                middle_b = PIDb_Cal(I_ob);
                middle_c = PIDc_Cal(I_oc);
                Da = 0.5f + RECT_M_INIT * mod_a + middle_a * 0.015f;
                Db = 0.5f + RECT_M_INIT * mod_b + middle_b * 0.015f;
                Dc = 0.5f + RECT_M_INIT * mod_c + middle_c * 0.015f;
            } else {
                pr_out_a = PR_Control(I_oa - pida.ref, &pr_err1_a, &pr_err2_a, &pr_res1_a, &pr_res2_a);
                pr_out_b = PR_Control(I_ob - pidb.ref, &pr_err1_b, &pr_err2_b, &pr_res1_b, &pr_res2_b);
                pr_out_c = PR_Control(I_oc - pidc.ref, &pr_err1_c, &pr_err2_c, &pr_res1_c, &pr_res2_c);
                Da = 0.5f + RECT_M_INIT * mod_a + pr_out_a;
                Db = 0.5f + RECT_M_INIT * mod_b + pr_out_b;
                Dc = 0.5f + RECT_M_INIT * mod_c + pr_out_c;
            }

            if (Da > 0.98f) Da = 0.98f;
            if (Da < 0.02f) Da = 0.02f;
            if (Db > 0.98f) Db = 0.98f;
            if (Db < 0.02f) Db = 0.02f;
            if (Dc > 0.98f) Dc = 0.98f;
            if (Dc < 0.02f) Dc = 0.02f;

            EPwm1Regs.CMPA.bit.CMPA = (Uint16)((float)EPWM_TIMER_TBPRD * Da);
            EPwm2Regs.CMPA.bit.CMPA = (Uint16)((float)EPWM_TIMER_TBPRD * Db);
            EPwm3Regs.CMPA.bit.CMPA = (Uint16)((float)EPWM_TIMER_TBPRD * Dc);
        }

        // RMS有效值窗口累加 (每N个采样计算一次)
        if (N_c1 < N)
        {
            N_c1++;                                      // 采样计数器递增
            sum1 += U_av * U_av;                         // 累加U_av²
            sum2 += U_out * U_out;                       // 累加U_out²
            sum3 += I_av * I_av;                         // 累加I_av²
        }
        else                                             // 一个完整窗口采集完毕
        {
            N_c1 = 0;                                    // 重置采样计数器
            U_bus_rms  = sqrtf(sum1 / (float)N);         // 相电压有效值 = √(ΣU²/N)
            sum1 = 0;                                    // 清零累加器
            U_out_rms  = sqrtf(sum2 / (float)N);         // 输出电压有效值
            sum2 = 0;
            I_bus_rms  = sqrtf(sum3 / (float)N);         // 相电流有效值
            sum3 = 0;

            // 电压外环PID: 仅在有输出电压时调节 (防功率电未上时M误调到底)
            if (U_bus_rms > 0.5f) {
                M += PID1_Cal(U_bus_rms);                // M反比于输出电压: M↑→电压↓
            } else {
                pid1.Integral = 0.0f;                    // 无输出时清零积分, 保持M=8.0软启动
            }
            if (M >= 8.0f)  M = 8.0f;                    // M上限
            if (M <= 1.50f) M = 1.50f;                   // M下限
        }
    }
    else                                                 // tag=0: 关断模式
    {
        EALLOW;
        EPwm1Regs.DBCTL.bit.POLSEL = 0;                  // 禁止EPWM1互补→A/B独立输出
        EPwm2Regs.DBCTL.bit.POLSEL = 0;                  // 禁止EPWM2互补
        EPwm3Regs.DBCTL.bit.POLSEL = 0;                  // 禁止EPWM3互补
        EPwm4Regs.DBCTL.bit.POLSEL = 0;                  // 禁止EPWM4互补
        EPwm5Regs.DBCTL.bit.POLSEL = 0;                  // 禁止EPWM5互补
        EPwm6Regs.DBCTL.bit.POLSEL = 0;                  // 禁止EPWM6互补
        EDIS;
        EPwm1Regs.CMPA.bit.CMPA = 0;                     // EPWM1占空比归零
        EPwm2Regs.CMPA.bit.CMPA = 0;                     // EPWM2占空比归零
        EPwm3Regs.CMPA.bit.CMPA = 0;                     // EPWM3占空比归零
        EPwm4Regs.CMPA.bit.CMPA = 0;                     // EPWM4占空比归零
        EPwm5Regs.CMPA.bit.CMPA = 0;                     // EPWM5占空比归零
        EPwm6Regs.CMPA.bit.CMPA = 0;                     // EPWM6占空比归零
        // 强制所有输出全低, 防止CMPA=0+POLSEL=0时上管意外导通
        EPwm1Regs.AQCSFRC.bit.CSFA = 1;
        EPwm1Regs.AQCSFRC.bit.CSFB = 1;
        EPwm2Regs.AQCSFRC.bit.CSFA = 1;
        EPwm2Regs.AQCSFRC.bit.CSFB = 1;
        EPwm3Regs.AQCSFRC.bit.CSFA = 1;
        EPwm3Regs.AQCSFRC.bit.CSFB = 1;
        EPwm4Regs.AQCSFRC.bit.CSFA = 1;
        EPwm4Regs.AQCSFRC.bit.CSFB = 1;
        EPwm5Regs.AQCSFRC.bit.CSFA = 1;
        EPwm5Regs.AQCSFRC.bit.CSFB = 1;
        EPwm6Regs.AQCSFRC.bit.CSFA = 1;
        EPwm6Regs.AQCSFRC.bit.CSFB = 1;
        sum1 = 0;                                        // 清零电压累加器
        sum2 = 0;
        sum3 = 0;                                        // 清零电流累加器
        N_c1 = 0;                                        // 重置采样计数
    }

    // ===== Step 9: 按键扫描 (每10000次ISR≈0.5s, 自然消抖) =====
    N_key++;
    if (N_key >= 10000)
    {
        N_key = 0;
        KEY_Control(KEY_Scan(0));
    }

    // ===== Step 10: 清除ADC中断标志, 应答PIE =====
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;               // 清除ADCA INT1中断标志
    if (1 == AdcaRegs.ADCINTOVF.bit.ADCINT1)              // 检查ADC溢出 (ISR执行超时)
    {
        AdcaRegs.ADCINTOVFCLR.bit.ADCINT1 = 1;            // 清除溢出标志
        AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;            // 再次清除中断标志
    }
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;               // 应答PIE第1组, 允许下次中断
}


//******************* 整流侧强制关断 *******************//
void Rectifier_ForceOff(void)
{
    EALLOW;
    EPwm1Regs.DBCTL.bit.POLSEL = 0;                      // 禁止EPWM1互补→A/B独立输出
    EPwm2Regs.DBCTL.bit.POLSEL = 0;                      // 禁止EPWM2互补
    EPwm3Regs.DBCTL.bit.POLSEL = 0;                      // 禁止EPWM3互补
    EDIS;

    EPwm1Regs.CMPA.bit.CMPA = 0;                         // 整流侧占空比归零
    EPwm2Regs.CMPA.bit.CMPA = 0;
    EPwm3Regs.CMPA.bit.CMPA = 0;

    EPwm1Regs.AQCSFRC.bit.CSFA = 1;                      // EPWM1/2/3强制全低
    EPwm1Regs.AQCSFRC.bit.CSFB = 1;
    EPwm2Regs.AQCSFRC.bit.CSFA = 1;
    EPwm2Regs.AQCSFRC.bit.CSFB = 1;
    EPwm3Regs.AQCSFRC.bit.CSFA = 1;
    EPwm3Regs.AQCSFRC.bit.CSFB = 1;
}


//******************* EPWM1 中断（备用）*******************//
__interrupt void epwm1_isr(void)
{
    EPwm1Regs.ETCLR.bit.INT = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}


//******************* OLED 轮播显示 *******************//
void OLED_output(float freq_val, float M_val, float iout_val, float iref_val, int tag_val, float uref_val, int flag_val)
{
    switch (oled_step)
    {
        case 0:
            OLED_ShowFloat(5, 0, freq_val, 3);
            OLED_ShowFloat(11,0, uref_val, 3);
            break;
        case 1:
            OLED_ShowFloat(2, 1, M_val, 4);
            break;
        case 2:
            OLED_ShowFloat(4, 2, iout_val, 3);
            break;
        case 3:
            OLED_ShowNum(4, 3, tag_val, 1);
            OLED_ShowNum(14, 3, flag_val, 1);
            break;
    }
    oled_step = (oled_step + 1) % 4;
}


//******************* PLL 锁相环（三相 Clarke + 0612 PI 结构 + 频率限幅）*******************//
void PLL1(float UI)
{
    // ===== Clarke 变换: 三相(abc) → 两相静止(αβ) =====
    U_alpha = 0.66667f * (U_oa - 0.5f * U_ob - 0.5f * U_oc);   // α轴 = 2/3×(Ua - Ub/2 - Uc/2)
    U_beta  = 0.66667f * (-0.86603f * U_ob + 0.86603f * U_oc); // β轴 = 2/3×(√3/2×Uc - √3/2×Ub)

    // ===== Park 变换: 两相静止(αβ) → 两相旋转(dq) =====
    Ud =  cosf(theta) * U_alpha + sinf(theta) * U_beta;          // d轴分量 (与电网电压矢量同相)
    Uq = -sinf(theta) * U_alpha + cosf(theta) * U_beta;          // q轴分量 (电网电压矢量正交)

    // ===== PI 锁相: 锁定Uq=0, 使d轴对齐电网电压矢量 =====
    err_pll = 0.0f - Uq;                                         // 误差 = 目标(0) - 实际(Uq)
    result_pll = kp_pll * err_pll + ki_pll * (err_pll - err_pll_last); // PI: P×err + D×(err变化)
    err_pll_last = err_pll;                                      // 保存本次误差→下次用

    // ===== 频率更新+限幅: 防止锁相环跑飞 =====
    W1 -= result_pll;                                            // 角频率修正 (PI输出负反馈)
    if (W1 >= 115.0f * PI)  W1 = 115.0f * PI;                    // 频率上限 ≈57.5Hz
    if (W1 <= 85.0f * PI)   W1 = 85.0f * PI;                     // 频率下限 ≈42.5Hz

    // ===== 相位积分: θ = ∫ω·dt = Σω·Ts =====
    theta += W1 * Ts;                                            // 相位累加 (欧拉积分)
    if (theta >= 2.0f * PI)                                      // 相位归一到 [0, 2π)
        theta -= 2.0f * PI;
    else if (theta <= 0.0f)
        theta += 2.0f * PI;
}


//******************* PID1 — 电压外环（条件积分 + 抗饱和 + 死区）*******************//
void PID1_Init(void)
{
    pid1.Xin = 0;                                            // 输入清零
    pid1.ref = 0;                                            // 参考清零
    pid1.Err = 0;                                            // 当前误差清零
    pid1.Err_last = 0;                                       // 上一次误差清零
    pid1.Kp = 0.01f;                                         // 比例系数
    pid1.Ki = 0.001f;                                        // 积分系数 (Kp/10, 慢速消除静差)
    pid1.Kd = 0;                                             // 微分系数 (未使用)
    pid1.result = 0;                                         // 输出清零
    pid1.result_last = 0;                                    // 上一次输出清零
    pid1.Integral = 0;                                       // 积分累加器清零
}

float PID1_Cal(float u)
{
    pid1.Xin = u;                                            // 采样输入 (相电压有效值)
    pid1.Err = pid1.Xin - pid1.ref;                          // 误差 = 实际值 - 参考值

    // 积分抗饱和: 先钳位再积分
    if (pid1.Integral > 15.0f)                               // 积分上限
        pid1.Integral = 15.0f;
    else if (pid1.Integral < -15.0f)                         // 积分下限
        pid1.Integral = -15.0f;

    // 条件积分: 误差在带内才积分, 带外衰减
    if (fabsf(pid1.Err) < 0.2f) {                            // 误差<0.2V: 正常积分
        pid1.Integral += pid1.Err;                           // I += Err×Ts (Ts隐含在Ki中)
        pid1.Integral = (pid1.Integral > 15.0f) ? 15.0f : pid1.Integral;   // 上钳位
        pid1.Integral = (pid1.Integral < -15.0f) ? -15.0f : pid1.Integral; // 下钳位
    } else {                                                 // 误差≥0.2V: 积分衰减
        pid1.Integral *= 0.9f;                               // 每次衰减10%, 防止windup
    }

    pid1.result = pid1.Kp * pid1.Err                         // 比例项: Kp × Err
                + pid1.Ki * pid1.Integral                    // 积分项: Ki × Integral
                + pid1.Kd * (pid1.Err - pid1.Err_last);      // 微分项: Kd × ΔErr
    pid1.Err_last = pid1.Err;                                // 保存本次误差

    // 死区: 误差<0.02V不输出, 避免稳态抖动
    if (fabsf(pid1.Err) > 0.02f)
        return pid1.result;                                  // 正常输出PID结果
    else
        return 0.0f;                                         // 死区内输出0
}


//******************* PID2 — 备用电压环 *******************//
void PID2_Init(void)
{
    pid2.Xin = 0;
    pid2.ref = 0;
    pid2.Err = 0;
    pid2.Err_last = 0;
    pid2.Kp = 0;
    pid2.Ki = 0.0003f;
    pid2.Kd = 0;
    pid2.result = 0;
    pid2.result_last = 0;
    pid2.Integral = 0;
}

float PID2_Cal(float u)
{
    pid2.Xin = u;
    pid2.Err = pid2.Xin - pid2.ref;
    pid2.result = pid2.Kp * (pid2.Err - pid2.Err_last) + pid2.Ki * pid2.Err;
    pid2.Err_last = pid2.Err;
    pid2.result_last = pid2.result;
    return pid2.result;
}


//******************* PIDa — A 相电流内环 PI（条件积分 + 抗饱和 + 死区）*******************//
void PIDa_Init(void)
{
    pida.Xin = 0;
    pida.ref = 0;
    pida.Err = 0;
    pida.Err_last = 0;
    pida.Kp = 12.0f;                                         // 比例系数（1A误差→15%占空比, 提升跟踪精度）
    pida.Ki = 0.05f;                                        // 积分系数（折中, 避免过冲）
    pida.Kd = 0;
    pida.result = 0;
    pida.result_last = 0;
    pida.Integral = 0;
}

float PIDa_Cal(float u)
{
    pida.Xin = u;
    pida.Err = pida.Xin - pida.ref;

    // 正弦跟踪不适用条件积分: 正负半周误差自然抵消, 直接积分+钳位即可
    pida.Integral += pida.Err;
    if (pida.Integral > 50.0f)   pida.Integral = 50.0f;
    if (pida.Integral < -50.0f)  pida.Integral = -50.0f;

    pida.result = pida.Kp * pida.Err
                + pida.Ki * pida.Integral
                + pida.Kd * (pida.Err - pida.Err_last);
    pida.Err_last = pida.Err;
    pida.result_last = pida.result;

    // 死区: 误差<0.02A不输出, 避免稳态PWM抖动
    if (fabsf(pida.Err) > 0.02f)
        return pida.result;
    else
        return 0.0f;
}


//******************* PIDb — B 相电流内环 PI（条件积分 + 抗饱和 + 死区）*******************//
void PIDb_Init(void)
{
    pidb.Xin = 0;
    pidb.ref = 0;
    pidb.Err = 0;
    pidb.Err_last = 0;
    pidb.Kp = 12.0f;
    pidb.Ki = 0.05f;
    pidb.Kd = 0;
    pidb.result = 0;
    pidb.result_last = 0;
    pidb.Integral = 0;
}

float PIDb_Cal(float u)
{
    pidb.Xin = u;
    pidb.Err = pidb.Xin - pidb.ref;

    pidb.Integral += pidb.Err;
    if (pidb.Integral > 50.0f)   pidb.Integral = 50.0f;
    if (pidb.Integral < -50.0f)  pidb.Integral = -50.0f;

    pidb.result = pidb.Kp * pidb.Err
                + pidb.Ki * pidb.Integral
                + pidb.Kd * (pidb.Err - pidb.Err_last);
    pidb.Err_last = pidb.Err;
    pidb.result_last = pidb.result;

    if (fabsf(pidb.Err) > 0.02f)
        return pidb.result;
    else
        return 0.0f;
}


//******************* PIDc — C 相电流内环 PI（条件积分 + 抗饱和 + 死区）*******************//
void PIDc_Init(void)
{
    pidc.Xin = 0;
    pidc.ref = 0;
    pidc.Err = 0;
    pidc.Err_last = 0;
    pidc.Kp = 12.0f;
    pidc.Ki = 0.05f;
    pidc.Kd = 0;
    pidc.result = 0;
    pidc.result_last = 0;
    pidc.Integral = 0;
}

float PIDc_Cal(float u)
{
    pidc.Xin = u;
    pidc.Err = pidc.Xin - pidc.ref;

    pidc.Integral += pidc.Err;
    if (pidc.Integral > 50.0f)   pidc.Integral = 50.0f;
    if (pidc.Integral < -50.0f)  pidc.Integral = -50.0f;

    pidc.result = pidc.Kp * pidc.Err
                + pidc.Ki * pidc.Integral
                + pidc.Kd * (pidc.Err - pidc.Err_last);
    pidc.Err_last = pidc.Err;
    pidc.result_last = pidc.result;

    if (fabsf(pidc.Err) > 0.02f)
        return pidc.result;
    else
        return 0.0f;
}


//******************* PR 控制器（保留，当前未使用）*******************//
//******************* PR 谐振控制器 (三相独立, 50Hz) *******************//
// 差分方程: res(k) = err(k) - err(k-2) - PR_A1×res(k-1) - PR_A2×res(k-2)
//   输出:   pr  = PR_KP×err(k) + PR_KR×res(k)
float PR_Control(float err, float *err1, float *err2, float *res1, float *res2)
{
    float res0 = err - *err2 - PR_A1 * (*res1) - PR_A2 * (*res2);

    // 谐振项限幅, 防数值发散
    // 谐振器自带衰减(|z|=0.9995<1), Da钳位兜底, 不做额外限幅

    float output = PR_KP * err + PR_KR * res0;

    // 移位: k-2 ← k-1 ← k
    *err2 = *err1;
    *err1 = err;
    *res2 = *res1;
    *res1 = res0;

    return output;
}


//******************* 按键控制 *******************//
void KEY_Control(int key)
{
    if (tag == 2)                                        // 发挥部分: 能量回馈
    {
        switch (key)
        {
            case KEY1_PRESS:
                I_PID_REF += 0.01f;
                if (I_PID_REF > 4.0f) I_PID_REF = 4.0f;
                break;
            case KEY2_PRESS:
                I_PID_REF -= 0.01f;
                if (I_PID_REF < 0.1f) I_PID_REF = 0.1f;
                break;
            case KEY3_PRESS:
                tag_num = (tag_num + 1) % TAG_LIST_LEN;
                tag = tag_list[tag_num];
                break;
            case KEY4_PRESS:
                U_BUS_REF = 32.0f;
                I_PID_REF = TAG2_I_PID_REF;
                break;
            case KEY5_PRESS:
                U_BUS_REF += 0.1f;
                break;
            case KEY6_PRESS:
                U_BUS_REF -= 0.1f;
                break;
            default: break;
        }
    }
    else                                                 // 基础部分: 关断/变频
    {
        switch (key)
        {
            case KEY1_PRESS:  freq += 1.0f;   break;
            case KEY2_PRESS:  freq -= 1.0f;   break;
            case KEY3_PRESS:
                tag_num = (tag_num + 1) % TAG_LIST_LEN;
                tag = tag_list[tag_num];
                break;
            case KEY4_PRESS:
                freq = 50.0f;
                U_BUS_REF = 32.0f;
                break;
            case KEY5_PRESS:  U_BUS_REF += 0.1f; break;
            case KEY6_PRESS:  U_BUS_REF -= 0.1f; break;
            default: break;
        }
    }
}


//******************* 按键扫描 *******************//
char KEY_Scan(char key_mode)
{
    if ((KEY_H1 == 0) || (KEY_H2 == 0) || (KEY_H3 == 0) ||
        (KEY_H4 == 0) || (KEY_H5 == 0) || (KEY_H6 == 0))
    {
        if (KEY_H1 == 0) return KEY1_PRESS;
        if (KEY_H2 == 0) return KEY2_PRESS;
        if (KEY_H3 == 0) return KEY3_PRESS;
        if (KEY_H4 == 0) return KEY4_PRESS;
        if (KEY_H5 == 0) return KEY5_PRESS;
        if (KEY_H6 == 0) return KEY6_PRESS;
    }
    return KEY_UNPRESS;
}


//******************* 按键初始化 *******************//
void Init_KEY(void)
{
    EALLOW;

    GPIO_setPadConfig(27, GPIO_PIN_TYPE_STD);            // GPIO27 — KEY1
    GPIO_setDirectionMode(27, GPIO_DIR_MODE_IN);
    GpioCtrlRegs.GPAPUD.bit.GPIO27 = 0;

    GPIO_setPadConfig(25, GPIO_PIN_TYPE_STD);            // GPIO25 — KEY2
    GPIO_setDirectionMode(25, GPIO_DIR_MODE_IN);
    GpioCtrlRegs.GPAPUD.bit.GPIO25 = 0;

    GPIO_setPadConfig(17, GPIO_PIN_TYPE_STD);            // GPIO17 — KEY3
    GPIO_setDirectionMode(17, GPIO_DIR_MODE_IN);
    GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0;

    GPIO_setPadConfig(26, GPIO_PIN_TYPE_STD);            // GPIO26 — KEY4
    GPIO_setDirectionMode(26, GPIO_DIR_MODE_IN);
    GpioCtrlRegs.GPAPUD.bit.GPIO26 = 0;

    GPIO_setAnalogMode(16, GPIO_ANALOG_DISABLED);        // GPIO16 — KEY5, 禁用模拟
    GPIO_setPadConfig(16, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(16, GPIO_DIR_MODE_IN);
    GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0;

    GPIO_setAnalogMode(39, GPIO_ANALOG_DISABLED);        // GPIO39 — KEY6, 禁用模拟
    GPIO_setPadConfig(39, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(39, GPIO_DIR_MODE_IN);
    GpioCtrlRegs.GPBPUD.bit.GPIO39 = 0;                  // GPIO39 在 GPIOB 组

    EDIS;
}


//******************* ADC 初始化 *******************//
void initADC(void)
{
    ADC_setVREF(ADCA_BASE, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
    EALLOW;
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6;                  // 200MHz/(2×6) ≈ 16.67MHz ADCCLK
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    EDIS;
    DELAY_US(1000);

    ADC_setVREF(ADCB_BASE, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
    EALLOW;
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6;
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    EDIS;
    DELAY_US(1000);

    ADC_setVREF(ADCC_BASE, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
    EALLOW;
    AdccRegs.ADCCTL2.bit.PRESCALE = 6;
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    EDIS;
    DELAY_US(1000);
}


//******************* ADC SOC 配置 *******************//
void initADCSOC(void)
{
    EALLOW;

    // ADCA SOC 配置
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0;                  // SOC0 → A0 (U_ab)
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 9;
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 9;                // 触发源: EPWM3 SOCA

    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 1;                  // SOC1 → A1 (U_bc)
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 9;
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 9;

    AdcaRegs.ADCSOC2CTL.bit.CHSEL = 2;                  // SOC2 → A2 (U_out)
    AdcaRegs.ADCSOC2CTL.bit.ACQPS = 9;
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 9;

    AdcaRegs.ADCSOC3CTL.bit.CHSEL = 3;                  // SOC3 → A3 (备用)
    AdcaRegs.ADCSOC3CTL.bit.ACQPS = 9;
    AdcaRegs.ADCSOC3CTL.bit.TRIGSEL = 9;

    AdcaRegs.ADCSOC4CTL.bit.CHSEL = 4;                  // SOC4 → A4 (备用)
    AdcaRegs.ADCSOC4CTL.bit.ACQPS = 9;
    AdcaRegs.ADCSOC4CTL.bit.TRIGSEL = 9;

    // ADCB SOC 配置
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 0;                  // SOC0 → B0 (I_oa)
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = 9;
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 9;

    AdcbRegs.ADCSOC1CTL.bit.CHSEL = 1;                  // SOC1 → B1 (I_ob)
    AdcbRegs.ADCSOC1CTL.bit.ACQPS = 9;
    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = 9;

    AdcbRegs.ADCSOC2CTL.bit.CHSEL = 2;                  // SOC2 → B2 (I_oc)
    AdcbRegs.ADCSOC2CTL.bit.ACQPS = 9;
    AdcbRegs.ADCSOC2CTL.bit.TRIGSEL = 9;

    // ADCA INT1 中断：SOC4 完成时触发（最后一个 ADCA SOC）
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 4;
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

    EDIS;
}


//******************* EPWM 总初始化 *******************//
void initEPWM(void)
{
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;               // 冻结所有 EPWM 时基
    EDIS;

    EPWM1_Init();
    EPWM2_Init();
    EPWM3_Init();
    EPWM4_Init();
    EPWM5_Init();
    EPWM6_Init();
    // EPWM7/8 未使用

    // 上电默认强制所有输出全低, 防止 CMPA=0+POLSEL=2 导致上管意外导通
    // 后续由 tag 状态机按需释放
    EPwm1Regs.AQCSFRC.bit.CSFA = 1;
    EPwm1Regs.AQCSFRC.bit.CSFB = 1;
    EPwm2Regs.AQCSFRC.bit.CSFA = 1;
    EPwm2Regs.AQCSFRC.bit.CSFB = 1;
    EPwm3Regs.AQCSFRC.bit.CSFA = 1;
    EPwm3Regs.AQCSFRC.bit.CSFB = 1;
    EPwm4Regs.AQCSFRC.bit.CSFA = 1;
    EPwm4Regs.AQCSFRC.bit.CSFB = 1;
    EPwm5Regs.AQCSFRC.bit.CSFA = 1;
    EPwm5Regs.AQCSFRC.bit.CSFB = 1;
    EPwm6Regs.AQCSFRC.bit.CSFA = 1;
    EPwm6Regs.AQCSFRC.bit.CSFB = 1;

    // TBCLKSYNC 在 main() 末尾统一释放
}


//******************* EPWM 同步策略 *******************//
// 前级同步整流组（EPWM1 主 → EPWM2/3 从）
// 后级 SPWM 组（EPWM4 主 → EPWM5/6 从）
// 两组通过 TBCLKSYNC 全局同步启动

void EPWM1_Init(void)
{
    EALLOW;
    CpuSysRegs.PCLKCR2.bit.EPWM1 = 1;
    EDIS;

    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;
    EPwm1Regs.TBPHS.all = 0;
    EPwm1Regs.TBCTR = 0x0000;
    EPwm1Regs.TBPRD = EPWM_TIMER_TBPRD;
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    EPwm1Regs.CMPA.bit.CMPA = 0;

    EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm1Regs.AQCTLA.bit.CAD = AQ_SET;
    EPwm1Regs.AQCTLB.bit.ZRO = AQ_CLEAR;
    EPwm1Regs.AQCTLB.bit.CBU = AQ_SET;
    EPwm1Regs.AQCTLB.bit.CBD = AQ_CLEAR;

    EPwm1Regs.DBCTL.bit.IN_MODE = 0;
    EPwm1Regs.DBCTL.bit.POLSEL = 2;
    EPwm1Regs.DBCTL.bit.OUT_MODE = 3;
    EPwm1Regs.DBRED.bit.DBRED = 15;
    EPwm1Regs.DBFED.bit.DBFED = 15;

    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;
    EPwm1Regs.ETSEL.bit.INTEN = 0;
    EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;
}

void EPWM2_Init(void)
{
    EALLOW;
    CpuSysRegs.PCLKCR2.bit.EPWM2 = 1;
    EDIS;

    EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
    EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE;
    EPwm2Regs.TBPHS.all = 0;
    EPwm2Regs.TBCTR = 0x0000;
    EPwm2Regs.TBPRD = EPWM_TIMER_TBPRD;
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    EPwm2Regs.CMPA.bit.CMPA = 0;

    EPwm2Regs.AQCTLA.bit.ZRO = AQ_SET;
    EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm2Regs.AQCTLA.bit.CAD = AQ_SET;
    EPwm2Regs.AQCTLB.bit.ZRO = AQ_CLEAR;
    EPwm2Regs.AQCTLB.bit.CBU = AQ_SET;
    EPwm2Regs.AQCTLB.bit.CBD = AQ_CLEAR;

    EPwm2Regs.DBCTL.bit.IN_MODE = 0;
    EPwm2Regs.DBCTL.bit.POLSEL = 2;
    EPwm2Regs.DBCTL.bit.OUT_MODE = 3;
    EPwm2Regs.DBRED.bit.DBRED = 15;
    EPwm2Regs.DBFED.bit.DBFED = 15;

    EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;
    EPwm2Regs.ETSEL.bit.INTEN = 0;
    EPwm2Regs.ETPS.bit.INTPRD = ET_1ST;
}

void EPWM3_Init(void)
{
    EALLOW;
    CpuSysRegs.PCLKCR2.bit.EPWM3 = 1;
    EDIS;

    EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
    EPwm3Regs.TBCTL.bit.PHSEN = TB_ENABLE;
    EPwm3Regs.TBPHS.all = 0;
    EPwm3Regs.TBCTR = 0x0000;
    EPwm3Regs.TBPRD = EPWM_TIMER_TBPRD;
    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    EPwm3Regs.CMPA.bit.CMPA = 0;                        // CMPA=0，ADC 在 CTR=0 上计数时触发

    EPwm3Regs.AQCTLA.bit.ZRO = AQ_SET;
    EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm3Regs.AQCTLA.bit.CAD = AQ_SET;
    EPwm3Regs.AQCTLB.bit.ZRO = AQ_CLEAR;
    EPwm3Regs.AQCTLB.bit.CBU = AQ_SET;
    EPwm3Regs.AQCTLB.bit.CBD = AQ_CLEAR;

    EPwm3Regs.DBCTL.bit.IN_MODE = 0;
    EPwm3Regs.DBCTL.bit.POLSEL = 2;
    EPwm3Regs.DBCTL.bit.OUT_MODE = 3;
    EPwm3Regs.DBRED.bit.DBRED = 15;
    EPwm3Regs.DBFED.bit.DBFED = 15;

    // ADC 触发配置：CTR=CMPA(=0) 上计数时触发 SOCA
    EPwm3Regs.ETSEL.bit.SOCAEN  = 1;
    EPwm3Regs.ETSEL.bit.SOCASEL = 2;                    // CTR=CMPA on up-count
    EPwm3Regs.ETPS.bit.SOCAPRD  = 1;
}

void EPWM4_Init(void)
{
    EALLOW;
    CpuSysRegs.PCLKCR2.bit.EPWM4 = 1;
    EDIS;

    EPwm4Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;         // 后级主模块
    EPwm4Regs.TBCTL.bit.PHSEN = TB_DISABLE;
    EPwm4Regs.TBPHS.all = 0;
    EPwm4Regs.TBCTR = 0x0000;
    EPwm4Regs.TBPRD = EPWM_TIMER_TBPRD;
    EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm4Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm4Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm4Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    EPwm4Regs.CMPA.bit.CMPA = 0;

    EPwm4Regs.AQCTLA.bit.ZRO = AQ_SET;
    EPwm4Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm4Regs.AQCTLA.bit.CAD = AQ_SET;
    EPwm4Regs.AQCTLB.bit.ZRO = AQ_CLEAR;
    EPwm4Regs.AQCTLB.bit.CBU = AQ_SET;
    EPwm4Regs.AQCTLB.bit.CBD = AQ_CLEAR;

    EPwm4Regs.DBCTL.bit.IN_MODE = 0;
    EPwm4Regs.DBCTL.bit.POLSEL = 2;
    EPwm4Regs.DBCTL.bit.OUT_MODE = 3;
    EPwm4Regs.DBRED.bit.DBRED = 15;
    EPwm4Regs.DBFED.bit.DBFED = 15;
}

void EPWM5_Init(void)
{
    EALLOW;
    CpuSysRegs.PCLKCR2.bit.EPWM5 = 1;
    EDIS;

    EPwm5Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
    EPwm5Regs.TBCTL.bit.PHSEN = TB_ENABLE;
    EPwm5Regs.TBPHS.all = 0;
    EPwm5Regs.TBCTR = 0x0000;
    EPwm5Regs.TBPRD = EPWM_TIMER_TBPRD;
    EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm5Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm5Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm5Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm5Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm5Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    EPwm5Regs.CMPA.bit.CMPA = 0;

    EPwm5Regs.AQCTLA.bit.ZRO = AQ_SET;
    EPwm5Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm5Regs.AQCTLA.bit.CAD = AQ_SET;
    EPwm5Regs.AQCTLB.bit.ZRO = AQ_CLEAR;
    EPwm5Regs.AQCTLB.bit.CBU = AQ_SET;
    EPwm5Regs.AQCTLB.bit.CBD = AQ_CLEAR;

    EPwm5Regs.DBCTL.bit.IN_MODE = 0;
    EPwm5Regs.DBCTL.bit.POLSEL = 2;
    EPwm5Regs.DBCTL.bit.OUT_MODE = 3;
    EPwm5Regs.DBRED.bit.DBRED = 15;
    EPwm5Regs.DBFED.bit.DBFED = 15;
}

void EPWM6_Init(void)
{
    EALLOW;
    CpuSysRegs.PCLKCR2.bit.EPWM6 = 1;
    EDIS;

    EPwm6Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;         // 同步于 EPWM4（后级从模块）
    EPwm6Regs.TBCTL.bit.PHSEN = TB_ENABLE;
    EPwm6Regs.TBPHS.all = 0;
    EPwm6Regs.TBCTR = 0x0000;
    EPwm6Regs.TBPRD = EPWM_TIMER_TBPRD;
    EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm6Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm6Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm6Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm6Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm6Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm6Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    EPwm6Regs.CMPA.bit.CMPA = 0;

    EPwm6Regs.AQCTLA.bit.ZRO = AQ_SET;
    EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm6Regs.AQCTLA.bit.CAD = AQ_SET;
    EPwm6Regs.AQCTLB.bit.ZRO = AQ_CLEAR;
    EPwm6Regs.AQCTLB.bit.CBU = AQ_SET;
    EPwm6Regs.AQCTLB.bit.CBD = AQ_CLEAR;

    EPwm6Regs.DBCTL.bit.IN_MODE = 0;
    EPwm6Regs.DBCTL.bit.POLSEL = 2;
    EPwm6Regs.DBCTL.bit.OUT_MODE = 3;
    EPwm6Regs.DBRED.bit.DBRED = 15;
    EPwm6Regs.DBFED.bit.DBFED = 15;

    EPwm6Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;
    EPwm6Regs.ETSEL.bit.INTEN = 0;
    EPwm6Regs.ETPS.bit.INTPRD = ET_1ST;
}

void EPWM7_Init(void)
{
    EALLOW;
    CpuSysRegs.PCLKCR2.bit.EPWM7 = 1;
    EDIS;

    EPwm7Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
    EPwm7Regs.TBCTL.bit.PHSEN = TB_ENABLE;
    EPwm7Regs.TBPHS.all = 0;
    EPwm7Regs.TBCTR = 0x0000;
    EPwm7Regs.TBPRD = EPWM_TIMER_TBPRD;
    EPwm7Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm7Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm7Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm7Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm7Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm7Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm7Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    EPwm7Regs.CMPA.bit.CMPA = 0;

    EPwm7Regs.AQCTLA.bit.ZRO = AQ_SET;
    EPwm7Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm7Regs.AQCTLA.bit.CAD = AQ_SET;
    EPwm7Regs.AQCTLB.bit.ZRO = AQ_CLEAR;
    EPwm7Regs.AQCTLB.bit.CBU = AQ_SET;
    EPwm7Regs.AQCTLB.bit.CBD = AQ_CLEAR;

    EPwm7Regs.DBCTL.bit.IN_MODE = 0;
    EPwm7Regs.DBCTL.bit.POLSEL = 2;
    EPwm7Regs.DBCTL.bit.OUT_MODE = 3;
    EPwm7Regs.DBRED.bit.DBRED = 15;
    EPwm7Regs.DBFED.bit.DBFED = 15;

    EPwm7Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;
    EPwm7Regs.ETSEL.bit.INTEN = 0;
    EPwm7Regs.ETPS.bit.INTPRD = ET_1ST;
}

void EPWM8_Init(void)
{
    EALLOW;
    CpuSysRegs.PCLKCR2.bit.EPWM8 = 1;
    EDIS;

    EPwm8Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
    EPwm8Regs.TBCTL.bit.PHSEN = TB_ENABLE;
    EPwm8Regs.TBPHS.all = 0;
    EPwm8Regs.TBCTR = 0x0000;
    EPwm8Regs.TBPRD = EPWM_TIMER_TBPRD;
    EPwm8Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm8Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm8Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm8Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm8Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm8Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm8Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    EPwm8Regs.CMPA.bit.CMPA = 0;

    EPwm8Regs.AQCTLA.bit.ZRO = AQ_SET;
    EPwm8Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm8Regs.AQCTLA.bit.CAD = AQ_SET;
    EPwm8Regs.AQCTLB.bit.ZRO = AQ_CLEAR;
    EPwm8Regs.AQCTLB.bit.CBU = AQ_SET;
    EPwm8Regs.AQCTLB.bit.CBD = AQ_CLEAR;

    EPwm8Regs.DBCTL.bit.IN_MODE = 0;
    EPwm8Regs.DBCTL.bit.POLSEL = 2;
    EPwm8Regs.DBCTL.bit.OUT_MODE = 3;
    EPwm8Regs.DBRED.bit.DBRED = 15;
    EPwm8Regs.DBFED.bit.DBFED = 15;

    EPwm8Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;
    EPwm8Regs.ETSEL.bit.INTEN = 0;
    EPwm8Regs.ETPS.bit.INTPRD = ET_1ST;
}
