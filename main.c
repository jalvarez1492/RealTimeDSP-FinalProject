#include <F28x_Project.h>
#include <AIC23.h>
#include <InitAIC23.c>
#include <math.h>

/***********************************/
/**********DEFINES******************/
/***********************************/

#define theta0 150
#define c 334 // speed of sound in meters / sec
#define a 0.1 // radius of head in meters
#define w0 417.5
#define pi 3.14159
#define Fs 48000

/***********************************/

interrupt void McBSP_Rx_ISR(void);
void InitMcBSP_Rx_int(void);
interrupt void Timer1_isr(void);

void InitTimer1(void);
void InitAdca(void);
void InitGPIO(void);

extern void InitSPIA();
extern void InitAIC23();
extern void InitMcBSPb();


Uint16 AdcData = 0;
int16 right;
int16 left;
volatile float theta = 0.0;
float theta_right = 0.0;
float alfa = 0;
float alfa_left = 0;
float alfa_right = 0;
float gdelay_right;
float gdelay_left;
float a_right;
float a_left;

float fb_tap;
float ff_tap1;
float ff_tap2;
float b0 = 0;
float b1 = 0;
float a1 = 0;

float x1_left = 0;
float y1_left = 0;
float x1_right = 0;
float y1_right = 0;
float x2_left = 0;
float y2_left = 0;
float x2_right = 0;
float y2_right = 0;

float out_right = 0;
float out_left = 0;

float out_right_final = 0;
float out_left_final = 0;


int num_taps = 63;



int n;
int mask;

float cur_sample;

float fftaps[] = {
                  0.158679625409819,  0.00523049100855436,    0.00531300623243236,    0.00538848110491647,    0.00548464392636858,    0.00556593331023983,    0.00566048954952847,    0.00573454184721844,    0.00582335101518208,    0.00589051581378136,    0.00597379720731156,    0.00602460689693898,    0.00608510771621834,    0.00608907822109611,    0.00610205493952379,    0.00600111106161672,    0.00638755660131146,    0.00633709813255301,    0.00634142428049436,    0.00637770822065878,    0.00639814242518927,    0.00643181598851039,    0.00645833398105875,    0.00648404488302978,    0.00650555222454002,    0.00652232336253855,    0.00654489217882526,    0.00656173771900931,    0.00659027784244443,    0.00659500329054849,    0.00660684987966123,    0.00654120259345378,    0.00654120259345378,    0.00660684987966123,    0.00659500329054849,    0.00659027784244443,    0.00656173771900931,    0.00654489217882526,    0.00652232336253855,    0.00650555222454002,    0.00648404488302978,    0.00645833398105875,    0.00643181598851039,    0.00639814242518927,    0.00637770822065878,    0.00634142428049436,    0.00633709813255301,    0.00638755660131146,    0.00600111106161672,    0.00610205493952379,    0.00608907822109611,    0.00608510771621834,    0.00602460689693898,    0.00597379720731156,    0.00589051581378136,    0.00582335101518208,    0.00573454184721844,    0.00566048954952847,    0.00556593331023983,    0.00548464392636858,    0.00538848110491647,    0.00531300623243236,    0.00523049100855436,    0.158679625409819
};
float samples_left[64] = {};
float samples_right[64] = {};



int main(void)
{

    n = 0;
    mask = 64 - 1;

    InitSysCtrl();
    EALLOW;
    // Init PIE
    InitPieCtrl();      // Initialize PIE -> disable PIE and clear all PIE registers
    IER = 0x0000;       // Clear CPU interrupt register
    IFR = 0x0000;       // Clear CPU interrupt flag register
    InitPieVectTable(); // Initialize PIE vector table to known state

    InitMcBSP_Rx_int();

    EALLOW;

    InitSPIA();
    InitAIC23(DSP_16b);
    InitMcBSPb(DSP_16b);
    EALLOW;
    InitAdca();
    InitGPIO();

    EALLOW;

    GpioCtrlRegs.GPEDIR.bit.GPIO139 = 1;

    a1 = -0.84; //constant term

    while(1)
    {

        if(AdcData > 0 && AdcData < 372)
        {
            GpioDataRegs.GPACLEAR.all = 1;
            GpioCtrlRegs.GPADIR.all = 0;
            GpioCtrlRegs.GPADIR.bit.GPIO0 = 1;
            GpioDataRegs.GPASET.bit.GPIO0 = 1;
        }else if(AdcData >= 372 && AdcData < 744)
        {
            GpioDataRegs.GPACLEAR.all = 1;
            GpioCtrlRegs.GPADIR.all = 0;
            GpioCtrlRegs.GPADIR.bit.GPIO1 = 1;
            GpioDataRegs.GPASET.bit.GPIO1 = 1;
        }else if(AdcData >= 744 && AdcData < 1116)
        {
            GpioDataRegs.GPACLEAR.all = 1;
            GpioCtrlRegs.GPADIR.all = 0;
            GpioCtrlRegs.GPADIR.bit.GPIO2 = 1;
            GpioDataRegs.GPASET.bit.GPIO2 = 1;
        }else if(AdcData >= 1116 && AdcData < 1488)
        {
            GpioDataRegs.GPACLEAR.all = 1;
            GpioCtrlRegs.GPADIR.all = 0;
            GpioCtrlRegs.GPADIR.bit.GPIO3 = 1;
            GpioDataRegs.GPASET.bit.GPIO3 = 1;
        }else if(AdcData >= 1488 && AdcData < 1860)
        {
            GpioDataRegs.GPACLEAR.all = 1;
            GpioCtrlRegs.GPADIR.all = 0;
            GpioCtrlRegs.GPADIR.bit.GPIO4 = 1;
            GpioDataRegs.GPASET.bit.GPIO4 = 1;
        }else if(AdcData >= 1860 && AdcData < 2232)
        {
            GpioDataRegs.GPACLEAR.all = 1;
            GpioCtrlRegs.GPADIR.all = 0;
            GpioCtrlRegs.GPADIR.bit.GPIO5 = 1;
            GpioDataRegs.GPASET.bit.GPIO5 = 1;
        }else if(AdcData >= 2232 && AdcData < 2604)
        {
            GpioDataRegs.GPACLEAR.all = 1;
            GpioCtrlRegs.GPADIR.all = 0;
            GpioCtrlRegs.GPADIR.bit.GPIO6 = 1;
            GpioDataRegs.GPASET.bit.GPIO6 = 1;
        }else if(AdcData >= 2604 && AdcData < 2976)
        {
            GpioDataRegs.GPACLEAR.all = 1;
            GpioCtrlRegs.GPADIR.all = 0;
            GpioCtrlRegs.GPADIR.bit.GPIO7 = 1;
            GpioDataRegs.GPASET.bit.GPIO7 = 1;
        }else if(AdcData >= 2976 && AdcData < 3348)
        {
            GpioDataRegs.GPACLEAR.all = 1;
            GpioCtrlRegs.GPADIR.all = 0;
            GpioCtrlRegs.GPADIR.bit.GPIO8 = 1;
            GpioDataRegs.GPASET.bit.GPIO8 = 1;
        }else if(AdcData >= 3348 && AdcData < 3720)
        {
            GpioDataRegs.GPACLEAR.all = 1;
            GpioCtrlRegs.GPADIR.all = 0;
            GpioCtrlRegs.GPADIR.bit.GPIO9 = 1;
            GpioDataRegs.GPASET.bit.GPIO9 = 1;
        }else if (AdcData >= 3720)
        {
            GpioDataRegs.GPACLEAR.all = 1;
            GpioCtrlRegs.GPADIR.all = 0;
            GpioCtrlRegs.GPADIR.bit.GPIO10 = 1;
            GpioDataRegs.GPASET.bit.GPIO10 = 1;
        }


    }

}

/*************************************************************
 * ISRs
 ************************************************************/

interrupt void McBSP_Rx_ISR(void) {

    GpioDataRegs.GPETOGGLE.bit.GPIO139 = 1;

    AdcaRegs.ADCSOCFRC1.all = 0x1;          // Force conversion on channel 0
    AdcData = AdcaResultRegs.ADCRESULT0;    // Read ADC result into global variable
    theta = AdcData * (180.00 / 4096.0)-20.0;;
    theta_right = theta-110.0;

    right = McbspbRegs.DRR1.all;
    left = McbspbRegs.DRR2.all;

    // STAGE 1 -- GROUP DELAY FILTER (HEAD SHADOW)
    alfa_left = 1.05 + 0.95*cosf((theta/150)*pi);        //
    alfa_right = 1.05 + 0.95*cosf((theta_right/150)*pi); //

    b0 = (alfa_left + w0/Fs)/(1 + w0/Fs);
    b1 = (-alfa_left + w0/Fs)/(1 + w0/Fs);

    out_left = (b0 * (float)left + b1 * (float)x1_left + a1 * y1_left) / 3.0;

    samples_left[n & mask] = out_left;

    b0 = (alfa_right + w0/Fs)/(1 + w0/Fs);
    b1 = (-alfa_right + w0/Fs)/(1 + w0/Fs);

    out_right = (b0 * (float)right + b1 * (float)x1_right + a1 * y1_right) / 3.0;

    // END OF PROCESSING REASSIGNMENTS
    x1_right = (float)right;
    x1_left = (float)left;
    y1_right = out_right;
    y1_left = out_left;

    // STAGE 2 -- LOW PASS / BASS BOOST FILTER

    samples_right[n & mask] = out_right;


    for(int i = 0; i < num_taps; i++)
    {

        out_left += fftaps[i] * samples_left[(n-i) & mask];
        out_right += fftaps[i] * samples_right[(n-i) & mask];

    }

    // STAGE 3 -- Interaural Time Difference Filter (all-pass)

    gdelay_right = -Fs/w0 * (cosf(theta_right*pi/180) - 1);
    gdelay_left = -Fs/w0 * (cosf(theta*pi/180) - 1);

    a_right = (1 - gdelay_right)/(1 + gdelay_right);
    a_left = (1 - gdelay_left)/(1 + gdelay_left);

    out_right_final = (a_right * (float)out_right + 1 * (float)x2_right + a_left * y2_right) / 3.0;
    out_left_final = (a_right * (float)out_left + 1 * (float)x2_left + a_left * y2_left) / 3.0;


    McbspbRegs.DXR1.all = (int16)out_right;
    McbspbRegs.DXR2.all = (int16)out_left;

    // END OF PROCESSING REASSIGNMENTS
    x2_right = out_right;
    x2_left = out_left;
    y2_right = out_right_final;
    y2_left = out_left_final;

    EALLOW;
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP6;
    EDIS;

    GpioDataRegs.GPETOGGLE.bit.GPIO139 = 1;
}
/*************************************************************
 * Functions
 ************************************************************/
void InitMcBSP_Rx_int(void) {
    InitCpuTimers();                            // Initialize all timers to known state
       // Configure CPU timer 1. 200 -> SYSCLK in MHz, 500000 -> period in usec. NOTE: Does NOT start timer
    EALLOW;
    PieVectTable.MCBSPB_RX_INT = &McBSP_Rx_ISR;      // Assign Rx to PIE vector table
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block
    PieCtrlRegs.PIEIER6.bit.INTx7 = 1;   // Enable PIE Group 6, INT 7 (MCBSPB_RX)
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
    IER |= M_INT6;                         // Enable CPU INT6

    EnableInterrupts();                         // Enable PIE and CPU interrupts
}

void InitGPIO(void)
{
    EALLOW;

    GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;     // Blue LED for timer

    // LEDs
    GpioCtrlRegs.GPADIR.bit.GPIO10 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO10 = 0;

    GpioCtrlRegs.GPADIR.bit.GPIO9 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO9 = 0;

    GpioCtrlRegs.GPADIR.bit.GPIO8 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO8 = 0;

    GpioCtrlRegs.GPADIR.bit.GPIO7 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO7 = 0;

    GpioCtrlRegs.GPADIR.bit.GPIO6 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO6 = 0;

    GpioCtrlRegs.GPADIR.bit.GPIO5 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO5 = 0;

    GpioCtrlRegs.GPADIR.bit.GPIO4 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO4 = 0;

    GpioCtrlRegs.GPADIR.bit.GPIO3 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 0;

    GpioCtrlRegs.GPADIR.bit.GPIO2 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 0;

    GpioCtrlRegs.GPADIR.bit.GPIO1 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 0;

    GpioCtrlRegs.GPADIR.bit.GPIO0 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 0;

    GpioDataRegs.GPACLEAR.all = 1;

}

void InitAdca(void) {
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6;                                 // Set ADCCLK to SYSCLK/4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); // Initializes ADCA to 12-bit and single-ended mode. Performs internal calibration
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;                                 // Powers up ADC
    DELAY_US(1000);                                                    // Delay to allow ADC to power up
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0;                                 // Sets SOC0 to channel 0 -> pin ADCINA0
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 14;                                // Sets sample and hold window -> must be at least 1 ADC clock long
}
