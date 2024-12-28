//#############################################################################
// FILE:   LABstarter_main.c
//
// TITLE:  Lab Starter
//#############################################################################

// Included Files
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "F28377sSerial.h"
#include "F28377sADC.h"
#include "F28377sDAC.h"
#include "F28377sEPWM3A8A.h"
#include "F28377sEQep.h"
#include "lcd.h"
#include "SpiRAM.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398


// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);
__interrupt void ADCB_ISR(void);

extern float SIMU_value1_32bit; // any float value
extern float SIMU_value2_32bit; // any float value
extern float SIMU_value3_16bit; // smaller float between -32 and 32 because 16bit
extern float SIMU_value4_16bit; // smaller float between -32 and 32 because 16bits

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
uint16_t UARTPrint = 0;
uint32_t time = 0;
extern uint16_t SpiRAM_total_data;
int32_t ADCB0count=0;

//ADC variables
int16_t adcb0result=0;
int16_t adcb1result=0;
int16_t adcb2result=0;
float voltageb0=0;
float voltageb1=0;
float voltageb2=0;
//ADC avg variables
float sumAdcb0volts = 0;
float sumAdcb1volts = 0;
float sumAdcb2volts = 0;
int16_t sampleCount = 0; // counts number of times ADC channels have been sampled
float avgAdcb0 = 0;
float avgAdcb1 = 0;
float avgAdcb2 = 0;

long double xk=0;
long double xk_1=0;
long double xk_2=0;
long double xk_3=0;
long double xk_4=0;
long double yk=0;
long double yk_1=0;
long double yk_2=0;
long double yk_3=0;
long double yk_4=0;

////500
//long double num[5]={  1.5999999595806003e-03L,    6.3999998383224010e-03L,    9.6000004559755325e-03L,    6.3999998383224010e-03L,    1.5999999595806003e-03L};
//long double den[5]={  1.0000000000000000e+00L,    -2.4000000953674316e+00L,   2.1600000858306885e+00L,    -8.6400002241134644e-01L,   1.2960000336170197e-01L};

//100
//long double num[5]={  5.1418905968603212e-06L,    2.0567562387441285e-05L,    3.0851344490656629e-05L,    2.0567562387441285e-05L,    5.1418905968603212e-06L};
//long double den[5]={  1.0000000000000000e+00L,    -3.6190476417541504e+00L,   4.9115648269653320e+00L,    -2.9625310897827148e+00L,   6.7009627819061279e-01L};

//10
long double num[5]={  6.1265470106331452e-10L,    2.4506188042532581e-09L,    3.6759282063798873e-09L,    2.4506188042532581e-09L,    6.1265470106331452e-10L};
long double den[5]={  1.0000000000000000e+00L,    -3.9601990049751250e+00L,   5.8811910596272376e+00L,    -3.8817811637340647e+00L,   9.6078911888442653e-01L};
void SetupADCOverSampling(void)
{
    Uint16 acqps;

    //determine minimum acquisition window (in SYSCLKS) based on resolution
    if(ADC_RESOLUTION_12BIT == AdcaRegs.ADCCTL2.bit.RESOLUTION){
        acqps = 99; //500ns
    }
    else { //resolution is 16-bit
        acqps = 63; //320ns
    }

    //Select the channels to convert and end of conversion flag
    //ADCA
    EALLOW;
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 2;  //SOC0 will convert pin A2
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 3;  //SOC1 will convert pin A3
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles

//  AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 1; //end of SOC1 will set INT1 flag
//  AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
//  AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

    //ADCB
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 0;  //SOC0 will convert pin B0
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles
    AdcbRegs.ADCSOC1CTL.bit.CHSEL = 1;  //SOC1 will convert pin B1
    AdcbRegs.ADCSOC1CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles

    AdcbRegs.ADCSOC2CTL.bit.CHSEL = 2;  //SOC2 will convert pin B2
    AdcbRegs.ADCSOC2CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles
//    AdcbRegs.ADCSOC3CTL.bit.CHSEL = 3;  //SOC3 will convert pin B3
//    AdcbRegs.ADCSOC3CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles

    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 0xD; // EPWM5 ADCSOCA will cause SOC0
    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = 0xD; // EPWM5 ADCSOCA will cause SOC1
    AdcbRegs.ADCSOC2CTL.bit.TRIGSEL = 0xD; // EPWM5 ADCSOCA will cause SOC2

#warn  Make sure to start ADCB0 ADCB1 and ADCB2
    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 2; //end of SOC2 will set INT1 flag
    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    EDIS;
}

void main(void)
{

    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xS_SysCtrl.c file.
    InitSysCtrl();

    InitGpio();

    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xS_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xS_DefaultIsr.c.
    // This function is found in F2837xS_PieVect.c.
    InitPieVectTable();

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this project
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    PieVectTable.SCIA_RX_INT = &RXAINT_recv_ready;
	PieVectTable.SCIB_RX_INT = &RXBINT_recv_ready;
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCIA_TX_INT = &TXAINT_data_sent;
	PieVectTable.SCIB_TX_INT = &TXBINT_data_sent;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
	PieVectTable.DMA_CH4_INT = &DMA_ISR;
    PieVectTable.SPIA_RX_INT = &SPIA_isr;//spia
    PieVectTable.ADCB1_INT = &ADCB_ISR;

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;

    EDIS;    // This is needed to disable write to EALLOW protected registers

    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xS_CpuTimers.c
    InitCpuTimers();
	
    // Configure CPU-Timer 0, 1, and 2 to interrupt every 1ms, 20ms, 40ms respectively:
    // 200MHz CPU Freq, 			Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, 200, 1000);
    ConfigCpuTimer(&CpuTimer1, 200, 20000);
    ConfigCpuTimer(&CpuTimer2, 200, 100000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

	// Red LED, GPIO12
    GPIO_SetupPinMux(12, GPIO_MUX_CPU1, 0);						
	GPIO_SetupPinOptions(12, GPIO_OUTPUT, GPIO_PUSHPULL);		
	GpioDataRegs.GPASET.bit.GPIO12 = 1;

	// Blue LED, GPIO13
    GPIO_SetupPinMux(13, GPIO_MUX_CPU1, 0);				
	GPIO_SetupPinOptions(13, GPIO_OUTPUT, GPIO_PUSHPULL);
	GpioDataRegs.GPASET.bit.GPIO13 = 1;

	//LEDS
	//LED1  GPIO64
	GPIO_SetupPinMux(64, GPIO_MUX_CPU1, 0);
	GPIO_SetupPinOptions(64, GPIO_OUTPUT, GPIO_PUSHPULL);
	GpioDataRegs.GPCSET.bit.GPIO64 = 1;
	//LED2  GPIO91
	GPIO_SetupPinMux(91, GPIO_MUX_CPU1, 0);
	GPIO_SetupPinOptions(91, GPIO_OUTPUT, GPIO_PUSHPULL);
	GpioDataRegs.GPCSET.bit.GPIO91 = 1;
	//LED3  GPIO92
	GPIO_SetupPinMux(92, GPIO_MUX_CPU1, 0);
	GPIO_SetupPinOptions(92, GPIO_OUTPUT, GPIO_PUSHPULL);
	GpioDataRegs.GPCSET.bit.GPIO92 = 1;
	//LED4  GPIO99
	GPIO_SetupPinMux(99, GPIO_MUX_CPU1, 0);
	GPIO_SetupPinOptions(99, GPIO_OUTPUT, GPIO_PUSHPULL);
	GpioDataRegs.GPDSET.bit.GPIO99 = 1;
	
	//SWITCHES
	//SW1	GPIO41
	GPIO_SetupPinMux(41, GPIO_MUX_CPU1, 0);
	GPIO_SetupPinOptions(41, GPIO_INPUT, GPIO_PULLUP | GPIO_QUAL6);
	//SW2	GPIO66
	GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
	GPIO_SetupPinOptions(66, GPIO_INPUT, GPIO_PULLUP | GPIO_QUAL6);
	//SW3	GPIO73
	GPIO_SetupPinMux(73, GPIO_MUX_CPU1, 0);
	GPIO_SetupPinOptions(73, GPIO_INPUT, GPIO_PULLUP | GPIO_QUAL6);
	//SW4	GPIO78
	GPIO_SetupPinMux(78, GPIO_MUX_CPU1, 0);
	GPIO_SetupPinOptions(78, GPIO_INPUT, GPIO_PULLUP | GPIO_QUAL6);
	
	InitSpiaGpio();
	InitEPwm3Gpio();
	InitEPwm8Gpio();

	//Configure the ADCs and power them up
	ConfigureADC();

	//Setup the ADCs for software conversions
	SetupADCOverSampling();
	EALLOW;
	EPwm5Regs.ETSEL.bit.SOCAEN = 0; // Disable SOC on A group
	EPwm5Regs.TBCTL.bit.CTRMODE = TB_FREEZE; // freeze counter
	EPwm5Regs.ETSEL.bit.SOCASEL = 2; // Select Event when equal to PRD
	EPwm5Regs.ETPS.bit.SOCAPRD = 1; // Generate pulse on 1st event
	EPwm5Regs.TBCTR = 0x0000; // Clear counter
	EPwm5Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
	EPwm5Regs.TBCTL.bit.PHSEN = TB_DISABLE; // Disable phase loading
	EPwm5Regs.TBCTL.bit.CLKDIV = 0; // divide by 1 50Mhz Clock
	EPwm5Regs.TBPRD = 50000U; // (1)ms sample. U for unsigned
	EPwm5Regs.ETSEL.bit.SOCAEN = 1; //enable SOCA
	EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; //unfreeze, and enter up count mode
	EDIS;

	initEPwm3A();
    initEPwm7A();
	initEPwm8();
	setEPWM3A(0);
	setEPWM8A(0);
	initDACs();
	init_EQEPs();
	InitDma();         // Set up DMA for SPI configuration
    setupSpia();       // Initialize the SPI only
	//
    // Ensure DMA is connected to Peripheral Frame 2 bridge (EALLOW protected)
    //
    EALLOW;
    CpuSysRegs.SECMSEL.bit.PF2SEL = 1;
    EDIS;


	init_serialSCIA(&SerialA,115200);  // Matlab COM
	
	init_serialSCIB(&SerialB,115200);  //Simulink COM
	
	init_serialSCIC(&SerialC,19200); // For Text LCD


	// ************ your Init code here  *******************


    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT6;
    IER |= M_INT7;  //DMA
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;

    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block

    // Enable ADCB1 in the PIE: Group 1 interrupt 2
    PieCtrlRegs.PIEIER1.bit.INTx2 = 1;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
	PieCtrlRegs.PIEIER6.bit.INTx1 = 1; //spia
    // Enable dma ch5 in the PIE: Group 7 interrupt 4
    PieCtrlRegs.PIEIER7.bit.INTx4 = 1;   // Enable PIE Group 7, INT 4 (DMA CH4)
    // Enable SWI in the PIE: Group 12 interrupt
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;



    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

	init_lcd(90);
	
    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 ) {
            UART_printfLine(1,"ADCB0 voltage %.3f",avgAdcb0);
            UARTPrint = 0;
        }
    }
}



// SWI_isr,  Using this interrupt as a Software started interrupt
__interrupt void SWI_isr(void) {

    // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
    // making it lower priority than all other Hardware interrupts.
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    asm("       NOP");                    // Wait one cycle
    EINT;                                 // Clear INTM to enable interrupts



    // Insert SWI ISR Code here.......


//    numSWIcalls++;
//    if((numSWIcalls%100)==0){
//        UARTPrint=1;
//    }
//    setDAC1(avgAdcb0);
    DINT;

}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;

    numTimer0calls++;

    //    if ((numTimer0calls%50) == 0) {
    //        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
    //    }
    // Start ADC conversion.
//    AdcbRegs.ADCSOCPRICTL.bit.RRPOINTER = 0x10; //rr pionter reset, soc0 next
//    AdcbRegs.ADCSOCFRC1.all |= 0x7; //start conversion of ADCINB0, ADCINB1 & ADCINB2
    if ((numTimer0calls%250) == 0) {
		// Blink LaunchPad Red LED
		GpioDataRegs.GPATOGGLE.bit.GPIO12 = 1;
    }
	

    // Example for using the saveData function to save date to the SPIRAM for data collection to Matlab
    //saveData(time*0.001,1.5*sin(2*PI*.5*time*.001),3*sin(2*PI*.5*time*.001),4*sin(2*PI*.5*time*.001),5*sin(2*PI*.5*time*.001));
    //time = time + 1;

    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{


    CpuTimer1.InterruptCount++;
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{


    // Blink LaunchPad Blue LED
    GpioDataRegs.GPATOGGLE.bit.GPIO13 = 1;

    CpuTimer2.InterruptCount++;

//    if ((CpuTimer2.InterruptCount % 50) == 0) {
//        UARTPrint = 1;
//    }
}


//Adcb1 pie interrupt
__interrupt void ADCB_ISR (void)
{
    adcb0result = AdcbResultRegs.ADCRESULT0;
    adcb1result = AdcbResultRegs.ADCRESULT1;
    adcb2result = AdcbResultRegs.ADCRESULT2;
    // Here covert ADCINB0, ADCINB1 and ADCINB2 to volts
    // Here write ADCINB0�s voltage value to DAC1 channel
    // Print ADCINB0�s voltage to the text LCD every 100ms by setting UARTPrint to 1
    // and the function UART_printfLine is called in main()�s while(1) loop
    voltageb0 = 10-20*(float)(adcb0result/4096.0);
//    voltageb1 = 10-20*(float)(adcb1result/4096.0);
//    voltageb2 = 3*(float)(adcb2result/4096.0);

//    setDAC1(voltageb0);
    xk = voltageb0;
    yk = num[0]*xk + num[1]*xk_1 + num[2]*xk_2+num[3]*xk_3+num[4]*xk_4 - den[1]*yk_1 - den[2]*yk_2 - den[3]*yk_3 - den[4]*yk_4; // Make sure you know where this equation came from
    setDAC1(yk); // write filtered value to the DAC to see filtered value on the Oscilloscope
    // save past xk states for next interrupt call to this function
    // save past yk states for next interrupt call to this function
    yk_4 =yk_3;
    yk_3=yk_2;
    yk_2=yk_1;
    yk_1=yk;

    xk_4=xk_3;
    xk_3=xk_2;
    xk_2=xk_1;
    xk_1=xk;



    ADCB0count++;
    if((ADCB0count%100)==0){
        UARTPrint=1;
    }

    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

}


//adcb1 pie interrupt
//__interrupt void ADCB_ISR (void)
//{
//    adcb0result = AdcbResultRegs.ADCRESULT0;
//    adcb1result = AdcbResultRegs.ADCRESULT1;
//    adcb2result = AdcbResultRegs.ADCRESULT2;
//    sumAdcb0volts = 10-20.0*(float)(adcb0result/4096.0); // equation converting 12bit value to 10 to -10V voltages
//    sumAdcb1volts = 10-20.0*(float)(adcb1result/4096.0); // equation converting 12bit value to 10 to -10V voltages
//    sumAdcb2volts = 3*(float)(adcb2result/4096.0); // equation converting 12bit value to 0 to 3V voltages
////    if (sampleCount == 16){
////        avgAdcb0 = sumAdcb0volts/16.0; // oversampled value
////        avgAdcb1 = sumAdcb1volts/16.0; // oversampled value
////        avgAdcb2 = sumAdcb2volts/16.0; // oversampled value
////        // Reset variables for next cycle of oversampling
////        sampleCount = 0;
////        sumAdcb0volts = 0;
////        sumAdcb1volts = 0;
////        sumAdcb2volts = 0;
////        // This below line causes the SWI to run when priority permits
////        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1; // Manually cause the interrupt for the SWI
////    }
//
//    setDAC1(sumAdcb0volts);
//    PieCtrlRegs.PIEIFR12.bit.INTx9 = 1; // Manually cause the interrupt for the SWI
//    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
//    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
//}





