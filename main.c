#include "msp.h"
#include "stdio.h"

#define ALLDUTY (uint16_t)(7540)
#define DUTYDOWN (uint16_t)(377)
#define DUTYUP (uint16_t)(754)
#define PERIOD_TIMER TIMER_A2
#define PWM_TIMER TIMER_A0
#define ADC14MEM0 0x00000001 // destination register
#define ADC14MEM0_IDX 0

/**
void ACLK_REFOCLK_config_1kHz(void);
void ACLK_REFOCLK_config_32kHz(void);
void HSMCLK_DCOCLK_config_48MHz(void);

void Timer_A_delay_config();
void Timer_A_delay(uint16_t delay, uint_fast8_t* isTimerOn);
**/

void Timer_A0_PWM_Config(uint16_t dutyCycle);

void Timer_A2_config();
void TA2_0_IRQHandler(void);

void ADC14_config(void);
void ADC14_sample(void);
void ADC14_IRQHandler(void);

/**
 * main.c
 */
void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD; // stop watchdog timer

    //**********************************************************************
    // Set up TimerA clock signal: RFOCLK to 32kHz, ACLK -- CHECKED
    //**********************************************************************
    CS->KEY = CS_KEY_VAL;
    BITBAND_PERI(CS->CLKEN, CS_CLKEN_REFO_EN_OFS) = 1; // enable REFOCLK
    BITBAND_PERI(CS->CLKEN, CS_CLKEN_REFOFSEL_OFS) = 0; // set REFOCLK to 32kHz
    while (!BITBAND_PERI(CS->STAT, CS_STAT_ACLK_READY_OFS));
    CS->CTL1 = (CS_CTL1_DIVA_1 + CS_CTL1_SELA__REFOCLK) | (CS->CTL1 & ~((CS_CTL1_SELA_MASK | CS_CTL1_DIVA_MASK))); // set ACLK, source: REFOCLK, divider: /1
    while (!BITBAND_PERI(CS->STAT, CS_STAT_ACLK_READY_OFS));
    CS->KEY = 0x0000;

    //**********************************************************************
//    // Set up ADC14 clock signal: HSMCLK with DCO default frequency, DCOCLK -- CHECKED
//    //**********************************************************************
    CS->KEY = CS_KEY_VAL;
    while (!BITBAND_PERI(CS->STAT, CS_STAT_HSMCLK_READY_OFS));
    CS->CTL1 = (CS_CTL1_DIVHS_0 + CS_CTL1_SELS__DCOCLK) | (CS->CTL1 & ~((CS_CTL1_SELS_MASK | CS_CTL1_DIVHS_MASK))); // set ACLK, source: REFOCLK, divider: 2^5=32
    while (!BITBAND_PERI(CS->STAT, CS_STAT_HSMCLK_READY_OFS));

    while (!BITBAND_PERI(CS->STAT, CS_STAT_SMCLK_READY_OFS));
    CS->CTL1 = (CS_CTL1_DIVS_3 + CS_CTL1_SELS__DCOCLK) | (CS->CTL1 & ~(CS_CTL1_DIVS_MASK | CS_CTL1_SELS_MASK));
    while (!BITBAND_PERI(CS->STAT, CS_STAT_SMCLK_READY_OFS));
    CS->KEY = 0x0000;

    // For testing, comment when finished
    // P4->DIR= 0xFF; //set Port 4 to output direction

    // Testing of Timer A0, for PWM
    // Timer_A0_PWM_Config((uint16_t)((DUTYUP+DUTYDOWN)/2));

    // Testing of Timer A2, for periodic interruption
    Timer_A2_config();

    // Testing of ADC14
    P4->DIR= 0xFF; //set Port 4 to output direction
    ADC14_config();
    // ADC14_sample();

    while(1) {

    }
}



// Timer A2 for periodic interruption
void Timer_A2_config(void){
    //**********************************************************************
    // Interrupt controller setup
    //**********************************************************************
    NVIC_EnableIRQ(TA2_0_IRQn);
    //**********************************************************************
    // Set up clock signal: RFOCLK to 32kHz, ACLK -- CHECKED
    //**********************************************************************
    // ACLK_REFOCLK_config_32kHz();
    //**********************************************************************
    // Configure Timer A to Compare & Up Mode
    //**********************************************************************
    // TimerA2 divider: 1
    PERIOD_TIMER->CTL &= ~TIMER_A_CTL_ID_MASK; TIMER_A0->CTL |= TIMER_A_CTL_ID__1;
    PERIOD_TIMER->EX0 = TIMER_A_EX0_TAIDEX_0;
    // ACLK as source clock, up mode, disable timer interrupt
    PERIOD_TIMER->CTL &= ~(TIMER_A_CTL_SSEL_MASK + TIMER_A_CTL_MC_MASK + TIMER_A_CTL_CLR + TIMER_A_CTL_IE);
    PERIOD_TIMER->CTL |= (TIMER_A_CTL_SSEL__ACLK + TIMER_A_CTL_MC__UP + TIMER_A_CTL_CLR);
    // enable CCR0 interrupt, compare mode, set/reset(no use)
    PERIOD_TIMER->CCR[0] = 32*50-1; // set CCR0, interrupt every 50ms
    PERIOD_TIMER->CCTL[0] &= ~((1<<TIMER_A_CCTLN_CCIE_OFS) + TIMER_A_CCTLN_OUTMOD_MASK + (1<<TIMER_A_CCTLN_CAP_OFS)); // clear interrupt flag
    PERIOD_TIMER->CCTL[0] |= (0<<TIMER_A_CCTLN_CAP_OFS) + TIMER_A_CCTLN_OUTMOD_3 + TIMER_A_CCTLN_CCIE; // interrupt enable, outmode: set/reset, compare
    PERIOD_TIMER->CTL |= TIMER_A_CTL_MC_1; // start timer

    // For testing, comment when finished
    BITBAND_PERI(P2->DIR , 7) = 1; // toggle output pin
}


void TA2_0_IRQHandler(void)
{
    BITBAND_PERI(PERIOD_TIMER->CCTL[0],TIMER_A_CCTLN_CCIFG_OFS) = 0;
    BITBAND_PERI(PERIOD_TIMER->CTL , TIMER_A_CTL_CLR_OFS) = 1;

    // For testing, comment when finished
    BITBAND_PERI(P2->OUT , 7) = ~BITBAND_PERI(P2->OUT , 7); // toggle output pin

    // TODO:
    ADC14_sample();
}

void ADC14_config(void){
    /*
    // HSMCLK_DCOCLK_config_48MHz();
    CS->KEY = CS_KEY_VAL;
    BITBAND_PERI(CS->CLKEN, CS_STAT_DCO_ON_OFS) = 1; // enable DCOCLK
    // CS->CTL0 = ((CS->CTL0 & ~CS_CTL0_DCORSEL_MASK) | CS_CTL0_DCORSEL_5); // set DCOCLK to 24MHz
    while (!BITBAND_PERI(CS->STAT, CS_STAT_HSMCLK_READY_OFS));
    CS->CTL1 = (CS_CTL1_DIVHS_0 + CS_CTL1_SELS__DCOCLK) | (CS->CTL1 & ~((CS_CTL1_SELS_MASK | CS_CTL1_DIVHS_MASK))); // set ACLK, source: REFOCLK, divider: 2^5=32
    while (!BITBAND_PERI(CS->STAT, CS_STAT_HSMCLK_READY_OFS));
    CS->KEY = 0x0000;
    */
    //**********************************************************************
    // Configure FPU for floating point number calculation
    //**********************************************************************
    // enable module
    SCB->CPACR = ((SCB->CPACR & ~(SCB_CPACR_CP11_MASK | SCB_CPACR_CP10_MASK)) | SCB_CPACR_CP11_MASK | SCB_CPACR_CP10_MASK);
    // enable lazy stacking
    FPU->FPCCR |= (1 << 31 + 1 << 30);
    //**********************************************************************
    // Configure analog input pin, tertiary module function, P5.5
    //**********************************************************************
    P5->DIR |= (0<<5);
    P5->SEL1 |= (1<<5);
    P5->SEL0 |= (1<<5);
    //**********************************************************************
    // Configure ADC14
    //**********************************************************************
    // enable interrupt handler
    NVIC_EnableIRQ(ADC14_IRQn);
    // enable module
    BITBAND_PERI(ADC14->CTL0, ADC14_CTL0_ON_OFS) = 1;
    // module initialization: divider (/1), HSMCLK as source
    ADC14->CTL0 &= ~(ADC14_CTL0_PDIV_MASK + ADC14_CTL0_DIV_MASK + ADC14_CTL0_SSEL_MASK);
    ADC14->CTL0 |= ADC14_CTL0_PDIV_0 + ADC14_CTL0_DIV_0 + ADC14_CTL0_SSEL__HSMCLK;
    // set resolution
    ADC14->CTL1 &= ~(ADC14_CTL1_CH3MAP + ADC14_CTL1_CH2MAP + ADC14_CTL1_CH1MAP + ADC14_CTL1_CH0MAP + ADC14_CTL1_TCMAP + ADC14_CTL1_BATMAP + ADC14_CTL1_RES_MASK);

    // For pin out testing, comment when finished and set to 14-bit resolution
    // ADC14->CTL1 |= 0 + ADC14_CTL1_RES__8BIT; // ADC14_CTL1_RES__14BIT

    ADC14->CTL1 |= 0 + ADC14_CTL1_RES__14BIT;
    // set destination register
    ADC14->CTL1 &= ~(ADC14_CTL1_CSTARTADD_MASK);
    ADC14->CTL1 |= ADC14MEM0_IDX << 16;
    // set repeated single sample mode
    ADC14->CTL0 &= ~(ADC14_CTL0_CONSEQ_MASK);
    ADC14->CTL0 |= ADC14_CTL0_CONSEQ_2;
    // configure conversion memory
    while(BITBAND_PERI(ADC14->CTL0, ADC14_CTL0_BUSY_OFS)); // wait still finish running
    // set to single ended mode and reference voltage
    ADC14->MCTL[0] &= ~(ADC14_MCTLN_INCH_MASK + ADC14_MCTLN_VRSEL_MASK + ADC14_MCTLN_DIF);
    ADC14->MCTL[0] |= ADC14_MCTLN_INCH_0 + ADC14_MCTLN_VRSEL_0;
    // enable interrupt
    uint32_t stat = ADC14_IER0_IE0 & 0xFFFFFFFF;
    ADC14->IER0 |= stat;
    stat = (ADC14_IER0_IE0 >> 32);
    ADC14->IER1 |= (stat);

}

void ADC14_sample(void){
    // enable sample timer
    while(BITBAND_PERI(ADC14->CTL0, ADC14_CTL0_BUSY_OFS)); // wait still finish running
    BITBAND_PERI(ADC14->CTL0, ADC14_CTL0_SHP_OFS) = 1; // SAMPCON signal is sourced from the sampling timer.
    BITBAND_PERI(ADC14->CTL0, ADC14_CTL0_MSC_OFS) = 0; // The sampling timer requires a rising edge of the SHI signal to trigger each sample-and-convert.
    // enable conversion
    ADC14->CTL0 |= (ADC14_CTL0_ENC);
    // toggle conversion trigger
    BITBAND_PERI(ADC14->CTL0, ADC14_CTL0_SC_OFS) = ~BITBAND_PERI(ADC14->CTL0, ADC14_CTL0_SC_OFS);
}

void ADC14_IRQHandler(void){
    // get interrupt status
    uint_fast64_t status = ((ADC14->IFGR1 << 32) | ADC14->IFGR0) & ((ADC14->IER1 << 32) | ADC14->IER0);
    // clear interrupt flag
    uint32_t flag = status & 0xFFFFFFFF;
    ADC14->CLRIFGR0 |= flag;
    flag = (status >> 32);
    ADC14->CLRIFGR1 |= (flag);

    if (status & ADC14_IER0_IE0){
        uint16_t curADCvalue = *((uint16_t*) (&ADC14->MCTL[0] + 0x20));

        // For testing, comment when finished - pin out for varification, 8-bit resolution
        P4->OUT = (uint_fast8_t)(curADCvalue<<6);
        uint16_t duty = curADCvalue * (DUTYUP - DUTYDOWN) / 16384 + DUTYDOWN;
        // uint16_t duty = curADCvalue * ALLDUTY /16384;
        // PWM_TIMER->CCR[1] = duty;
        Timer_A0_PWM_Config(duty);

        // uint16_t duty = curADCvalue*ALLDUTY/16384;
        // PWM_TIMER->CCR[1] = duty;

    }
    printf("%d",ADC14->MEM[0]);

    // For testing, comment when finished
    // ADC14_sample();
}

void Timer_A0_PWM_Config(uint16_t dutyCycle){
    //**********************************************************************
    // Set up clock signal: RFOCLK to 32kHz, ACLK -- CHECKED
    //**********************************************************************
    // ACLK_REFOCLK_config_32kHz();
    //*********************************************************************
    // Map P2.4 to TimerA0-C1, primary peripheral function
    //*********************************************************************
    P2->DIR |= (1<<4);
    P2->OUT |= (1<<4);
    P2->SEL1 &= (0<<4);
    P2->SEL0 |= (1<<4);
    //*********************************************************************
    // Generate PWM signal
    //*********************************************************************
    PWM_TIMER->CTL &= ~TIMER_A_CTL_ID_MASK;
    PWM_TIMER->CTL |= TIMER_A_CTL_ID__1;
    PWM_TIMER->EX0 = TIMER_A_EX0_TAIDEX_0; // TimerA0 divider: 1
    PWM_TIMER->CTL &= ~(TIMER_A_CTL_SSEL_MASK + TIMER_A_CTL_MC_MASK + TIMER_A_CTL_CLR + TIMER_A_CTL_IE);
    PWM_TIMER->CTL |= (TIMER_A_CTL_SSEL__SMCLK + TIMER_A_CTL_MC__UP + TIMER_A_CTL_CLR);
    PWM_TIMER->CCR[0] = ALLDUTY; // all duty
    PWM_TIMER->CCTL[0] &= ~(TIMER_A_CCTLN_CCIE + TIMER_A_CCTLN_OUTMOD_MASK); // disable interrupt, set to Output mode.
    PWM_TIMER->CCTL[1] |= TIMER_A_CCTLN_OUTMOD_7; // Reset/Set
    PWM_TIMER->CCR[1] = dutyCycle;
}

void Timer_A_delay_config(){

    //**********************************************************************
    // Set up clock signal: RFOCLK to 32kHz, ACLK -- CHECKED
    //**********************************************************************
// ACLK_REFOCLK_config_1kHz();
    //**********************************************************************
    // Configure Timer A to Compare & Up Mode
    //**********************************************************************
    // Timer divider - 1; using ACLK, Up mode, clear enable, interrupt enable
    TIMER_A0->CTL &= ~TIMER_A_CTL_ID_MASK; TIMER_A0->CTL |= TIMER_A_CTL_ID__1;
    TIMER_A0->EX0 = TIMER_A_EX0_TAIDEX_0; // TimerA0 divider: 1
    TIMER_A0->CTL &= ~(TIMER_A_CTL_SSEL_MASK + TIMER_A_CTL_MC_MASK + TIMER_A_CTL_CLR + TIMER_A_CTL_IE);
    TIMER_A0->CTL |= (TIMER_A_CTL_SSEL__ACLK + TIMER_A_CTL_MC__UP + TIMER_A_CTL_CLR + TIMER_A_CTL_IE); // source: ACLK, Up Mode, Clear enable, interrupt enable
    TIMER_A0->CCR[0] = 0xffff; // set CCR0
    TIMER_A0->CCTL[1] &= ~((1<<TIMER_A_CCTLN_CCIE_OFS) + TIMER_A_CCTLN_OUTMOD_MASK + (1<<TIMER_A_CCTLN_CAP_OFS));
    TIMER_A0->CCTL[1] |= (0<<TIMER_A_CCTLN_CAP_OFS) + TIMER_A_CCTLN_OUTMOD_1 + TIMER_A_CCTLN_CCIE; // interrupt enable, outmode: set, compare
    // TIMER_A0->CTL |= TIMER_A_CTL_MC_1; // start timer
    // TIMER_A0->CCR[1] = 0x0011;
}

void Timer_A_delay(uint16_t delay, uint_fast8_t* isTimerOn){
    if (delay >= 0xffff){ return; }
    if (!BITBAND_PERI(*isTimerOn, 4)){
        TIMER_A0->CCR[1] = delay; BITBAND_PERI(*isTimerOn, 4)=1; // write CCRn register
    }
    if (!BITBAND_PERI(*isTimerOn, 0)){
        TIMER_A0->CTL |= TIMER_A_CTL_MC_1; *isTimerOn = 0x01;// start timer
    }
    if (BITBAND_PERI(TIMER_A0->CCTL[1], TIMER_A_CCTLN_CCIFG_OFS) || BITBAND_PERI(TIMER_A0->CCTL[0], TIMER_A_CCTLN_CCIFG_OFS)){
        TIMER_A0->CTL &= ~TIMER_A_CTL_MC_3; // stop timer
        *isTimerOn = 0x00;
        P2->OUT = (0x00 + 1<<1);
    }
}

void ACLK_REFOCLK_config_1kHz(void){
    CS->KEY = CS_KEY_VAL;
    BITBAND_PERI(CS->CLKEN, CS_CLKEN_REFO_EN_OFS) = 1; // enable REFOCLK
    BITBAND_PERI(CS->CLKEN, CS_CLKEN_REFOFSEL_OFS) = 0; // set REFOCLK to 32kHz
    while (!BITBAND_PERI(CS->STAT, CS_STAT_ACLK_READY_OFS));
    CS->CTL1 = (CS_CTL1_DIVA_5 + CS_CTL1_SELA__REFOCLK) | (CS->CTL1 & ~((CS_CTL1_SELA_MASK | CS_CTL1_DIVA_MASK))); // set ACLK, source: REFOCLK, divider: 2^5=32
    while (!BITBAND_PERI(CS->STAT, CS_STAT_ACLK_READY_OFS));
    CS->KEY = 0x0000;
}

void ACLK_REFOCLK_config_32kHz(void){
    CS->KEY = CS_KEY_VAL;
    BITBAND_PERI(CS->CLKEN, CS_CLKEN_REFO_EN_OFS) = 1; // enable REFOCLK
    BITBAND_PERI(CS->CLKEN, CS_CLKEN_REFOFSEL_OFS) = 0; // set REFOCLK to 32kHz
    while (!BITBAND_PERI(CS->STAT, CS_STAT_ACLK_READY_OFS));
    CS->CTL1 = (CS_CTL1_DIVA_0 + CS_CTL1_SELA__REFOCLK) | (CS->CTL1 & ~((CS_CTL1_SELA_MASK | CS_CTL1_DIVA_MASK))); // set ACLK, source: REFOCLK, divider: /1
    while (!BITBAND_PERI(CS->STAT, CS_STAT_ACLK_READY_OFS));
    CS->KEY = 0x0000;
}

void HSMCLK_DCOCLK_config_48MHz(void){
    CS->KEY = CS_KEY_VAL;
    BITBAND_PERI(CS->CLKEN, CS_STAT_DCO_ON_OFS) = 1; // enable DCOCLK
    // CS->CTL0 = ((CS->CTL0 & ~CS_CTL0_DCORSEL_MASK) | CS_CTL0_DCORSEL_5); // set DCOCLK to 24MHz
    while (!BITBAND_PERI(CS->STAT, CS_STAT_HSMCLK_READY_OFS));
    CS->CTL1 = (CS_CTL1_DIVHS_0 + CS_CTL1_SELS__DCOCLK) | (CS->CTL1 & ~((CS_CTL1_SELS_MASK | CS_CTL1_DIVHS_MASK))); // set ACLK, source: REFOCLK, divider: 2^5=32
    while (!BITBAND_PERI(CS->STAT, CS_STAT_HSMCLK_READY_OFS));
    CS->KEY = 0x0000;
}
