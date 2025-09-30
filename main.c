/* main.c — Forward then Right Turn with OC-protect + 1s-hold reset (MSP430FR6989) */
#include <msp430.h>
#include <stdbool.h>
#include "movement.c"

/* ====== SELECT IN3 PIN ======
   1: use P1.3 as IN3 (recommended; your P2.5 is broken)
   0: keep original P2.5 as IN3 (input/pulldown)
*/
#define USE_IN3_ON_P13   1

/* --- PWM clocks --- */
#define F_SMCLK   (8000000UL)            /* SMCLK = 8 MHz */
#define F_PWM     (20000UL)              /* 20 kHz PWM */
#define CCR0VAL   ((F_SMCLK / F_PWM) - 1)
#define SEC       (F_SMCLK / 8UL)        /* ~1s helper with /8 assumption */

/* === Tunables === */
#define DRIVE_DUTY              800u     /* 0..CCR0VAL; ~70% */
#define FORWARD_SEC_X100        105u      /* forward for 0.80 s (tune for 1 ft) */
#define TURN_RIGHT_SEC_X100     100u      /* right turn for 0.60 s (tune for 90°) */
#define PAUSE_BETWEEN_SEC_X100  10u      /* brief coasting pause */

/* OC detection tuning */
#define OC_DEBOUNCE_TICKS       4u       /* require N consecutive trips */

/* Globals for OC logic & button */
volatile unsigned int channel_index = 0;   /* 0=C6, 1=C7 */
volatile unsigned int overCr_countA = 0;   /* current sense A (C6 / P8.5) */
volatile unsigned int overCr_countB = 0;   /* current sense B (C7 / P8.4) */
volatile unsigned int button_held   = 0;   /* ~100ms ticks → ~1s threshold */

/* Small delay helper: ~10ms ticks */
static void delay_0_01s(unsigned int ticks){
    while(ticks--){
        __delay_cycles(SEC / 100);
    }
}

/* Helpers (clamped to period) */
static inline void set_ena(unsigned int d){
    if (d > CCR0VAL) d = CCR0VAL;
    TB0CCR6 = d;                    /* ENA: P2.7 / TB0.6 */
}
static inline void set_enb(unsigned int d){
    if (d > CCR0VAL) d = CCR0VAL;
    TB0CCR2 = d;                    /* ENB: P3.6 / TB0.2 */
}

static void do_sequence(void){
    /* Forward */
    move(DRIVE_DUTY, FORWARD);
    delay_0_01s(FORWARD_SEC_X100);

    /* Pause/coast */
    move(0, REST);
    delay_0_01s(PAUSE_BETWEEN_SEC_X100);

    /* Right turn */
    turn(DRIVE_DUTY, RIGHT);
    delay_0_01s(TURN_RIGHT_SEC_X100);

    /* Stop/coast */
    move(0, REST);
    set_ena(0);
    set_enb(0);
}

int main(void) {
    WDTCTL  = WDTPW | WDTHOLD;

    /* ===== GPIO Direction (YOUR MAP) ===== */
    P1DIR |= BIT0; /* optional LED user */

    /* Direction pins: IN1=P2.2, IN2=P2.4, IN4=P2.3 outputs */
    P2DIR  |= BIT2 | BIT4 | BIT3;
    P2SEL0 &= ~(BIT2 | BIT4 | BIT3);
    P2SEL1 &= ~(BIT2 | BIT4 | BIT3);

#if USE_IN3_ON_P13
    /* IN3 on P1.3 as GPIO output */
    P1DIR  |= BIT3;
    P1SEL0 &= ~BIT3;
    P1SEL1 &= ~BIT3;
#else
    /* Original IN3 P2.5 broken → input + pulldown */
    P2DIR &= ~BIT5;
    P2REN |=  BIT5;
    P2OUT &= ~BIT5;
#endif

    /* ENA (P2.7) TB0.6 primary func */
    P2DIR  |= BIT7;
    P2SEL1 &= ~BIT7;
    P2SEL0 |=  BIT7;

    /* ENB (P3.6) TB0.2 alternate func */
    P3DIR  |= BIT6;
    P3SEL0 &= ~BIT6;
    P3SEL1 |=  BIT6;

    /* Overcurrent indicator LED + Button (S1=P1.1) */
    P9DIR |= BIT7;  P9OUT &= ~BIT7;              /* OC LED OFF */
    P1DIR &= ~BIT1; P1REN |= BIT1; P1OUT |= BIT1;/* S1 pull-up */

    /* Current sense inputs to Comparator_E (C7=P8.4, C6=P8.5) */
    P8SEL1 |= BIT4 | BIT5;
    P8SEL0 |= BIT4 | BIT5;

    PM5CTL0 &= ~LOCKLPM5;

    /* ===== Timer-B0 PWM ===== */
    TB0CTL   = TASSEL__SMCLK | MC__UP | TBCLR;
    TB0CCR0  = CCR0VAL;
    TB0CCTL6 = OUTMOD_7;   TB0CCR6 = 1;   /* ENA on TB0.6 */
    TB0CCTL2 = OUTMOD_7;   TB0CCR2 = 1;   /* ENB on TB0.2 */

    /* ===== Timers for OC polling & button hold ===== */
    TA0CTL   = TASSEL__SMCLK | MC__UP | TACLR;
    TA0CCR0  = (unsigned long)(SEC * 0.025);   /* ~25 ms tick (OC poll) */
    TA0CCTL0 = CCIE;

    TA1CTL   = TASSEL__SMCLK | MC__UP | TACLR;
    TA1CCR0  = (unsigned long)(SEC * 0.1);     /* ~100 ms tick (button) */
    TA1CCTL0 = CCIE;

    /* ===== Comparator E setup =====
       - Positive input mux starts on C6 (P8.5), we toggle C6<->C7 in TA0 ISR
       - Reference ladder ~0.45V (tune as needed for your sense scale)
    */
    CECTL0  = CEIPEN | CEIPSEL_6;               /* start on C6 */
    CECTL2  = CERSEL | CERS_2 | CEREFL_1 | CEREF1_12 | CEREF0_12;
    CECTL3 |= CEPD7 | CEPD6;                    /* disable input buffers on C7,C6 */
    CECTL1 |= CEON;
    __delay_cycles(500);

    __enable_interrupt();

    /* ===== Run the mission sequence ===== */
    do_sequence();

    /* Idle forever; press & hold S1 ~1s to reset & rerun */
    while(1){ __no_operation(); }
}

/* --- TA0 ISR: sample CEOUT, debounce OC, and toggle C6<->C7 --- */
#pragma vector = TIMER0_A0_VECTOR
__interrupt void TimerA0_ISR(void) {
    TA0CCTL0 &= ~CCIFG;

    /* If comparator output high, current > threshold on the active channel */
    if (CECTL1 & CEOUT) {
        if (channel_index == 0) {
            if (++overCr_countA >= OC_DEBOUNCE_TICKS) {
                P9OUT |= BIT7;  /* OC LED solid ON */
                shutdown();     /* stop PWMs, halt */
            }
        } else {
            if (++overCr_countB >= OC_DEBOUNCE_TICKS) {
                P9OUT |= BIT7;  /* OC LED solid ON */
                shutdown();
            }
        }
    } else {
        if (channel_index == 0) overCr_countA = 0;
        else                    overCr_countB = 0;
    }

    /* Toggle comparator positive input between C6 and C7 */
    if (channel_index == 0) {
        CECTL0 = (CECTL0 & ~0x000F) | CEIPEN | CEIPSEL_7;  /* now look at C7 */
        channel_index = 1;
    } else {
        CECTL0 = (CECTL0 & ~0x000F) | CEIPEN | CEIPSEL_6;  /* back to C6 */
        channel_index = 0;
    }
}

/* --- TA1 ISR: hold S1 ~1s -> watchdog reset --- */
#pragma vector = TIMER1_A0_VECTOR
__interrupt void Timer1A0_ISR(void) {
    TA1CCTL0 &= ~CCIFG;
    if (!(P1IN & BIT1)) {              /* button pressed (active low) */
        if (++button_held >= 10) {     /* ~10 * 100ms = ~1s */
            WDTCTL = 0;                /* cause BOR reset; program restarts */
        }
    } else {
        button_held = 0;
    }
}
