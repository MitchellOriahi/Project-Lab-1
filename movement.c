#include <msp430.h>

/* ===== SELECT IN3 PIN (must match main.c) ===== */
#define USE_IN3_ON_P13   1

/* --- Pin Assignments ---
   ENA  -> P2.7  (TB0.6 -> TB0CCR6)
   ENB  -> P3.6  (TB0.2 -> TB0CCR2)
   IN1  -> P2.2
   IN2  -> P2.4
   IN4  -> P2.3
   IN3  -> (P1.3 if USE_IN3_ON_P13==1) else P2.5 (broken/input)
*/

typedef enum { FORWARD, REVERSE, STOP, REST } MoveDir;
typedef enum { RIGHT, LEFT } TurnDir;

#define move1(arg1)           move(0, arg1)
#define move2(arg1, arg2)     move(arg1, arg2)
#define _move_select(_1,_2,NAME,...) NAME
#define move(...) _move_select(__VA_ARGS__, move2, move1)(__VA_ARGS__)

static void shutdown(void);
static void move(int duty, MoveDir dir);
static void turn(int duty, TurnDir dir);

/* === Public helpers (used by main) ======================================= */
__attribute__((unused)) static void shutdown(void){
    TB0CCR6 = 0;   /* ENA (TB0.6) */
    TB0CCR2 = 0;   /* ENB (TB0.2) */
    /* Leave H-bridge inputs in braking state for safety */
    P2OUT |= (BIT2 | BIT4);   /* A brake: IN1=1, IN2=1 */
#if USE_IN3_ON_P13
    P1OUT |=  BIT3;           /* B brake: IN3=1 */
#endif
    P2OUT |=  BIT3;           /* IN4=1 */
    while (1) { __no_operation(); }  /* halt here; S1 hold resets */
}

/* Drive IN3 regardless of pin choice */
static inline void in3_low(void){
#if USE_IN3_ON_P13
    P1OUT &= ~BIT3;
#else
    /* P2.5 broken/input; left low by pull-down in main */
#endif
}
static inline void in3_high(void){
#if USE_IN3_ON_P13
    P1OUT |=  BIT3;
#else
    /* cannot drive high on broken P2.5 */
#endif
}

/* === Motion primitives ==================================================== */
__attribute__((unused)) static void move(int duty, MoveDir dir){
    switch (dir) {
        case FORWARD:
            TB0CCR6 = duty;  /* A duty */
            TB0CCR2 = duty;  /* B duty */
            /* Forward = IN1=0, IN2=1, IN3=0, IN4=1 */
            P2OUT &= ~BIT2;        /* IN1 low  */
            P2OUT |=  BIT4;        /* IN2 high */
            in3_low();             /* IN3 low  */
            P2OUT |=  BIT3;        /* IN4 high */
            break;

        case REVERSE:
            TB0CCR6 = duty;
            TB0CCR2 = duty;
#if USE_IN3_ON_P13
            /* True reverse both: IN1=1, IN2=0, IN3=1, IN4=0 */
            P2OUT |=  BIT2;        /* IN1 high */
            P2OUT &= ~BIT4;        /* IN2 low  */
            in3_high();            /* IN3 high */
            P2OUT &= ~BIT3;        /* IN4 low  */
#else
            /* With broken IN3, approximate reverse */
            P2OUT |=  BIT2;
            P2OUT &= ~BIT4;
            in3_low();
            P2OUT &= ~BIT3;
#endif
            break;

        case REST: /* float: PWMs off, all low */
            TB0CCR6 = 0;
            TB0CCR2 = 0;
            P2OUT &= ~(BIT2 | BIT4 | BIT3);
            in3_low();
            break;

        case STOP: /* brake both sides where possible */
            TB0CCR6 = 0;
            TB0CCR2 = 0;
            P2OUT |= (BIT2 | BIT4);  /* A brake */
#if USE_IN3_ON_P13
            in3_high();              /* B brake */
            P2OUT |=  BIT3;
#else
            in3_low();
            P2OUT |=  BIT3;
#endif
            break;
    }
}

/* Pivot turn primitives (timed in main) */
__attribute__((unused)) static void turn(int duty, TurnDir dir){
    switch (dir) {
        case RIGHT:
            TB0CCR6 = duty;   /* A on */
            TB0CCR2 = duty;   /* B on */
#if USE_IN3_ON_P13
            /* RIGHT pivot: A forward, B reverse */
            P2OUT &= ~BIT2;    /* IN1=0 */
            P2OUT |=  BIT4;    /* IN2=1 */
            in3_high();        /* IN3=1 */
            P2OUT &= ~BIT3;    /* IN4=0 */
#else
            /* Fallback: both forward */
            P2OUT &= ~BIT2;
            P2OUT |=  BIT4;
            in3_low();
            P2OUT |=  BIT3;
#endif
            break;

        case LEFT:
            TB0CCR6 = duty;
            TB0CCR2 = duty;
#if USE_IN3_ON_P13
            /* LEFT pivot: A reverse, B forward */
            P2OUT |=  BIT2;    /* IN1=1 */
            P2OUT &= ~BIT4;    /* IN2=0 */
            in3_low();         /* IN3=0 */
            P2OUT |=  BIT3;    /* IN4=1 */
#else
            P2OUT |=  BIT2;
            P2OUT &= ~BIT4;
            in3_low();
            P2OUT |=  BIT3;
#endif
            break;
    }
}
