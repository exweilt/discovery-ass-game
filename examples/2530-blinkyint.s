  .syntax unified
  .cpu cortex-m4
  .fpu softvfp
  .thumb
  
  .global  Main
  .global  SysTick_Handler

  @ Definitions are in definitions.s to keep blinky.s "clean"
  .include "definitions.s"

  .equ    BLINK_PERIOD, 1000

  .section .text

Main:
  PUSH    {R4-R5,LR}

  @ Enable GPIO port E by enabling its clock
  @ STM32F303 Reference Manual 9.4.6 (pg. 148)
  LDR     R4, =RCC_AHBENR
  LDR     R5, [R4]
  ORR     R5, R5, #(0b1 << (RCC_AHBENR_GPIOEEN_BIT))
  STR     R5, [R4]

  @ We'll blink LED LD3 (the orange LED)

  @ Configure LD3 for output
  @ by setting bits 27:26 of GPIOE_MODER to 01 (GPIO Port E Mode Register)
  @ (by BIClearing then ORRing)
  @ STM32F303 Reference Manual 11.4.1 (pg. 237)
  LDR     R4, =GPIOE_MODER
  LDR     R5, [R4]                  @ Read ...
  BIC     R5, #(0b11<<(LD3_PIN*2))  @ Modify ...
  ORR     R5, #(0b01<<(LD3_PIN*2))  @ write 01 to bits 
  STR     R5, [R4]                  @ Write 

  @ We'll blink LED LD3 (the orange LED) every 1s
  @ Initialise the first countdown to 1000 (1000ms)
  LDR     R4, =countdown
  LDR     R5, =BLINK_PERIOD
  STR     R5, [R4]  


  @ Configure SysTick Timer to generate an interrupt every 1ms

  @ STM32 Cortex-M4 Programming Manual 4.4.3 (pg. 225)
  LDR     R4, =SCB_ICSR             @ Clear any pre-existing interrupts
  LDR     R5, =SCB_ICSR_PENDSTCLR   @
  STR     R5, [R4]                  @

  @ STM32 Cortex-M4 Programming Manual 4.5.1 (pg. 247)
  LDR   R4, =SYSTICK_CSR            @ Stop SysTick timer
  LDR   R5, =0                      @   by writing 0 to CSR
  STR   R5, [R4]                    @   CSR is the Control and Status Register
  
  @ STM32 Cortex-M4 Programming Manual 4.5.2 (pg. 248)
  LDR   R4, =SYSTICK_LOAD           @ Set SysTick LOAD for 1ms delay
  LDR   R5, =7999                   @ Assuming 8MHz clock
  STR   R5, [R4]                    @ 

  @ STM32 Cortex-M4 Programming Manual 4.5.3 (pg. 249)
  LDR   R4, =SYSTICK_VAL            @   Reset SysTick internal counter to 0
  LDR   R5, =0x1                    @     by writing any value
  STR   R5, [R4]

  @ STM32 Cortex-M4 Programming Manual 4.4.3 (pg. 225)
  LDR   R4, =SYSTICK_CSR            @   Start SysTick timer by setting CSR to 0x7
  LDR   R5, =0x7                    @     set CLKSOURCE (bit 2) to system clock (1)
  STR   R5, [R4]                    @     set TICKINT (bit 1) to 1 to enable interrupts
                                    @     set ENABLE (bit 0) to 1

  @ Nothing else to do in Main
  @ Idle loop forever (welcome to interrupts!!)
Idle_Loop:
  B     Idle_Loop
  
End_Main:
  POP   {R4-R5,PC}


@
@ SysTick interrupt handler
@
  .type  SysTick_Handler, %function
SysTick_Handler:

  PUSH  {R4, R5, LR}

  LDR   R4, =countdown              @ if (countdown != 0) {
  LDR   R5, [R4]                    @
  CMP   R5, #0                      @
  BEQ   .LelseFire                  @

  SUB   R5, R5, #1                  @   countdown = countdown - 1;
  STR   R5, [R4]                    @

  B     .LendIfDelay                @ }

.LelseFire:                         @ else {

  @ STM32F303 Reference Manual 11.4.6 (pg. 239)
  LDR     R4, =GPIOE_ODR            @   Invert LD3
  LDR     R5, [R4]                  @
  EOR     R5, #(0b1<<(LD3_PIN))     @   GPIOE_ODR = GPIOE_ODR ^ (1<<LD3_PIN);
  STR     R5, [R4]                  @ 

  LDR     R4, =countdown            @   countdown = BLINK_PERIOD;
  LDR     R5, =BLINK_PERIOD  @
  STR     R5, [R4]                  @

.LendIfDelay:                       @ }

  @ STM32 Cortex-M4 Programming Manual 4.4.3 (pg. 225)
  LDR     R4, =SCB_ICSR             @ Clear (acknowledge) the interrupt
  LDR     R5, =SCB_ICSR_PENDSTCLR   @
  STR     R5, [R4]                  @

  @ Return from interrupt handler
  POP  {R4, R5, PC}


  .section .data

countdown:
  .space  4

  .end
