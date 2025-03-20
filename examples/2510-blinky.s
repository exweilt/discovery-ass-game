  .syntax unified
  .cpu cortex-m4
  .fpu softvfp
  .thumb

  .global Main

  @ Definitions are in support/definitions.s to keep blinky.s "clean"
  .include "definitions.s"


@
@ You must have a STM32F3Discovery board connected to a USB
@ port of your computer for this example to work!!
@

  .section .text

Main:
  PUSH    {R4-R5,LR}

  @ Enable GPIO port E by enabling its clock
  LDR     R4, =RCC_AHBENR
  LDR     R5, [R4]
  ORR     R5, R5, #(0b1 << (RCC_AHBENR_GPIOEEN_BIT))
  STR     R5, [R4]

  @ We'll blink LED LD3

  @ Configure LD3 for output
  @ by setting bits 19:18 of GPIOE_MODER to 01 (GPIO Port E Mode Register)
  @ (by BIClearing then ORRing)
  LDR     R4, =GPIOE_MODER
  LDR     R5, [R4]                      @ Read ...
  BIC     R5, #(0b11<<(LD3_PIN*2))      @ Modify ...
  ORR     R5, #(0b01<<(LD3_PIN*2))      @ write 01 to bits 
  STR     R5, [R4]                      @ Write 

  @ Loop forever
.LwhBlink:
  @ Invert LD3
  @ by inverting bit 9 of GPIOE_ODR (GPIO Port E Output Data Register)
  @ (by using EOR to invert bit 9, leaving other bits unchanged)
  LDR     R4, =GPIOE_ODR
  LDR     R5, [R4]                      @ Read ...
  EOR     R5, #(0b1<<(LD3_PIN))         @ Modify ...
  STR     R5, [R4]                      @ Write

  @ wait for 1s ...
  LDR     R5, =500000     @ Assuming 8MHz clock, 4 cycles per iteration
                          @ (SUBS + BNE + 2 stall cycles for branch)
.Lwhwait:
  SUBS    R5, R5, #1      @ Keep looping until we count down to zero
  BNE     .Lwhwait  

  @ ... and repeat
  B       .LwhBlink
  
End_Main:
  POP   {R4-R5,PC}


  .end
