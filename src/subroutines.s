  .syntax unified
  .cpu cortex-m4
  .fpu softvfp
  .thumb
  
  .global   process_invitation
  .global   random

    .include "definitions.s"
    

@   Process Invitation
@   
process_invitation:
    PUSH {LR}

    POP {PC}


@   Random - generate random integer.
@   
@   Args:
@       R0 - seed (unsigned int) 32 bits
@       R1 - lower range boundary (unsigned int) 32 bits
@       R2 - upper range boundary (unsigned int) 32 bits
@
@   Returns:
@       R0 - random unsigned integer between R1 and R2 (unsigned int) 32 bits
@   
random:
    PUSH {LR}

    POP {PC}

bliking:
  PUSH  {R4-R12,LR}                     @ LED code in R1

  LDR     R4, =GPIOE_ODR
  LDR     R5, [R4]                      @ Read

  MOV     R6, #0b1                      @ Access LED
  LSL     R7, R6, R1

  EOR     R5, R7                        @ Modify
  STR     R5, [R4]                      @ Write

  BL      STICK_TIMER

  POP  {R4-R12,PC}

  @ Need Access to Stick Timer

clockwise_bliking:
  PUSH  {R4-R12,LR}			@

.reset:

  MOV	  R4, #7

.loop:

  ADD	  R4, R4, #1
  CMP	  R4, #16
  BEQ	  reset

  MOV     R1, R4
  STR 	  R1, =current_LED		@ Update current_LED
  BL 	  bliking			@ Open
  BL 	  bliking   			@ Close

  B  	  loop

  POP  {R4-R12,PC}

level_up:
  PUSH  {R4-R12,LR}			@

  LDR	  R0, =Level
  ADD	  R0, R0, #1			@ Level Up
  STR	  R0, =Level

  LDR	  R0, =Time
  SUB	  R0, R0, #200			@ Reduce by 200 milisec
  STR	  R0, =Time

  BL	  clockwise_bliking

  POP  {R4-R12,PC}

EXTI0_IRQHandler:

  PUSH  {R4,R5,LR}			@ Return R0 1 TRUE, 0 FALSE

  LDR     R4, =current_LED        	@ Check current LED and correct LED
  LDR     R5, =correct_LED
  CMP	  R4, R5
  BEQ	  end

  MOV 	  R0, #0

.end:

  MOV 	  R0, #1

  LDR   R4, =EXTI_PR      		@ Clear (acknowledge) the interrupt
  MOV   R5, #(1<<0)       		
  STR   R5, [R4]          		

  POP  {R4,R5,PC}




  .end


@
@ void set_pins_for_output()
@
@ Set every LED pin for output 
@
set_pins_for_output:
  PUSH {R4, R5}
  LDR     R4, =GPIOE_MODER
  LDR     R5, [R4]                                   @ Read ...
  BIC     R5, #0b11111111111111110000000000000000    @ clear 8 LEDs
  ORR     R5, #0b01010101010101010000000000000000    @ 01 for each LED 
  STR     R5, [R4]                                   @ Write 

  POP {R4, R5}
  BX LR

@
@ void set_gpio_port_e_clock()
@
@ Set GPIO port E clock on
@
set_gpio_port_e_clock:
  PUSH {R4, R5}
  
  LDR     R4, =RCC_AHBENR
  LDR     R5, [R4]
  ORR     R5, R5, #(0b1 << (RCC_AHBENR_GPIOEEN_BIT))
  STR     R5, [R4]

  BX LR


@ delay_ms subroutine
@ Use the Cortex SysTick timer to wait for a specified number of milliseconds
@
@ See Yiu, Joseph, "The Definitive Guide to the ARM Cortex-M3 and Cortex-M4
@   Processors", 3rd edition, Chapter 9.
@
@ Parameters:
@   R0: delay - time to wait in ms
@
@ Return:
@   None
delay_ms:
  PUSH  {R4-R5,LR}

  LDR   R4, =SYSTICK_CSR            @ Stop SysTick timer
  LDR   R5, =0                      @   by writing 0 to CSR
  STR   R5, [R4]                    @   CSR is the Control and Status Register
  
  LDR   R4, =SYSTICK_LOAD           @ Set SysTick LOAD for 1ms delay
  LDR   R5, =7999                   @ Assuming a 8MHz clock
  STR   R5, [R4]                    @ 
  
  LDR   R4, =SYSTICK_VAL            @ Reset SysTick internal counter to 0
  LDR   R5, =0x1                    @   by writing any value
  STR   R5, [R4]  

  LDR   R4, =SYSTICK_CSR            @ Start SysTick timer by setting CSR to 0x5
  LDR   R5, =0x5                    @   set CLKSOURCE (bit 2) to system clock (1)
  STR   R5, [R4]                    @   set ENABLE (bit 0) to 1

.LwhDelay:                          @ while (delay != 0) {
  CMP   R0, #0  
  BEQ   .LendwhDelay  
  
.Lwait:
  LDR   R5, [R4]                    @   Repeatedly load the CSR and check bit 16
  AND   R5, #0x10000                @   Loop until bit 16 is 1, indicating that
  CMP   R5, #0                      @     the SysTick internal counter has counted
  BEQ   .Lwait                      @     from 0x3E7F down to 0 and 1ms has elapsed 

  SUB   R0, R0, #1                  @   delay = delay - 1
  B     .LwhDelay                   @ }

.LendwhDelay:

  LDR   R4, =SYSTICK_CSR            @ Stop SysTick timer
  LDR   R5, =0                      @   by writing 0 to CSR
  STR   R5, [R4]                    @   CSR is the Control and Status Register
  
  POP   {R4-R5,PC}

  .end