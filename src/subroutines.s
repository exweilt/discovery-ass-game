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

@   Blinking - open/close LED
@   
@   Need input R1 (LED Number)
@
@   R4 Read GPIOE_ODR
@   R5 Store GPIOE_ODR and make change
@
@   R6 R7 Calculations to change GPIOE_ODR

bliking:
  PUSH  {R4-R12,LR}                     @ LED code in R1

  LDR     R4, =GPIOE_ODR
  LDR     R5, [R4]                      @ Read

  MOV     R6, #0b1                      @ Access LED
  LSL     R7, R6, R1

  EOR     R5, R7                        @ Modify
  STR     R5, [R4]                      @ Write

  BL      STICK_TIMER
  @ Need Access to Stick Timer (To check pause time)

  POP  {R4-R12,PC}

@   Clockwise Blinking - Blink in Clockwise
@
@   No input need
@
@   Use Blinking subroutine
@
@   R4 Current working LED
@   R1 Update to current LED (To check Level (Level subroutine))
@

clockwise_bliking:
  PUSH  {R4-R12,LR}

.reset:

  MOV	  R4, #7

.loop:

  ADD	  R4, R4, #1
  CMP	  R4, #16
  BEQ	  reset

  MOV   R1, R4
  STR 	R1, =current_LED		@ Update current_LED
  BL 	  bliking			        @ Open
  BL 	  bliking   			    @ Close

  B  	  loop

  POP  {R4-R12,PC}

@   Level - Level Up Count
@
@   No input need
@
@   R0 Load Level and add 1 Level
@   R0 Load Time breack and reduce by 200ms
@

level_up:
  PUSH  {R4-R12,LR}

  LDR	  R0, =Level
  ADD	  R0, R0, #1			@ Level Up
  STR	  R0, =Level

  LDR	  R0, =Time
  SUB	  R0, R0, #200			@ Reduce by 200 milisec
  STR	  R0, =Time

  BL	  clockwise_bliking

  POP  {R4-R12,PC}

@   EXTIO_IRQHandler - Check if you lose or win the round
@
@   No input need
@
@   The Exception Handler will automaticly called if bottom pressed
@   R4 current LED load (save from Clockwise Blinking)
@   R5 correct LED load (save from Random number)
@   R0 R4 R5 For reset the Exception Handler
@

EXTI0_IRQHandler:

  PUSH  {R4,R5,LR}			            @ Return R0 1 TRUE, 0 FALSE

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