# LINK TO TEAM VIDEO
# https://

  .syntax unified
  .cpu cortex-m4
  .fpu softvfp
  .thumb
  
  .global Main
  .global SysTick_Handler
  .global EXTI0_IRQHandler
  .global current_LED
  .global correct_LED
  .global current_period

  @ Definitions are in definitions.s to keep this file "clean"
  .include "definitions.s"
  .include "subroutines.s"

  .equ    INVITATION_BLINK_PERIOD, 1000

  .section .text

Main:
  PUSH  {R4-R5,LR}

  LDR     R4, =SCB_ICSR               @ Clear any pre-existing interrupts for SysTick Timer
  LDR     R5, =SCB_ICSR_PENDSTCLR
  STR     R5, [R4]

  BL set_gpio_port_e_clock

  BL set_pins_for_output  @ LEDs are set up to illuminate light

  BL set_up_button

  @ Configure SysTick Timer to generate an interrupt every 1 ms to count time for seed.
  MOV R0, #1
  BL set_tick_period                  @ set_tick_period(1)


  @ Idle loop forever 
Idle_Loop:
  B     Idle_Loop
  

@ @  On Falure blinks all LEDs for the amount of wins
@ On_Fail:
@   PUSH  {R0-R5}
@   LDR	  R0, =Level
@   MOV   R1, #0
@   MOV   R2, #8
@   MOV   R3, #1
@   @ Invert LD3
@   @ by inverting bit 13 of GPIOE_ODR (GPIO Port E Output Data Register)
@   @ (by using EOR to invert bit 13, leaving other bits unchanged)
@ .failure_blink
@   CMP     R0, R1
@   BEQ     end_failure
@   ADD     R1, R1, #1

@   LDR     R4, =GPIOE_ODR
@   LDR     R5, [R4]                  @ Read ...
@   EOR     R5, R5, R3, LSL R2        @ Modify ...
@   STR     R5, [R4]                  @ Write
@   ADD     R2, R2, #1
@   CMP     R2, #15
@   BGT     reset_fail_leds
@   @ wait for 1s ...
@   LDR     R0, =BLINK_PERIOD
@   BL      delay_ms

@   @ ... and repeat
@   B       .failure_blink
@ reset_fail_leds:
@   MOV     R2, #8
@   B       .failure_blink
@ end_failure:



End_Main:
  POP   {R4-R5,PC}



@
@ SysTick interrupt handler
@
  .type  SysTick_Handler, %function
SysTick_Handler:
  PUSH  {R4, R5, LR}

  LDR   R4, =total_ms               @ total_ms = total_ms + current_period
  LDR   R5, [R4]
  LDR   R6, =current_period
  LDR   R7, [R6]
  ADD   R5, R5, R7
  STR   R5, [R4]


@   LDR   R4, =blink_countdown        @ if (countdown != 0) {
@   LDR   R5, [R4]                    @
@   CMP   R5, #0                      @
@   BEQ   .LelseFire                  @

@   SUB   R5, R5, #1                  @   countdown = countdown - 1;
@   STR   R5, [R4]                    @

@   B     .LendIfDelay                @ }

@ .LelseFire:                         @ else {

@   LDR     R4, =GPIOE_ODR            @   Invert LD3
@   LDR     R5, [R4]                  @
@   EOR     R5, #(0b1<<(LD3_PIN))     @   GPIOE_ODR = GPIOE_ODR ^ (1<<LD3_PIN);
@   STR     R5, [R4]                  @ 

@   LDR     R4, =blink_countdown      @   countdown = BLINK_PERIOD;
@   LDR     R5, =BLINK_PERIOD         @
@   STR     R5, [R4]                  @

@ .LendIfDelay:                       @ }

@   LDR     R4, =SCB_ICSR             @ Clear (acknowledge) the interrupt
@   LDR     R5, =SCB_ICSR_PENDSTCLR   @
@   STR     R5, [R4]                  @

  @ Return from interrupt handler
  POP  {R4-R7, PC}



@ @
@ @ External interrupt line 0 interrupt handler
@ @   (count button presses)
@ @
@   .type  EXTI0_IRQHandler, %function
@ EXTI0_IRQHandler:

@   PUSH  {R4,R5,LR}

@   LDR   R4, =button_count           @ count = count + 1
@   LDR   R5, [R4]                    @
@   ADD   R5, R5, #1                  @
@   STR   R5, [R4]                    @

@   LDR   R4, =EXTI_PR                @ Clear (acknowledge) the interrupt
@   MOV   R5, #(1<<0)                 @
@   STR   R5, [R4]                    @

@   @ Return from interrupt handler
@   POP  {R4,R5,PC}


@ @     Takes R0 as number of wins
@ blink_for_each_win:
@   MOV   R1, #0
@ for_win_loop:
@   CMP   R1, R0
@   BGE   end_game

@ @ Blinking Code
  


@   ADD   R1, R1, #1
@   B for_win_loop

@ ===============================  Global data ===============================
  .section .data
button_count:
  .space  4

blink_countdown:
  .space  4

@ TODO: change to 1 byte
@ unsigned int 4 bytes range 8-15
current_LED:
  .space  4

@ unsigned int 4 bytes range 8-15
correct_LED:
  .space  4

@ time in ms since start of discovery.
total_ms:
  .space 4

@ PRIVATE: TO BE SET ONLY AFTER INSIDE set_tick_period
current_period:
  .space 4

@ program_stage (1 byte) (u8)
@
@ enum: The current stage of the program
@ Possible values:
@   0: WAITING_FOR_SEED
@   1: GAME_SKIPPING_SOME_TIME
@   2: GAME_READY_FOR_INPUT
@   3: GAME_FINISHED
  .equ WAITING_FOR_SEED, 0
  .equ GAME_READY_FOR_INPUT, 2
  .equ GAME_FINISHED, 3
program_stage:
  .space 1

  .end