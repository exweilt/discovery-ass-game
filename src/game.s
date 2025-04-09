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

  .extern set_gpio_port_e_clock
  .extern set_pins_for_output
  .extern set_up_button

  @ Definitions are in definitions.s to keep this file "clean"
  .include "definitions.s"
  .include "subroutines.s"


  .section .text

Main:
  PUSH  {R4-R6,LR}

  LDR     R4, =SCB_ICSR               @ Clear any pre-existing interrupts for SysTick Timer
  LDR     R5, =SCB_ICSR_PENDSTCLR
  STR     R5, [R4]

  LDR R4, =current_LED
  LDR R5, =8
  STR R5, [R4]


  BL set_gpio_port_e_clock

  BL set_pins_for_output  @ LEDs are set up to illuminate light

  BL set_up_button

  LDR     R4, =GPIOE_ODR
  LDR     R6, =0b1010101000000000
  STR     R6, [R4]                      @ Write

  @ @ Configure SysTick Timer to generate an interrupt every 1 ms to count time for seed.
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
  POP   {R4-R6,PC}



@
@ SysTick interrupt handler
@
  .type  SysTick_Handler, %function
SysTick_Handler:
  PUSH  {R4-R8, LR}

  LDR   R4, =total_ms               @ total_ms = total_ms + current_period
  LDR   R5, [R4]
  LDR   R6, =current_period
  LDR   R7, [R6]
  ADD   R5, R5, R7
  STR   R5, [R4]

  LDR   R4, =program_stage          @ program_stage: enum = load_byte(program_stage_ptr)
  LDR   R5, [R4]    
  CMP   R5, #WAITING_FOR_SEED       @ if (program_stage == WAITING_FOR_SEED)
  BNE   .tick.not_waiting_for_seed       @ {

  LDR   R4, =invitation_timer
  LDR   R5, [R4]
  ADD   R5, R5, R7
  STR   R5, [R4]
  CMP   R5, #1000
  BLO   .skip_updating_screen
  MOV   R5, #0
  STR   R5, [R4]
  LDR     R4, =GPIOE_ODR
  LDR     R5, [R4]                      @ Read ...
  LDR     R8, =0b1111111100000000
  EOR     R5, R8                        @ Modify ...
  STR     R5, [R4]                      @ Write
.skip_updating_screen:

  B .tick.finish_handling_button          @ }

.tick.not_waiting_for_seed:
  @                                 @ else if (program_stage == GAME_ONGOING)
  @                                 @ {
  BL      turn_off_all_led
  LDR     R4, =GPIOE_ODR
  LDR     R5, [R4]                      @ Read ...

  BL move_to_next_led

  LDR     R6, =current_LED
  LDR     R7, [R6]
  MOV     R0, R7
  BL      turn_on_led

  LDR     R6, =correct_LED
  LDR     R7, [R6]
  MOV     R0, R7
  BL      turn_on_led



  @                                 @ }


.tick.finish_handling_button:


  LDR     R4, =SCB_ICSR             @ Clear (acknowledge) the interrupt
  LDR     R5, =SCB_ICSR_PENDSTCLR   @
  STR     R5, [R4]                  @

  @ Return from interrupt handler
  POP  {R4-R8, PC}


@
@ External interrupt line 0 (EXTI0) interrupt handler
@
  .type  EXTI0_IRQHandler, %function
EXTI0_IRQHandler:
  PUSH  {R4-R10,LR}

  LDR   R4, =program_stage          @ program_stage: enum = load_byte(program_stage_ptr)
  LDR   R5, [R4]    
  CMP   R5, #WAITING_FOR_SEED       @ if (program_stage == WAITING_FOR_SEED)
  BNE   .not_waiting_for_seed       @ {
  LDR   R7, =total_ms               @     <$r8>total_ms = *total_ms_ptr
  LDR   R8, [R7]
  LDR   R6, =seed                  @     
  STR   R8, [R6]                    @     *seed_ptr = total_ms 
  BL    set_next_target             @     set_next_target()
  LDR   R8, =GAME_ONGOING           @
  STR   R8, [R4]                    @     *program_stage_ptr = GAME_ONGOING
  MOV   R0, #500
  BL    set_tick_period             @     set_tick_period(1000)     

  B .finish_handling_button          @ }

.not_waiting_for_seed:
  @                                 @ else if (program_stage == GAME_ONGOING)
  @                                 @ {
  LDR R6, =current_LED
  LDR R7, [R6]
  LDR R8, =correct_LED
  LDR R9, [R8]
  CMP R7, R9
  BNE .miss
.hit:
  BL set_next_target
  BL increase_level
  B .finish_handling_button

.miss:
  @                                 @ }
  BL reset_game

.finish_handling_button:
  @ Tell microcontroller that we have handled the EXTI0 interrupt
  @ By writing a 1 to bit 0 of the EXTI Pending Register (PR)
  @ (Writing 0s to bits has no effect)
  @ STM32F303 Reference Manual 14.3.6 (pg. 299)
  LDR   R4, =EXTI_PR      @ Clear (acknowledge) the interrupt
  MOV   R5, #(1<<0)       @
  STR   R5, [R4]          @
  @ Return from interrupt handler
  POP  {R4-R10,PC}


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

.equ MAX_SEED_VALUE, 4294967295
seed:
  .space 4

button_count:
  .space  4

blink_countdown:
  .space  4

.equ    INVITATION_BLINK_PERIOD, 1000
invitation_timer:
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

@ program_stage (4 bytes) (u8)
@
@ enum: The current stage of the program
@ Possible values:
@   0: WAITING_FOR_SEED
@   1: GAME_SKIPPING_SOME_TIME
@   2: GAME_ONGOING
@   3: GAME_FINISHED
  .equ WAITING_FOR_SEED, 0
  .equ GAME_ONGOING, 2
  .equ GAME_FINISHED, 3
program_stage:
  .space 4

  .equ MAX_LEVEL, 29
levels:
  .word 500, 450, 400, 350, 310, 280, 260, 240, 220, 200
  .word 180, 160, 150, 140, 130, 125, 120, 115, 110, 105
  .word 100, 95,  90,  85,  80,  75,  70,  65,  60,  50

level:
  .zero 4

  .end