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
  PUSH  {R4-R8,LR}

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


  .equ DIMMING_LOWER_BOUNDARY, 1000
  .equ DIMMING_HIGHER_BOUNDARY, 1150
  @ Really simple and not precise DIMMING forever loop
  @ if you want to disable dimming then comment out the following code and replace it with
  @ Idle_Loop:
  @   B     Idle_Loop
  @ also you need to uncomment a line inside Sys_Tick_Handler if you want to disable dimming.

MOV R6, #0                          @ time_counter = 0;
Dimming_Loop:                       @ while (true)  {
  LDR   R4, =program_stage          @     program_stage: enum = load_byte(program_stage_ptr);
  LDR   R5, [R4]    
  CMP   R5, #GAME_ONGOING           @     if (program_stage == WAITING_FOR_SEED)
  BNE Dimming_Loop                  @         continue;

  LDR     R4, =correct_LED
  LDR     R5, [R4]
  LDR     R7, =current_LED          @     // Avoid conflict with setting up current LED
  LDR     R8, [R7]                  @     if (correct_LED == current_LED)
  CMP     R5, R8                    @     {
  BNE     .Dimming_Not_The_Same      @
  MOV     R0, R4                    @
  BL      turn_on_led               @         turn_on_led(correct_LED);
  B       Dimming_Loop              @         continue;
  @                                 @     }
.Dimming_Not_The_Same:
  ADD R6, R6, #1                    @     time_counter += 1;

  LDR R4, =DIMMING_LOWER_BOUNDARY
  CMP R6, R4                        @     if (time_counter == DIMMING_LOWER_BOUNDARY)
  BEQ .Enable_Dimmed_Led            @         goto .Enable_Dimmed_Led;                    

  LDR R4, =DIMMING_HIGHER_BOUNDARY  @     else if (time_counter < DIMMING_LOWER_BOUNDARY)
  CMP R6, R4                        @         continue;
  BLO Dimming_Loop

.Disable_Dimmed_Led:
  LDR   R4, =correct_LED
  LDR   R5, [R4]
  MOV R0, R5
  BL turn_off_led                   @     turn_off_led(correct_LED);
  MOV R6, #0                        @     time_counter = 0; // reset
  B Dimming_Loop                    @     continue;

.Enable_Dimmed_Led:                 @   .Enable_Dimmed_Led:
  LDR   R4, =correct_LED
  LDR   R5, [R4]
  MOV R0, R5
  BL turn_on_led                    @     turn_on_led(correct_LED);

  B Dimming_Loop                    @ }
  

  // Unreachable
End_Main:
  POP   {R4-R8,PC}


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

  LDR     R8, =correct_LED
  LDR     R9, [R8]
  LDR     R6, =current_LED
  LDR     R7, [R6]

  @ CMP     R7, R9                    @ Avoid blink conflic
  @ BEQ     continue

  MOV     R0, R7
  BL      turn_on_led                @ turn_on_led(current_LED)

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
  BL on_fail
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