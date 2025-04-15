# LINK TO TEAM VIDEO
# https://

  .syntax unified
  .cpu cortex-m4
  .fpu softvfp
  .thumb
  
  .global Main
  .global SysTick_Handler
  .global EXTI0_IRQHandler

  @ Definitions are in definitions.s to keep this file "clean"
  .include "definitions.s"
 @ .include "subroutines.s"  @ we stored all subroutines in a separate file during development

  .section .text

@ ============================================================================
@ =============================== The Program ================================
@ ============================================================================

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


@ ============================================================================
@ =============================== Subroutines ================================
@ ============================================================================

@   Random Number Subroutine - generate random integer.
@   Args:
@     R0: Pointer to the current seed (uint32_t*)
@     R1: Minimum value (inclusive)
@     R2: Maximum value (inclusive)
@   Returns:
@     R0: Random unsigned integer 

random_int:
    PUSH    {R3-R5, LR}       

    @ Formula used is: new_seed = (a * seed + c) mod 2^32
    @ Constants (a=1664525, c=1013904223)
    LDR     R3, [R0]          @ Load current seed into R3
    LDR     R4, =1664525      @ Load multiplier (a)
    MUL     R3, R3, R4        @ R3 = seed * a
    LDR     R4, =1013904223   @ Load increment (c)
    ADD     R3, R3, R4        @ R3 = new_seed (a*seed + c)
    STR     R3, [R0]          @ Store updated seed back to memory

    SUB     R4, R2, R1        @ R4 = max - min
    ADD     R4, R4, #1        @ R4 = range_size (max - min + 1)

    UDIV    R5, R3, R4        @ R5 = new_seed / range_size (quotient)
    MUL     R4, R4, R5        @ R4 = (range_size * quotient) 
    SUB     R0, R3, R4        @ R0 = new_seed - (range_size * quotient) = remainder
    ADD     R0, R0, R1        @ R0 = remainder + min (final result)

    POP     {R3-R5, PC}       @ Restore registers and return


@
@ void set_pins_for_output()
@
@ Set every LED pin for output 
@
set_pins_for_output:
  PUSH {R4-R6, LR}
  LDR     R4, =GPIOE_MODER
  LDR     R5, [R4]                                   @ Read ...
  LDR     R6, =0b11111111111111110000000000000000
  BIC     R5, R6                                     @ clear 8 LEDs
  LDR     R6, =0b01010101010101010000000000000000
  ORR     R5, R6                                     @ 01 for each LED 
  STR     R5, [R4]                                   @ Write 

  POP {R4-R6, PC}

@
@ void set_gpio_port_e_clock()
@
@ Set GPIO port E clock on
@
set_gpio_port_e_clock:
  PUSH {R4, R5, LR}
  
  LDR     R4, =RCC_AHBENR
  LDR     R5, [R4]
  ORR     R5, R5, #(0b1 << (RCC_AHBENR_GPIOEEN_BIT))
  STR     R5, [R4]

  POP {R4, R5, PC}


@ WARNGING: (DEPRECATED) -> needs to become compatible with SysTick timer
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

  LDR   R4, =current_period
  STR   R0, [R4]

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

@ =============================================================================================

@ set_tick_period(u32 period)
@
@ Updates the SysTick calling interval with given period and calls SysTick_Handler instantly.
@ 
@ Parameters:
@   R0: period - time to wait in ms before SysTick_Handler calling intervals.
@
@ Return:
@   None
set_tick_period:
  PUSH  {R4-R5,LR}

  @ Dont understand what the following does so commented out
  @ LDR   R4, =SCB_ICSR               @ Clear any pre-existing interrupts
  @ LDR   R5, =SCB_ICSR_PENDSTCLR     @
  @ STR   R5, [R4]

  LDR   R4, =current_period
  STR   R0, [R4]

  LDR   R4, =SYSTICK_CSR            @ Stop SysTick timer
  LDR   R5, =0                      @   by writing 0 to CSR
  STR   R5, [R4]                    @   CSR is the Control and Status Register
  
  LDR   R4, =SYSTICK_LOAD           @ Set SysTick LOAD for period delay
  LDR   R5, =8000                   @ Assuming a 8MHz clock
  MUL   R5, R5, R0                  @ LOAD = 8000 * period - 1
  SUB   R5, R5, #1        
  STR   R5, [R4]                     
  
  LDR   R4, =SYSTICK_VAL            @ Reset SysTick internal counter to 0
  LDR   R5, =0x1                    @   by writing any value
  STR   R5, [R4]  

  LDR   R4, =SYSTICK_CSR            @ Start SysTick timer by setting CSR to 0x5
  LDR   R5, =0x7                    @   set CLKSOURCE (bit 2) to system clock (1)
  STR   R5, [R4]                    @   set ENABLE (bit 0) to 1

  
  POP   {R4-R5,PC}

@ set_up_button() - configure all the settings to use push button.
@
@ Args:
@   None
@ Return:
@   None
set_up_button:
  PUSH  {R4-R5,LR}
  @ Configure USER pushbutton (GPIO Port A Pin 0 on STM32F3 Discovery
  @   kit) to use the EXTI0 external interrupt signal
  @ Determined by bits 3..0 of the External Interrrupt Control
  @   Register (EXTIICR)
  @ STM32F303 Reference Manual 12.1.3 (pg. 249)
  LDR     R4, =SYSCFG_EXTIICR1
  LDR     R5, [R4]
  BIC     R5, R5, #0b1111
  STR     R5, [R4]

  @ Enable (unmask) interrupts on external interrupt EXTI0
  @ EXTI0 corresponds to bit 0 of the Interrupt Mask Register (IMR)
  @ STM32F303 Reference Manual 14.3.1 (pg. 297)
  LDR     R4, =EXTI_IMR
  LDR     R5, [R4]
  ORR     R5, R5, #1
  STR     R5, [R4]

  @ Set falling edge detection on EXTI0
  @ EXTI0 corresponds to bit 0 of the Falling Trigger Selection
  @   Register (FTSR)
  @ STM32F303 Reference Manual 14.3.4 (pg. 298)
  LDR     R4, =EXTI_FTSR
  LDR     R5, [R4]
  ORR     R5, R5, #1
  STR     R5, [R4]

  @ Enable NVIC interrupt channel (Nested Vectored Interrupt Controller)
  @ EXTI0 corresponds to NVIC channel #6
  @ Enable channels using the NVIC Interrupt Set Enable Register (ISER)
  @ Writing a 1 to a bit enables the corresponding channel
  @ Writing a 0 to a bit has no effect
  @ STM32 Cortex-M4 Programming Manual 4.3.2 (pg. 210)
  LDR     R4, =NVIC_ISER
  MOV     R5, #(1<<6)
  STR     R5, [R4]

  POP   {R4-R5,PC}


@
@ set_next_target()
@
set_next_target:
  PUSH {R4-R6, LR}

  LDR   R4, =seed     @ R4 = seed_ptr
  LDR   R5, [R4]      @ R5 seed = *seed_ptr 

  MOV   R0, R4
  MOV   R1, #8
  MOV   R2, #15
  BL    random_int     @     rand: int = random_number_generator(seed, 8, 15)

  LDR   R6, =correct_LED
  STR   R0, [R6]                    @     *correct_LED  = rand

  @ MOV   R0, R4
  @ MOV   R1, #0
  @ LDR   R2, =MAX_SEED_VALUE
  @ BL    tmp_random_int     @     new_seed: int = random_number_generator(8, 15)

  @ STR   R0, [R4]                      @     *seed = new_seed

  POP {R4-R6, PC}
  

@
@ move_to_next_led()
@
move_to_next_led:
  PUSH {R4-R6, LR}

  LDR R4, =current_LED
  LDR R5, [R4]
  ADD R5, R5, #1
  LDR R6, =15
  CMP R5, R6
  BLS .skip_circling
  MOV R5, #8

.skip_circling:
  STR R5, [R4]

  POP {R4-R6, PC}


@
@ turn_off_all_led()
@
turn_off_all_led:
  PUSH {R4-R5, LR}

  LDR     R4, =GPIOE_ODR
  LDR     R5, =0
  STR     R5, [R4]

  POP {R4-R5, PC}

@
@ turn_on_led()
@
@ Arguments
@
@ R0  led pin to enable (8-15 inclusive)
@
turn_on_led:
  PUSH {R4-R6, LR}

  LDR     R4, =GPIOE_ODR
  LDR     R5, [R4]
  LDR     R6, =1
  LSL     R6, R6, R0
  ORR     R5, R6
  STR     R5, [R4]

  POP {R4-R6, PC}

@
@ turn_off_led()
@
@ Arguments
@
@ R0  led pin to disable (8-15 inclusive)
@
turn_off_led:
  PUSH {R4-R6, LR}

  LDR     R4, =GPIOE_ODR
  LDR     R5, [R4]
  LDR     R6, =1
  LSL     R6, R6, R0
  BIC     R5, R6
  STR     R5, [R4]

  POP {R4-R6, PC}

@
@ switch_led()
@
@ Arguments
@
@ R0  led pin to enable (8-15 inclusive)
@
switch_led:
  PUSH {R4-R6, LR}

  LDR     R4, =GPIOE_ODR
  LDR     R5, [R4]
  LDR     R6, =1
  LSL     R6, R6, R0
  EOR     R5, R6
  STR     R5, [R4]

  POP {R4-R6, PC}



@
@ increase_level()
@
increase_level:
  PUSH {R4-R6, LR}

  LDR R4, =level
  LDR R5, [R4]
  LDR R6, =MAX_LEVEL
  

  CMP R5, R6
  BHS .skip_increasing_level
  ADD R5, R5, #1
  STR R5, [R4]

  LDR R6, =levels
  LDR R0, [R6, R5, LSL 2]
  BL set_tick_period

.skip_increasing_level:

  POP {R4-R6, PC}


@
@ reset_game()
@
reset_game:
  PUSH {R4-R5, LR}

  LDR R4, =level
  LDR R5, =0
  STR R5, [R4]
  
  LDR R4, =levels
  LDR R0, [R4]
  BL set_tick_period

  LDR R4, =current_LED
  LDR R5, =8
  STR R5, [R4]

  BL set_next_target

  POP {R4-R5, PC}


@ On Falure blinks all LEDs for the amount of levels completed 
on_fail:
  PUSH    {R4-R6, LR}
  LDR	    R4, =level          @ int currentLevel = [level]
  LDR     R5, [R4]            @ int levelCounter = [currentLevel]
  MOV     R4, R5              @ currentLevel  =  levelCounter
  MOV     R5, #-1             @ levelCounter = -1 (counts the level 0)
  MOV     R6, #8              @ currentLed = 8
.failure_blink:               
  BL      turn_off_all_led    @ turn_off_all_led()
  @ wait for 0.2s
  LDR     R0, =200
  BL      delay_ms

  CMP     R4, R5
  BEQ     end_failure
.loop_through_leds:
  MOV     R0, R6
  BL      turn_on_led
  ADD     R6, R6, #1
  CMP     R6, #15
  BGT     .end_loop_through_leds
  B       .loop_through_leds
.end_loop_through_leds:
  @ wait for 0.5s
  LDR     R0, =500
  BL      delay_ms

  ADD     R5, R5, #1
  MOV     R6, #8                  @ Resets LED counter
  B       .failure_blink
end_failure:
  POP     {R4-R6, PC}


@ ============================================================================
@ ===============================  Global data ===============================
@ ============================================================================
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