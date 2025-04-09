  .syntax unified
  .cpu cortex-m4
  .fpu softvfp
  .thumb
  
  .global   process_invitation
  .global   random
  @ .global EXTI0_IRQHandler
  .global set_gpio_port_e_clock
  .global set_pins_for_output
  .global set_up_button

  .extern current_LED
  .extern current_period

  .include "definitions.s"
  @ .include "game.s"
    

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
@
random_number_generator:
  PUSH {R3, R4,LR}    
  MOV R0, #11        
@ .set_variables:
@   SUB   R3, R2, R1              @ range = max - min
@   UDIV  R4, R2, R3              @ threshold = (max / range) * range;
@   MUL   R4, R4, R3              @ 
@ .generate_random_number:          @
@   EOR   R0, R0, R0,  LSL #13    @ seed ^= seed << 13
@   EOR   R0, R0, R0,  LSR #17    @ seed ^= seed >> 17
@   EOR   R0, R0, R0,  LSL #5     @ seed ^= seed << 5
@ .find_value:                     @ do {
@   CMP   R0, R2                  @ random = generaterandomNumber();
@   BGE   .generate_random_number @ while (random >= threshold); }// Reject values outside safe multiples 
@   UDIV  R1, R0, R2              @ R1 = a / b
@   MUL   R2, R1, R2              @ R2 = (a/b) * b
@   SUB   R0, R0, R2              @ R0 = a - (a/b) * b (remainder in R2)
  POP {R3,R4,PC}

@ @   Input handling
@ @   Returns:
@ @       R0 - 0 ifButtonNotPressed
@ @       R0 - 1 ifButtonISPressed
@ .type EXTI0_IRQHandler, %function
@ EXTI0_IRQHandler:
@   PUSH  {R4,R5,LR}
@   @ Set button state to pressed
@   MOV   R0, #1
    
@   @ Clear the interrupt pending bit
@   LDR   R4, =EXTI_PR
@   MOV   R5, #1
@   STR   R5, [R4]
@   POP   {R4,R5,PC}

@ @   Blinking - open/close LED
@ @   
@ @   Need input R1 (LED Number)
@ @
@ @   R4 Read GPIOE_ODR
@ @   R5 Store GPIOE_ODR and make change
@ @
@ @   R6 R7 Calculations to change GPIOE_ODR

@ bliking:
@   PUSH  {R4-R12,LR}                     @ LED code in R1

@   LDR     R4, =GPIOE_ODR
@   LDR     R5, [R4]                      @ Read

@   MOV     R6, #0b1                      @ Access LED
@   LSL     R7, R6, R1

@   EOR     R5, R7                        @ Modify
@   STR     R5, [R4]                      @ Write

@   BL      STICK_TIMER
@   @ Need Access to Stick Timer (To check pause time)
@
@   POP  {R4-R12,PC}


@ @
@ @   Clockwise Blinking - Blink in Clockwise
@ @
@ @   No input need
@ @
@ @   Use Blinking subroutine
@ @
@ @   R4 Current working LED
@ @   R1 Update to current LED (To check Level (Level subroutine))
@ @
@ clockwise_bliking:
@   PUSH  {R4-R12,LR}

@ .reset:

@   MOV	  R4, #7

@ .loop:

@   ADD	  R4, R4, #1
@   CMP	  R4, #16
@   BEQ	  .reset


@   MOV   R1, R4
@   LDR 	R3, =current_LED		@ Update current_LED
@   LDR   R1, [R3]
@   BL 	  blinking			        @ Open
@   BL 	  blinking   			    @ Close


@   B  	  loop

@   POP  {R4-R12,PC}

@ ==========================================================

@ @   Level - Level Up Count
@ @
@ @   No input need
@ @
@ @   R0 Load Level and add 1 Level
@ @   R0 Load Time break and reduce by 200ms
@ @

@ level_up:
@   PUSH  {LR}

@   LDR	  R1, =Level
@   LDR   R0, [R1]
@   ADD	  R0, R0, #1			@ Level Up
@   STR	  R0, [R1]

@   LDR	  R1, =Time
@   LDR   R0, [R1]
@   SUB	  R0, R0, #200			@ Reduce by 200 milisec
@   STR	  R0, [R1]

@   BL	  clockwise_bliking

@   POP  {PC}


@ ==========================================================


@ @   EXTIO_IRQHandler - Check if you lose or win the round
@ @
@ @   No input need
@ @
@ @   The Exception Handler will automaticly called if bottom pressed
@ @   R4 current LED load (save from Clockwise Blinking)
@ @   R5 correct LED load (save from Random number)
@ @   R0 R4 R5 For reset the Exception Handler
@ @

@ EXTI0_IRQHandler:

@   PUSH  {R4,R5,LR}			            @ Return R0 1 TRUE, 0 FALSE

@   LDR   R4, =current_LED        	@ Check current LED and correct LED
@   LDR   R1, [R4]
@   LDR   R5, =correct_LED
@   LDR   R2, [R5]
@   CMP	  R1, R2
@   BEQ	  end

@   MOV 	  R0, #0



@   MOV 	  R0, #1

@   LDR   R4, =EXTI_PR      		@ Clear (acknowledge) the interrupt
@   MOV   R5, #(1<<0)       		
@   STR   R5, [R4]          		

@   POP  {R4,R5,PC}

@ end_game:


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
  BL    tmp_random_int     @     rand: int = random_number_generator(seed, 8, 15)

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


@ Subroutine: Generates a pseudo-random integer in [min, max] using LCG
@ Inputs:
@   R0: Pointer to the current seed (uint32_t*)
@   R1: Minimum value (inclusive)
@   R2: Maximum value (inclusive)
@ Output:
@   R0: Random integer in [min, max]
@ Modifies:
@   R3, R4, R5 (saved/restored via stack)
tmp_random_int:
    PUSH    {R3-R5, LR}       @ Save registers and return address

    @ --- Step 1: Update the seed using LCG ---
    @ LCG formula: new_seed = (a * seed + c) mod 2^32
    @ Constants (a=1664525, c=1013904223)
    LDR     R3, [R0]          @ Load current seed into R3
    LDR     R4, =1664525      @ Load multiplier (a)
    MUL     R3, R3, R4        @ R3 = seed * a
    LDR     R4, =1013904223   @ Load increment (c)
    ADD     R3, R3, R4        @ R3 = new_seed (a*seed + c)
    STR     R3, [R0]          @ Store updated seed back to memory

    @ --- Step 2: Compute range_size = max - min + 1 ---
    SUB     R4, R2, R1        @ R4 = max - min
    ADD     R4, R4, #1        @ R4 = range_size (max - min + 1)

    @ --- Step 3: Scale new_seed to [min, max] ---
    UDIV    R5, R3, R4        @ R5 = new_seed / range_size (quotient)
    MLS     R0, R4, R5, R3    @ R0 = new_seed - (range_size * quotient) = remainder
    ADD     R0, R0, R1        @ R0 = remainder + min (final result)

    POP     {R3-R5, PC}       @ Restore registers and return