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

  @ Definitions are in definitions.s to keep this file "clean"
  .include "definitions.s"
  .include "subroutines.s"

  .equ    BLINK_PERIOD, 250

  .section .text

Main:
  PUSH  {R4-R5,LR}


  BL set_gpio_port_e_clock

  @ So LEDs are set up to illuminate light
  BL set_pins_for_output

  @ Initialise the static variables
  LDR     R4, =blink_countdown
  LDR     R5, =BLINK_PERIOD
  STR     R5, [R4]  

  @ Configure SysTick Timer to generate an interrupt every 1ms

  LDR     R4, =SCB_ICSR               @ Clear any pre-existing interrupts
  LDR     R5, =SCB_ICSR_PENDSTCLR     @
  STR     R5, [R4]                    @

  LDR     R4, =SYSTICK_CSR            @ Stop SysTick timer
  LDR     R5, =0                      @   by writing 0 to CSR
  STR     R5, [R4]                    @   CSR is the Control and Status Register
  
  LDR     R4, =SYSTICK_LOAD           @ Set SysTick LOAD for 1ms delay
  LDR     R5, =7999                   @ Assuming 8MHz clock
  STR     R5, [R4]                    @ 

  LDR     R4, =SYSTICK_VAL            @   Reset SysTick internal counter to 0
  LDR     R5, =0x1                    @     by writing any value
  STR     R5, [R4]

  LDR     R4, =SYSTICK_CSR            @   Start SysTick timer by setting CSR to 0x7
  LDR     R5, =0x7                    @     set CLKSOURCE (bit 2) to system clock (1)
  STR     R5, [R4]                    @     set TICKINT (bit 1) to 1 to enable interrupts
                                      @     set ENABLE (bit 0) to 1


  @
  @ Prepare external interrupt Line 0 (USER pushbutton)
  @ We'll count the number of times the button is pressed
  @

  @ Initialise count to zero
  LDR   R4, =button_count             @ count = 0;
  MOV   R5, #0                        @
  STR   R5, [R4]                      @

  BL set_up_button

  @ Nothing else to do in Main
  @ Idle loop forever (welcome to interrupts!!)
Idle_Loop:
  B     Idle_Loop
  

@  On Falure blinks all LEDs for the amount of wins
On_Fail:
  PUSH  {R0-R5}
  LDR	  R0, =Level
  MOV   R1, #0
  MOV   R2, #8
  MOV   R3, #1
  @ Invert LD3
  @ by inverting bit 13 of GPIOE_ODR (GPIO Port E Output Data Register)
  @ (by using EOR to invert bit 13, leaving other bits unchanged)
.failure_blink
  CMP     R0, R1
  BEQ     end_failure
  ADD     R1, R1, #1

  LDR     R4, =GPIOE_ODR
  LDR     R5, [R4]                  @ Read ...
  EOR     R5, R5, R3, LSL R2        @ Modify ...
  STR     R5, [R4]                  @ Write
  ADD     R2, R2, #1
  CMP     R2, #15
  BGT     reset_fail_leds
  @ wait for 1s ...
  LDR     R0, =BLINK_PERIOD
  BL      delay_ms

  @ ... and repeat
  B       .failure_blink
reset_fail_leds:
  MOV     R2, #8
  B       .failure_blink
end_failure:



End_Main:
  POP   {R4-R5,PC}



@
@ SysTick interrupt handler (blink LED LD3)
@
  .type  SysTick_Handler, %function
SysTick_Handler:

  PUSH  {R4, R5, LR}

  LDR   R4, =blink_countdown        @ if (countdown != 0) {
  LDR   R5, [R4]                    @
  CMP   R5, #0                      @
  BEQ   .LelseFire                  @

  SUB   R5, R5, #1                  @   countdown = countdown - 1;
  STR   R5, [R4]                    @

  B     .LendIfDelay                @ }

.LelseFire:                         @ else {

  LDR     R4, =GPIOE_ODR            @   Invert LD3
  LDR     R5, [R4]                  @
  EOR     R5, #(0b1<<(LD3_PIN))     @   GPIOE_ODR = GPIOE_ODR ^ (1<<LD3_PIN);
  STR     R5, [R4]                  @ 

  LDR     R4, =blink_countdown      @   countdown = BLINK_PERIOD;
  LDR     R5, =BLINK_PERIOD         @
  STR     R5, [R4]                  @

.LendIfDelay:                       @ }

  LDR     R4, =SCB_ICSR             @ Clear (acknowledge) the interrupt
  LDR     R5, =SCB_ICSR_PENDSTCLR   @
  STR     R5, [R4]                  @

  @ Return from interrupt handler
  POP  {R4, R5, PC}



@
@ External interrupt line 0 interrupt handler
@   (count button presses)
@
  .type  EXTI0_IRQHandler, %function
EXTI0_IRQHandler:

  PUSH  {R4,R5,LR}

  LDR   R4, =button_count           @ count = count + 1
  LDR   R5, [R4]                    @
  ADD   R5, R5, #1                  @
  STR   R5, [R4]                    @

  LDR   R4, =EXTI_PR                @ Clear (acknowledge) the interrupt
  MOV   R5, #(1<<0)                 @
  STR   R5, [R4]                    @

  @ Return from interrupt handler
  POP  {R4,R5,PC}

@     Takes R0 as number of wins
blink_for_each_win:
  MOV   R1, #0
for_win_loop:
  CMP   R1, R0
  BGE   end_game

@ Blinking Code
  


  ADD   R1, R1, #1
  B for_win_loop

@   Global data
  .section .data
button_count:
  .space  4

blink_countdown:
  .space  4

@ unsigned int 4 bytes range 8-15
current_LED:
  .space  4

@ unsigned int 4 bytes range 8-15
correct_LED:
  .space  4

  .end