  .syntax unified
  .cpu cortex-m4
  .fpu softvfp
  .thumb
  
  .global Main
  .global EXTI0_IRQHandler

  @ Definitions are in definitions.s to keep this file "clean"
  .include "definitions.s"

  .section .text

Main:
  PUSH  {R4-R5,LR}

  @ Initial count of button presses to 0
  @ Count must be maintained in memory - interrupt handlers
  @   must not rely on registers to maintain values across
  @   different invocations of the handler (i.e. across
  @   different presses of the pushbutton)
  LDR   R4, =count        @ count = 0;
  MOV   R5, #0            @
  STR   R5, [R4]          @

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

  @ Nothing else to do in Main
  @ Idle loop forever (welcome to interrupts!!)
Idle_Loop:
  B     Idle_Loop
  
End_Main:
  POP   {R4-R5,PC}


@
@ External interrupt line 0 (EXTI0) interrupt handler
@
  .type  EXTI0_IRQHandler, %function
EXTI0_IRQHandler:

  PUSH  {R4,R5,LR}

  @ Add one to count of button presses
  LDR   R4, =count        @ count = count + 1
  LDR   R5, [R4]          @
  ADD   R5, R5, #1        @
  STR   R5, [R4]          @

  @ Tell microcontroller that we have handled the EXTI0 interrupt
  @ By writing a 1 to bit 0 of the EXTI Pending Register (PR)
  @ (Writing 0s to bits has no effect)
  @ STM32F303 Reference Manual 14.3.6 (pg. 299)
  LDR   R4, =EXTI_PR      @ Clear (acknowledge) the interrupt
  MOV   R5, #(1<<0)       @
  STR   R5, [R4]          @

  @ Return from interrupt handler
  POP  {R4,R5,PC}



  .section .data
  
count:
  .space  4

  .end