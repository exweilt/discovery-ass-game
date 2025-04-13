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

