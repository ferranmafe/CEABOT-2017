#include <util/delay.h>
#include <stdio.h>
#include "cm510.h" //Que a su vez import adc.h que se usa para leer los sensores...
#include "balance.h"
#include "exp_board.h"
#include "mtn_library.h"
#include "action_id.h"

typedef enum {wait_start, wait_ready, scanning_enemy, walking_to_enemy, wait_walk, attack, wait_attack, searching_enemy, wait_search} main_states;

void user_init(void)
{
  serial_console_init(57600);
  balance_init();
  balance_calibrate_gyro();
  balance_enable_gyro();
  user_time_set_period(100);
  exp_adc_start();
}

void user_loop(void)
{
  static main_states state=wait_start;
  uint8_t adc_value;

  switch(state)
  {
    case wait_start: if(is_button_rising_edge(BTN_START))
                     {
                       action_set_page(31);
                       action_start_page();
                       state=wait_ready;
                     }
                     else
                       state=wait_start; //La espera de 5 s ??? O no se hace en Sumo
                     break;

     case wait_ready: if (is_action_running()) {
                        state = wait_ready;
                      }
                      else {
                        state = scanning_enemy;
                      }
                      break;

     case scanning_enemy: adc_value = exp_adc_get_avg_channel(ADC_PORT_6);

                          if (adc_value >= 400) {
                            state = attack;
                          }
                          else if (adc_value <= 150) {
                            turn_left();
                            state = searching_enemy;
                          }
                          else {
                            walk_forward();
                            state = walking_to_enemy;
                          }
                          break;

      case attack:        action_set_page(A_F_attack);
                          action_start_page();
                          state=wait_attack;
                          break;

      case wait_attack: if (is_action_running()) state = wait_attack;
                        else state = scanning_enemy;
                        break;

      case searching_enemy:   adc_value = exp_adc_get_avg_channel(ADC_PORT_6);
                              if (adc_value > 150) {
                                mtn_lib_stop_mtn();
                                state=wait_search;
                              }
                              else
                                state=searching_enemy;
                              turn_left();
                              break;

      case wait_search:        if(turn_left())
                                 state= scanning_enemy;
                               else
                                 state=wait_search;
                               break;

     case walking_to_enemy:  adc_value = exp_adc_get_avg_channel(ADC_PORT_6);
                             if (adc_value >= 400) {
                               mtn_lib_stop_mtn();
                               state=wait_walk;
                             }
                             else
                               state=walking_to_enemy;
                             walk_forward();
                             break;

     case wait_walk:        if(walk_forward())
                                state= scanning_enemy;
                              else
                                state=wait_walk;
                              break;

  }
}
