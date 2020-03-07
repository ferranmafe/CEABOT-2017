#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include "cm510.h" //Que a su vez import adc.h que se usa para leer los sensores...
#include "balance.h"
#include "exp_board.h"
#include "mtn_library.h"
#include "action_id.h"

typedef enum {wait_start, wait_ready, side_step, wait_side_step, scanning_enemy, walking_to_enemy, wait_walk, attack, wait_attack, searching_enemy, wait_search, check_fallen, get_up} main_states;
typedef enum {stop_motion,wait_stop_motion,wait_getting_up,wait_stabilize} get_up_states;

uint8_t get_up_process(fallen_t fall_state)
{
  static get_up_states state;
  uint8_t done=0x00;

  switch(state)
  {
    case stop_motion: if(is_action_running())
                      {
                        action_stop_page();
                        state=wait_stop_motion;
                      }
                      else
                      {
                        if(fall_state==robot_face_down)
                          action_set_page(27);
                        else
                          action_set_page(28);
                        action_start_page();
                        state=wait_getting_up;
                      }
                      break;
    case wait_stop_motion: if(!is_action_running())
                           {
                             if(fall_state==robot_face_down)
                               action_set_page(27);
                             else
                               action_set_page(28);
                             action_start_page();
                             state=wait_getting_up;
                           }
                           else
                             state= wait_stop_motion;
                           break;
    case wait_getting_up: if(!is_action_running())
                          {
                            state=wait_stabilize;
                            user_time_set_one_time(1000);
                          }
                          else
                            state=wait_getting_up;
                          break;
    case wait_stabilize: if(user_time_is_done())
                         {
                           balance_reset_fall_state();
                           state=stop_motion;
                           done=0x01;
                         }
                         else
                           state=wait_stabilize;
                         break;
  }

  return done;
}

void user_init(void)
{
  serial_console_init(57600);
  balance_init();
  balance_calibrate_gyro();
  balance_enable_gyro();
  user_time_set_period(100);
  exp_adc_start();
  exp_bno055_start();
}

void user_loop(void)
{
  static main_states state=wait_start;

  static int side_steps_count = 0;

  static int turn_to_right = 0;

  static int actual_attack = 0;
  static int adc5_read, adc6_read;
  static int last_attack = 0;

  int attacks[4] = {A_F_attack, A_F_attack, A_F_attack, F_Throw2};

  static fallen_t fall_state;
  uint8_t error = 10;

  switch(state)
  {
    case wait_start: if(is_button_rising_edge(BTN_START))
                     {
                       action_set_page(31);
                       action_start_page();
                       state=wait_ready;
                     }
                     else
                       state=wait_start;
                     break;

    case wait_ready: if (is_action_running()) {
                        state = wait_ready;
                      }
                      else {
                        state = side_step;
                        walk_left();
                      }
                      break;

    case side_step:         mtn_lib_stop_mtn();
                            state=wait_side_step;
                            walk_left();
                            break;

    case wait_side_step:     if(walk_left()) {
                               if (side_steps_count > 2)
                                 state= scanning_enemy;
                               else {
                                 walk_left();
                                 ++side_steps_count;
                                 state = side_step;
                               }
                             }
                             else
                               state=wait_side_step;
                             break;

     case scanning_enemy: right_sensor = exp_adc_get_avg_channel(ADC_PORT_6);
                          middle_sensor = exp_adc_get_avg_channel(ADC_PORT_5);
                          left_sensor = exp_adc_get_avg_channel(ADC_PORT_4);

                          if (right_sensor >= 300) turn_to_right = 1;
                          else if (middle_sensor >= 300 - error) turn_to_right = 0;

                          cm510_printf("Turn to left: %d\n",turn_to_right);
			                       _delay_ms(100);

                          if (middle_sensor >= 400 + error) {
                            state = attack;
                          }
                          else if (adc6_read <= 300 - error) {
                            if (!(turn_to_right == 1)) turn_left();
                            else turn_right();
                            state = searching_enemy;
                          }
                          else {
                            walk_forward();
                            state = walking_to_enemy;
                          }
                          break;

      case attack:        action_set_page(attacks [actual_attack%4]);
                          action_start_page();
                          last_attack = actual_attack%4;
                          state=wait_attack;
                          break;

      case wait_attack: if (is_action_running()) state = wait_attack;
                        else {
                          if (last_attack == 3) state = check_fallen;
                          else {
                            action_set_page(31);
                            action_start_page();
                            state=wait_ready;
                          }
                        }
                        break;

      case searching_enemy:   right_sensor = exp_adc_get_avg_channel(ADC_PORT_6);
                              middle_sensor = exp_adc_get_avg_channel(ADC_PORT_5);
                              left_sensor = exp_adc_get_avg_channel(ADC_PORT_4);

                              if (middle_sensor > 300 - error) {
                                mtn_lib_stop_mtn();
                                state=wait_search;
                              }

                              else state=searching_enemy;
                              if (!(turn_to_right == 1)) turn_left();
                              else turn_right();
                              break;

      case wait_search:        if(turn_left())
                                 state= scanning_enemy;
                               else
                                 state=wait_search;
                               break;

     case walking_to_enemy: right_sensor = exp_adc_get_avg_channel(ADC_PORT_6);
                            middle_sensor = exp_adc_get_avg_channel(ADC_PORT_5);
                            left_sensor = exp_adc_get_avg_channel(ADC_PORT_4);

                            if (middle_sensor >= 400 - error || middle_sensor <= 300 + error) {
                               mtn_lib_stop_mtn();
                               state=wait_walk;
                             }
                             else state=walking_to_enemy;
                             walk_forward();
                             break;

     case wait_walk:        if(walk_forward())
                                state= scanning_enemy;
                              else
                                state=wait_walk;
                              break;

    case check_fallen:
                       if (exp_bno055_get_gravity_x() > 6000) fall_state = robot_face_up;
                       else if (exp_bno055_get_gravity_x() < -6000) fall_state = robot_face_down;
                       else fall_state = robot_standing;

                       if(fall_state!=robot_standing)
                         state=get_up;
                       else
                         state=check_fallen;

                       break;

     case get_up: if(get_up_process(fall_state)==0x01) {
                     action_set_page(31);
                     action_start_page();
                     state=wait_ready;
                  }
                 else
                   state=get_up;
                 break;
  }
  ++actual_attack;
}
