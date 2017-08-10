#include <avr/io.h>
#include "mtn_library.h"
#include "action_id.h"
#include "action.h"
#include <stdio.h>

// private variables
typedef enum {mtn_fwd=0,mtn_bwd=1,mtn_turn_left=2,mtn_turn_right=3,mtn_left=4,mtn_right=5,mtn_fwd_turn_left=6,mtn_fwd_turn_right=7,
              mtn_bwd_turn_left=8,mtn_bwd_turn_right=9,mtn_fwd_left=10,mtn_fwd_right=11,mtn_bwd_left=12,mtn_bwd_right=13,
              mtn_fast_fwd=14,mtn_fast_bwd=15} mtn_t;
typedef enum {idle,wait_start,wait_middle,wait_end} full_states;

typedef struct{
  uint8_t start_l;
  uint8_t start_r;
  uint8_t middle_l;
  uint8_t middle_r;
  uint8_t end_l;
  uint8_t end_r;
}TPages;

uint8_t mtn_lib_stop;
foot_t mtn_lib_start_foot;
foot_t mtn_lib_current_foot;
TPages mtn_pages[]={{F_S_L,F_S_R,F_M_L,F_M_R,F_E_L,F_E_R},
                    {B_S_L,B_S_R,B_M_L,B_M_R,B_E_L,B_E_R},
                    {LT_S_L,LT_S_R,LT_M_L,LT_M_R,LT_E_L,LT_E_R},
                    {RT_S_L,RT_S_R,RT_M_L,RT_M_R,RT_E_L,RT_E_R},
                    {L_S_L,L_S_R,L_M_L,L_M_R,L_E_L,L_E_R},
                    {R_S_L,R_S_R,R_M_L,R_M_R,R_E_L,R_E_R},
                    {FLT_S_L,FLT_S_R,FLT_M_L,FLT_M_R,FLT_E_L,FLT_E_R},
                    {FRT_S_L,FRT_S_R,FRT_M_L,FRT_M_R,FRT_E_L,FRT_E_R},
                    {BLT_S_L,BLT_S_R,BLT_M_L,BLT_M_R,BLT_E_L,BLT_E_R},
                    {BRT_S_L,BRT_S_R,BRT_M_L,BRT_M_R,BRT_E_L,BRT_E_R},
                    {FL_S_L,FL_S_R,FL_M_L,FL_M_R,FL_E_L,FL_E_R},
                    {FR_S_L,FR_S_R,FR_M_L,FR_M_R,FR_E_L,FR_E_R},
                    {BL_S_L,BL_S_R,BL_M_L,BL_M_R,BL_E_L,BL_E_R},
                    {BR_S_L,BR_S_R,BR_M_L,BR_M_R,BR_E_L,BR_E_R},
                    {fst_F_L_S,fst_F_R_S,fst_F_R_L_M,fst_F_L_R_M,fst_F_R_E,fst_F_L_E},
                    {fst_B_L_S,fst_B_R_S,fst_B_L_M,fst_B_R_M,fst_B_L_E,fst_B_R_E}};

/* private functions */
uint8_t mtn_lib_full(TPages *pages)
{
  static full_states state=idle;
  uint8_t done=0x00;

  switch(state)
  {
    case idle: if(mtn_lib_start_foot==left_foot)
                 action_set_page(pages->start_l);
               else
                 action_set_page(pages->start_r);
               action_start_page();
               mtn_lib_current_foot=mtn_lib_start_foot;
               state=wait_start;
               break;
    case wait_start: if(is_action_running())
                       state=wait_start;
                     else
                     {
                       if(mtn_lib_stop==0x01)
                       {
                         if(mtn_lib_current_foot==left_foot)
                         {
                           action_set_page(pages->end_r);
                           mtn_lib_current_foot=right_foot;
                         }
                         else
                         {
                           action_set_page(pages->end_l);
                           mtn_lib_current_foot=left_foot;
                         }
                         action_start_page();
                         state=wait_end;
                       }
                       else
                       {
                         if(mtn_lib_current_foot==left_foot)
                         {
                           action_set_page(pages->middle_r);
                           mtn_lib_current_foot=right_foot;
                         }
                         else
                         {
                           action_set_page(pages->middle_l);
                           mtn_lib_current_foot=left_foot;
                         }
                         action_start_page();
                         state=wait_middle;
                       }
                     }
                     break;
    case wait_middle: if(is_action_running())
                        state=wait_middle;
                      else
                      {
                        if(mtn_lib_stop==0x01)
                        {
                          if(mtn_lib_current_foot==left_foot)
                          {
                            action_set_page(pages->end_r);
                            mtn_lib_current_foot=right_foot;
                          }
                          else
                          {
                            action_set_page(pages->end_l);
                            mtn_lib_current_foot=left_foot;
                          }
                          action_start_page();
                          state=wait_end;
                        }
                        else
                        {
                          if(mtn_lib_current_foot==left_foot)
                          {
                            action_set_page(pages->middle_r);
                            mtn_lib_current_foot=right_foot;
                          }
                          else
                          {
                            action_set_page(pages->middle_l);
                            mtn_lib_current_foot=left_foot;
                          }
                          action_start_page();
                          state=wait_middle;
                        }
                      }
                      break;
    case wait_end: if(is_action_running())
                     state=wait_end;
                   else
                   {
                     mtn_lib_stop=0x00;
                     state=idle;
                     done=0x01;
                   }
                   break;
  }

  return done;
}

uint8_t mtn_lib_left(TPages *pages)
{
  static full_states state=idle;
  uint8_t done=0x00;

  switch(state)
  {
    case idle: if(mtn_lib_start_foot==left_foot)
                 action_set_page(pages->start_l);
               else
                 action_set_page(pages->start_r);
               action_start_page();
               mtn_lib_current_foot=mtn_lib_start_foot;
               state=wait_start;
               break;
    case wait_start: if(is_action_running())
                       state=wait_start;
                     else
                     {
                       if(mtn_lib_current_foot==left_foot)
                       {
                         if(mtn_lib_stop==0x01)
                         {
                           action_set_page(pages->end_r);
                           state=wait_end;
                         }
                         else
                         {
                           action_set_page(pages->middle_r);
                           state=wait_middle;
                         }
                         mtn_lib_current_foot=right_foot;
                         action_start_page();
                       }
                       else
                       {
                         action_set_page(pages->middle_l);
                         mtn_lib_current_foot=left_foot;
                         action_start_page();
                         state=wait_middle;
                       }
                     }
                     break;
    case wait_middle: if(is_action_running())
                        state=wait_middle;
                      else
                      {
                        if(mtn_lib_current_foot==left_foot)
                        {
                          if(mtn_lib_stop==0x01)
                          {
                            action_set_page(pages->end_r);
                            state=wait_end;
                          }
                          else
                          {
                            action_set_page(pages->middle_r);
                            state=wait_middle;
                          }
                          mtn_lib_current_foot=right_foot;
                          action_start_page();
                        }
                        else
                        {
                          action_set_page(pages->middle_l);
                          mtn_lib_current_foot=left_foot;
                          action_start_page();
                          state=wait_middle;
                        }
                      }
                      break;
    case wait_end: if(is_action_running())
                     state=wait_end;
                   else
                   {
                     mtn_lib_stop=0x00;
                     state=idle;
                     done=0x01;
                   }
                   break;
  }

  return done;
}

uint8_t mtn_lib_right(TPages *pages)
{
  static full_states state=idle;
  uint8_t done=0x00;

  switch(state)
  {
    case idle: if(mtn_lib_start_foot==left_foot)
                 action_set_page(pages->start_l);
               else
                 action_set_page(pages->start_r);
               action_start_page();
               mtn_lib_current_foot=mtn_lib_start_foot;
               state=wait_start;
               break;
    case wait_start: if(is_action_running())
                       state=wait_start;
                     else
                     {
                       if(mtn_lib_current_foot==right_foot)
                       {
                         if(mtn_lib_stop==0x01)
                         {
                           action_set_page(pages->end_l);
                           state=wait_end;
                         }
                         else
                         {
                           action_set_page(pages->middle_l);
                           state=wait_middle;
                         }
                         mtn_lib_current_foot=left_foot;
                         action_start_page();
                       }
                       else
                       {
                         action_set_page(pages->middle_r);
                         mtn_lib_current_foot=right_foot;
                         action_start_page();
                         state=wait_middle;
                       }
                     }
                     break;
    case wait_middle: if(is_action_running())
                        state=wait_middle;
                      else
                      {
                        if(mtn_lib_current_foot==right_foot)
                        {
                          if(mtn_lib_stop==0x01)
                          {
                            action_set_page(pages->end_l);
                            state=wait_end;
                          }
                          else
                          {
                            action_set_page(pages->middle_l);
                            state=wait_middle;
                          }
                          mtn_lib_current_foot=left_foot;
                          action_start_page();
                        }
                        else
                        {
                          action_set_page(pages->middle_r);
                          mtn_lib_current_foot=right_foot;
                          action_start_page();
                          state=wait_middle;
                        }
                      }
                      break;
    case wait_end: if(is_action_running())
                     state=wait_end;
                   else
                   {
                     mtn_lib_stop=0x00;
                     state=idle;
                     done=0x01;
                   }
                   break;
  }

  return done;
}

/* public functions */
void mtn_lib_init(void)
{
  mtn_lib_stop=0x00;
  mtn_lib_start_foot=left_foot;
  mtn_lib_current_foot=left_foot;
}

void mtn_lib_stop_mtn(void)
{
  mtn_lib_stop=0x01;
}

void mtn_lib_set_start_foot(foot_t foot)
{
  mtn_lib_start_foot=foot;
}

uint8_t walk_forward(void)
{
  return mtn_lib_full(&mtn_pages[mtn_fwd]);
}

uint8_t walk_backward(void)
{
  return mtn_lib_full(&mtn_pages[mtn_bwd]);
}

uint8_t turn_left(void)
{
  return mtn_lib_left(&mtn_pages[mtn_turn_left]);
}

uint8_t turn_right(void)
{
  return mtn_lib_right(&mtn_pages[mtn_turn_right]);
}

uint8_t walk_left(void)
{
  return mtn_lib_left(&mtn_pages[mtn_left]);
}

uint8_t walk_right(void)
{
  return mtn_lib_right(&mtn_pages[mtn_right]);
}

uint8_t walk_forward_turn_left(void)
{
  return mtn_lib_left(&mtn_pages[mtn_fwd_turn_left]);
}

uint8_t walk_forward_turn_right(void)
{
  return mtn_lib_right(&mtn_pages[mtn_fwd_turn_right]);
}

uint8_t walk_backward_turn_left(void)
{
  return mtn_lib_left(&mtn_pages[mtn_bwd_turn_left]);
}

uint8_t walk_backward_turn_right(void)
{
  return mtn_lib_right(&mtn_pages[mtn_bwd_turn_right]);
}

uint8_t walk_forward_left(void)
{
  return mtn_lib_left(&mtn_pages[mtn_fwd_left]);
}

uint8_t walk_forward_right(void)
{
  return mtn_lib_right(&mtn_pages[mtn_fwd_right]);
}

uint8_t walk_backward_left(void)
{
  return mtn_lib_left(&mtn_pages[mtn_bwd_left]);
}

uint8_t walk_backward_right(void)
{
  return mtn_lib_right(&mtn_pages[mtn_bwd_right]);
}


uint8_t fast_walk_forward(void)
{
  return mtn_lib_full(&mtn_pages[mtn_fast_fwd]);
}

uint8_t fast_walk_backward(void)
{
  return mtn_lib_full(&mtn_pages[mtn_fast_bwd]);
}

uint8_t fast_turn_left(void)
{
 
}

uint8_t fast_turn_right(void)
{
 
}

uint8_t fast_walk_left(void)
{
}

uint8_t fast_walk_right(void)
{
}

uint8_t fast_walk_forward_turn_left(void)
{
 
}

uint8_t fast_walk_forward_turn_right(void)
{
 
}

uint8_t fast_walk_backward_turn_left(void)
{
 
}

uint8_t fast_walk_backward_turn_right(void)
{
 
}


