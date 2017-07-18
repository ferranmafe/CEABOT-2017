#include "ceabot_maze_alg_node.h"

CeabotMazeAlgNode::CeabotMazeAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<CeabotMazeAlgorithm>(),
  walk("ceabot_maze_walk"),
  tracking_module("ceabot_maze_track")
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]
  this->event_start = true;
  this->darwin_state = SCAN_MAZE;
  this->half_maze_achieved = false;

  this->search_started=false;
  this->first_scan_goal_achieved = false;
  this->second_scan_goal_achieved = false;
  this->third_scan_goal_achieved = false;
  // [init publishers]

  // [init subscribers]
  this->imu_subscriber_ = this->public_node_handle_.subscribe("imu", 1, &CeabotMazeAlgNode::imu_callback, this);
  pthread_mutex_init(&this->imu_mutex_,NULL);

  this->odom_subscriber_ = this->public_node_handle_.subscribe("odom", 1, &CeabotMazeAlgNode::odom_callback, this);
  pthread_mutex_init(&this->odom_mutex_,NULL); //By the moment we're just going to use the bno055 sensor

  this->joint_states_subscriber_ = this->public_node_handle_.subscribe("joint_states", 1, &CeabotMazeAlgNode::joint_states_callback, this);
  pthread_mutex_init(&this->joint_states_mutex_,NULL);

  this->qr_pose_subscriber_ = this->public_node_handle_.subscribe("qr_pose", 1, &CeabotMazeAlgNode::qr_pose_callback, this);
  pthread_mutex_init(&this->qr_pose_mutex_,NULL);
  // [init services]

  // [init clients]

  // [init action servers]

  // [init action clients]
}

CeabotMazeAlgNode::~CeabotMazeAlgNode(void)
{
  // [free dynamic memory]
  pthread_mutex_destroy(&this->imu_mutex_);
  pthread_mutex_destroy(&this->odom_mutex_);
  pthread_mutex_destroy(&this->joint_states_mutex_);
  pthread_mutex_destroy(&this->qr_pose_mutex_);
}

void CeabotMazeAlgNode::mainNodeThread(void)
{
  // [fill msg structures]

  // [fill srv structure and make request to the server]

  // [fill action structure and make request to the action server]

  // [publish messages]

  if (!this->event_start) {
    state_machine();
  }

  //Code for the start. It sets all the configuration from the .cfg to the modules.
  //Then, waits for 5 sec (start condition - read CEABOT rules) and change the Darwin
  //state to start searching for the first QR code
  else {
    //Initialization of Darwin parameters
    init_walk_module();
    init_headt_module();

    //5 sec sleep. It's for the init of the event, the rules says that Darwin must
    //wait this amount of time in seconds
    ros::Duration(5.0).sleep();

    //We must set down the start flag in terms of dont enter to this condition again
    this->event_start = false;
  }
}

/*  [subscriber callbacks] */
void CeabotMazeAlgNode::imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {
  /*/ROS_INFO("CeabotMazeAlgNode::imu_callback: New Message Received");

  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
  //this->imu_mutex_enter();
  double bnoaux = tf::getYaw(msg->orientation);
  if (bnoaux < 0) bnoaux += 2*PI;
  this->bno055_measurement = bnoaux; //We normalize the measurement...
  //ROS_INFO("Darwin Ceabot Vision : The actual Yaw is : %f", this->bno055_measurement);
  //std::cout << msg->data << std::endl;
  //unlock previously blocked shared variables
  //this->alg_.unlock();
  //this->imu_mutex_exit();*/
}

void CeabotMazeAlgNode::imu_mutex_enter(void) {
  pthread_mutex_lock(&this->imu_mutex_);
}

void CeabotMazeAlgNode::imu_mutex_exit(void) {
  pthread_mutex_unlock(&this->imu_mutex_);
}

void CeabotMazeAlgNode::odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
  //ROS_INFO("CeabotMazeAlgNode::odom_callback: New Message Received");

  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
  //this->odom_mutex_enter();

  //std::cout << msg->data << std::endl;
  //unlock previously blocked shared variables
  //this->alg_.unlock();
  //this->odom_mutex_exit();
}

void CeabotMazeAlgNode::odom_mutex_enter(void) {
  pthread_mutex_lock(&this->odom_mutex_);
}

void CeabotMazeAlgNode::odom_mutex_exit(void) {
  pthread_mutex_unlock(&this->odom_mutex_);
}

void CeabotMazeAlgNode::joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg) {
  unsigned int i;

  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
  this->joint_states_mutex_enter();

  for (i = 0; i < msg->name.size() ; ++i){
    if (msg->name[i]=="j_pan"){
      this->current_pan_angle=msg->position[i];
      if (this->darwin_state == SCAN_MAZE and this->search_started) {
          if (this->current_pan_angle >= ((85 * PI) / 180)) this->first_scan_goal_achieved = true;
          else if (this->first_scan_goal_achieved and this->current_pan_angle <= ((-85 * PI) / 180)) this->second_scan_goal_achieved = true;
          else if (this->second_scan_goal_achieved and this->current_pan_angle >= 0.0) this->third_scan_goal_achieved = true;
      }
    }
    else if (msg->name[i]=="j_tilt"){
      this->current_tilt_angle=(msg->position[i]);
    }
  }
  ROS_INFO("angle_pan: %f angle_tilt: %f",this->current_pan_angle,this->current_tilt_angle);
  //unlock previously blocked shared variables
  //this->alg_.unlock();
  this->joint_states_mutex_exit();
}

void CeabotMazeAlgNode::joint_states_mutex_enter(void) {
  pthread_mutex_lock(&this->joint_states_mutex_);
}

void CeabotMazeAlgNode::joint_states_mutex_exit(void) {
  pthread_mutex_unlock(&this->joint_states_mutex_);
}

void CeabotMazeAlgNode::qr_pose_callback(const humanoid_common_msgs::tag_pose_array::ConstPtr& msg) {
  this->qr_pose_mutex_enter();
  if (msg->tags.size()>0) {
     for (int i = 0; i < msg->tags.size(); ++i) {
         set<humanoid_common_msgs::tag_pose>::iterator it;
         it = this->qr_tags_detected.find(msg->tags[i]);

         if (it == this->qr_tags_detected.end()) this->qr_tags_detected.insert(msg->tags[i]);
     }
  }
  this->qr_pose_mutex_exit();
}

void CeabotMazeAlgNode::qr_pose_mutex_enter(void) {
  pthread_mutex_lock(&this->qr_pose_mutex_);
}

void CeabotMazeAlgNode::qr_pose_mutex_exit(void) {
  pthread_mutex_unlock(&this->qr_pose_mutex_);
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void CeabotMazeAlgNode::node_config_update(Config &config, uint32_t level) {
  this->alg_.lock();
  this->config_=config;
  this->alg_.unlock();
}

void CeabotMazeAlgNode::addNodeDiagnostics(void) {}

void CeabotMazeAlgNode::init_walk_module(void) {
  //ROS_INFO("Darwin Ceabot Vision : Walk Parameters Initialization");
  this->walk.set_x_offset(config_.X_OFFSET);
  this->walk.set_y_offset(config_.Y_OFFSET);
  this->walk.set_z_offset(config_.Z_OFFSET);
  this->walk.set_roll_offset(config_.R_OFFSET);
  this->walk.set_pitch_offset(config_.P_OFFSET);
  this->walk.set_yaw_offset(config_.A_OFFSET);
  this->walk.set_hip_pitch_offset(config_.HIP_PITCH_OFFSET);
  this->walk.set_period(config_.PERIOD_TIME);
  this->walk.set_ssp_dsp_ratio(config_.DSP_RATIO);
  this->walk.set_fwd_bwd_ratio(config_.STEP_FB_RATIO);
  this->walk.set_foot_height(config_.FOOT_HEIGHT);
  this->walk.set_left_right_swing(config_.Y_SWAP_AMPLITUDE);
  this->walk.set_top_down_swing(config_.Z_SWAP_AMPLITUDE);
  this->walk.set_pelvis_offset(config_.PELVIS_OFFSET);
  this->walk.set_arm_swing_gain(config_.ARM_SWING_GAIN);
  this->walk.set_trans_speed(config_.MAX_VEL);
  this->walk.set_rot_speed(config_.MAX_ROT_VEL);
}

void CeabotMazeAlgNode::init_headt_module(void) {
  //ROS_INFO("Darwin Ceabot Vision : HeadTracking Parameters Initialization");
  this->tracking_module.set_pan_range(config_.max_pan, config_.min_pan);
  this->tracking_module.set_tilt_range(config_.max_tilt, config_.min_tilt);
  this->tracking_module.set_pan_pid(config_.pan_p, config_.pan_i, config_.pan_d, config_.pan_i_clamp);
  this->tracking_module.set_tilt_pid(config_.tilt_p, config_.tilt_i, config_.tilt_d, config_.tilt_i_clamp);
}

void CeabotMazeAlgNode::scan_maze(void) {
    if (!this->search_started) {
      this->tracking_module.start_tracking(PI/2, 0.0);
      this->search_started = true;
    }
    else {
      if (this->first_scan_goal_achieved) {
        this->tracking_module.update_target(-PI/2, 0.0);
      }
      else if (this->second_scan_goal_achieved) {
        this->tracking_module.update_target(0.0, 0.0);
      }
      else if (this->third_scan_goal_achieved) {
        this->tracking_module.stop_tracking();
        this->darwin_state = CALCULATE_NEXT_MOVE;
        this->search_started = false;
        this->first_scan_goal_achieved = false;
        this->second_scan_goal_achieved = false;
        this->third_scan_goal_achieved = false;
      }
    }
}

void CeabotMazeAlgNode::calculate_next_move(void) {}

void CeabotMazeAlgNode::darwin_movement(void) {}

void CeabotMazeAlgNode::check_goal(void) {}

void CeabotMazeAlgNode::state_machine(void) {
  switch(this->darwin_state) {
    case IDLE:
      ROS_INFO("Darwin Ceabot Vision : state IDLE");
      break;

    case SCAN_MAZE:
      ROS_INFO("Scanning Maze...");
      scan_maze();
      break;

    case CALCULATE_NEXT_MOVE:
      ROS_INFO("Calculating next move...");
      calculate_next_move();
      break;

    case MOVEMENT:
      ROS_INFO("Moving...");
      darwin_movement();
      break;

    case CHECK_GOAL:
      ROS_INFO("Checking goal...");
      check_goal();
      break;
  }
}

/* main function */
int main(int argc,char *argv[]) {
  return algorithm_base::main<CeabotMazeAlgNode>(argc, argv, "ceabot_maze_alg_node");
}
