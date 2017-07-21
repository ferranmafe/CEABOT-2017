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
    }
    else if (msg->name[i]=="j_tilt"){
      this->current_tilt_angle=(msg->position[i]);
    }
  }
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
         qr_info aux;
         aux.pos.x = msg->tags[i].position.x;    aux.pos.y = msg->tags[i].position.y;    aux.pos.z = msg->tags[i].position.z;
         aux.ori.x = msg->tags[i].orientation.x; aux.ori.y = msg->tags[i].orientation.y; aux.ori.z = msg->tags[i].orientation.z; aux.ori.w = msg->tags[i].orientation.w;

         qr_tags_detected[msg->tags[i].tag_id] = aux;

         //std::cout << msg->tags[i].tag_id << std::endl;

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
  this->walk.set_x_offset(this->config_.X_OFFSET);
  this->walk.set_y_offset(this->config_.Y_OFFSET);
  this->walk.set_z_offset(this->config_.Z_OFFSET);
  this->walk.set_roll_offset(this->config_.R_OFFSET);
  this->walk.set_pitch_offset(this->config_.P_OFFSET);
  this->walk.set_yaw_offset(this->config_.A_OFFSET);
  this->walk.set_hip_pitch_offset(this->config_.HIP_PITCH_OFFSET);
  this->walk.set_period(this->config_.PERIOD_TIME);
  this->walk.set_ssp_dsp_ratio(this->config_.DSP_RATIO);
  this->walk.set_fwd_bwd_ratio(this->config_.STEP_FB_RATIO);
  this->walk.set_foot_height(this->config_.FOOT_HEIGHT);
  this->walk.set_left_right_swing(this->config_.Y_SWAP_AMPLITUDE);
  this->walk.set_top_down_swing(this->config_.Z_SWAP_AMPLITUDE);
  this->walk.set_pelvis_offset(this->config_.PELVIS_OFFSET);
  this->walk.set_arm_swing_gain(this->config_.ARM_SWING_GAIN);
  this->walk.set_trans_speed(this->config_.MAX_VEL);
  this->walk.set_rot_speed(this->config_.MAX_ROT_VEL);
}

void CeabotMazeAlgNode::init_headt_module(void) {
  //ROS_INFO("Darwin Ceabot Vision : HeadTracking Parameters Initialization");
  this->tracking_module.set_pan_range(this->config_.max_pan, this->config_.min_pan);
  this->tracking_module.set_tilt_range(this->config_.max_tilt, this->config_.min_tilt);
  this->tracking_module.set_pan_pid(this->config_.pan_p, this->config_.pan_i, this->config_.pan_d, this->config_.pan_i_clamp);
  this->tracking_module.set_tilt_pid(this->config_.tilt_p, this->config_.tilt_i, this->config_.tilt_d, this->config_.tilt_i_clamp);
}

void CeabotMazeAlgNode::scan_maze(void) {
    if (!this->search_started) {
      this->direction = 1;
      this->goal_x = PI/2.0;
      this->goal_y = PI/4.5;
      this->current_angle_travelled = 0.0;
      this->tracking_module.start_tracking(goal_x, goal_y);
      this->search_started = true;
    }
    else {
      this->tracking_module.update_target(goal_x, goal_y);
    }
    this->darwin_state = WAIT_FOR_SCAN;
}

void CeabotMazeAlgNode::wait_for_scan(void) {
  double aux_pan = this->current_pan_angle;
  if (this->search_started) {
    if (aux_pan >= (this->goal_x - ERROR) and aux_pan <= (this->goal_x + ERROR)) { //Sub-goal achieved
      ROS_INFO("Ceabot Maze : Goal achieved, moving on for the next one!");
      if (fabs(this->goal_x) == PI/2.0) this->direction *= -1;
      if (this->direction == 1) {
        this->goal_x += PI/2.0;
        this->current_angle_travelled += PI/2.0;
      }
      else {
        this->goal_x -= PI/4.0;
        this->current_angle_travelled += PI/4.0;
      }
      if (this->direction == -1) {
        ros::Duration(0.5).sleep();
      }
      this->darwin_state = SCAN_MAZE;
    }
    if (this->current_angle_travelled == DegtoRad(360.0)) {
      this->search_started = false;
      this->darwin_state = CALCULATE_NEXT_MOVE;

      print_map(this->qr_tags_detected);
    }
      /*if (aux_pan >= DegtoRad(85.0)) {
        //this->first_scan_goal_achieved = true;
        goal_x = -PI/2;
      }
      if (aux_pan <= DegtoRad(-85.0)) {
        //this->second_scan_goal_achieved = true;
        //this->first_scan_goal_achieved = false;
        goal_x = 0.0;
      }
      if (this->second_scan_goal_achieved and aux_pan >= (0.0 - ERROR) and aux_pan <= (0.0 + ERROR)) {
         //this->third_scan_goal_achieved = true;
         //this->second_scan_goal_achieved = false;
      }*/
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
      //ROS_INFO("Scanning Maze...");
      //scan_maze();
      print_map(this->qr_tags_detected);
      break;

    case WAIT_FOR_SCAN:
      ROS_INFO("Waiting for scan...");
      wait_for_scan();
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

double CeabotMazeAlgNode::DegtoRad (double degree) {
  return (degree * PI)/180.0;
}

void CeabotMazeAlgNode::print_map(std::map<std::string, qr_info>& map) {
  std::map<std::string, qr_info>::iterator it;

  for (it = map.begin(); it != map.end(); ++it) {
    std::cout << "QR TAG: " << it->first << std:: endl;
    std::cout << "Position parameters:" << std::endl;
    std::cout << "x: " << it->second.pos.x << " y: " << it->second.pos.y << " z: " << it->second.pos.z << std:: endl;
    //std::cout << "Orientation parameters:" << std::endl;
    //std::cout << "x: " << it->second.ori.x << " y: " << it->second.ori.y << " z: " << it->second.ori.z << " w: " << it->second.ori.w << std:: endl;
    std::cout << std::endl;
    std::cout << "---------------------" << std::endl;
    std::cout << std::endl;
  }
}

/* main function */
int main(int argc,char *argv[]) {
  return algorithm_base::main<CeabotMazeAlgNode>(argc, argv, "ceabot_maze_alg_node");
}
