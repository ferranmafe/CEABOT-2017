#include "ceabot_stairs_alg_node.h"

CeabotStairsAlgNode::CeabotStairsAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<CeabotStairsAlgorithm>(),
  walk("walk_client"),
  tracking_module("headt_client"),
  action("action_client"),
  stairs("stairs_client")
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]
  this->first_bno_lecture = false;
  this->fallen = false;
  this->stairs_counter = 0;
  this->darwin_state = IDLE;
  this->event_start = false;

  // [init publishers]

  // [init subscribers]
  this->right_foot_data_subscriber_ = this->public_node_handle_.subscribe("right_foot_data", 1, &CeabotStairsAlgNode::right_foot_data_callback, this);
  pthread_mutex_init(&this->right_foot_data_mutex_,NULL);

  this->left_foot_data_subscriber_ = this->public_node_handle_.subscribe("left_foot_data", 1, &CeabotStairsAlgNode::left_foot_data_callback, this);
  pthread_mutex_init(&this->left_foot_data_mutex_,NULL);

  this->imu_subscriber_ = this->public_node_handle_.subscribe("imu", 1, &CeabotStairsAlgNode::imu_callback, this);
  pthread_mutex_init(&this->imu_mutex_,NULL);

  this->odom_subscriber_ = this->public_node_handle_.subscribe("odom", 1, &CeabotStairsAlgNode::odom_callback, this);
  pthread_mutex_init(&this->odom_mutex_,NULL);

  this->fallen_state_subscriber_ = this->public_node_handle_.subscribe("fallen_state", 1, &CeabotStairsAlgNode::fallen_state_callback, this);
  pthread_mutex_init(&this->fallen_state_mutex_,NULL);

  this->buttons_subscriber_ = this->public_node_handle_.subscribe("buttons", 1, &CeabotStairsAlgNode::buttons_callback, this);
  pthread_mutex_init(&this->buttons_mutex_,NULL);

  // [init services]

  // [init clients]

  // [init action servers]

  // [init action clients]
}

CeabotStairsAlgNode::~CeabotStairsAlgNode(void)
{
  // [free dynamic memory]
  pthread_mutex_destroy(&this->right_foot_data_mutex_);
  pthread_mutex_destroy(&this->left_foot_data_mutex_);
  pthread_mutex_destroy(&this->imu_mutex_);
  pthread_mutex_destroy(&this->odom_mutex_);
  pthread_mutex_destroy(&this->fallen_state_mutex_);
}

void CeabotStairsAlgNode::mainNodeThread(void)
{
  // [fill msg structures]

  // [fill srv structure and make request to the server]

  // [fill action structure and make request to the action server]

  // [publish messages]
  if (this->event_start) state_machine();
}

/*  [subscriber callbacks] */
void CeabotStairsAlgNode::buttons_callback(const humanoid_common_msgs::buttons::ConstPtr& msg)
{
  for (int i = 0; i < msg->name.size(); ++i) {
    if (msg->name[i] == "start" && msg->state[i] == true) {
	    this->darwin_state = START;
	    this->event_start = true;
    }
  }
}

void CeabotStairsAlgNode::buttons_mutex_enter(void)
{
  pthread_mutex_lock(&this->buttons_mutex_);
}

void CeabotStairsAlgNode::buttons_mutex_exit(void)
{
  pthread_mutex_unlock(&this->buttons_mutex_);
}

void CeabotStairsAlgNode::fallen_state_callback(const std_msgs::Int8::ConstPtr& msg) {
  if (msg->data == 0 || msg->data == 1) this->fallen = true;
}

void CeabotStairsAlgNode::fallen_state_mutex_enter(void) {
  pthread_mutex_lock(&this->fallen_state_mutex_);
}

void CeabotStairsAlgNode::fallen_state_mutex_exit(void) {
  pthread_mutex_unlock(&this->fallen_state_mutex_);
}

void CeabotStairsAlgNode::odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
  this->odom_x = msg->pose.pose.position.x;
  this->odom_y = msg->pose.pose.position.y;
 // std::cout << "Leyendo odometria..." << std::endl;

}

void CeabotStairsAlgNode::odom_mutex_enter(void) {
  pthread_mutex_lock(&this->odom_mutex_);
}

void CeabotStairsAlgNode::odom_mutex_exit(void) {
  pthread_mutex_unlock(&this->odom_mutex_);
}

void CeabotStairsAlgNode::right_foot_data_callback(const humanoid_common_msgs::ir_foot_data::ConstPtr& msg)
{
  for (int i = 0; i < msg->names.size(); ++i) {
    this->right_foot[msg->names[i]] = msg->status[i];
  }
}

void CeabotStairsAlgNode::right_foot_data_mutex_enter(void)
{
  pthread_mutex_lock(&this->right_foot_data_mutex_);
}

void CeabotStairsAlgNode::right_foot_data_mutex_exit(void)
{
  pthread_mutex_unlock(&this->right_foot_data_mutex_);
}
void CeabotStairsAlgNode::left_foot_data_callback(const humanoid_common_msgs::ir_foot_data::ConstPtr& msg)
{
  for (int i = 0; i < msg->names.size(); ++i) {
    this->left_foot[msg->names[i]] = msg->status[i];
  }
}

void CeabotStairsAlgNode::left_foot_data_mutex_enter(void)
{
  pthread_mutex_lock(&this->left_foot_data_mutex_);
}

void CeabotStairsAlgNode::left_foot_data_mutex_exit(void)
{
  pthread_mutex_unlock(&this->left_foot_data_mutex_);
}
void CeabotStairsAlgNode::imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {
  double bnoaux = tf::getYaw(msg->orientation);
  if (bnoaux < 0) bnoaux += 2*PI;
  this->bno055_measurement = bnoaux;

  if (this->first_bno_lecture) {
    this->first_bno_lecture = false;
    this->straight_forward_direction = bnoaux;
  }
}

void CeabotStairsAlgNode::imu_mutex_enter(void) {
  pthread_mutex_lock(&this->imu_mutex_);
}

void CeabotStairsAlgNode::imu_mutex_exit(void) {
  pthread_mutex_unlock(&this->imu_mutex_);
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void CeabotStairsAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_=config;
  this->alg_.unlock();
}

void CeabotStairsAlgNode::addNodeDiagnostics(void)
{
}

void CeabotStairsAlgNode::state_machine(void) {
  switch(this->darwin_state) {
    case IDLE:
      break;

    case START:
      ROS_INFO("START STATE");
      start_function();
      break;

    case WAIT_START_TIME:
      ROS_INFO("WAIT_START_TIME STATE");
      wait_start_time();
      break;

    case START_WALKING_PHASE_1:
      ROS_INFO("START WALKING PHASE 1 STATE");
      start_walking_phase_1();
      break;

    case CHECK_WALKING_DIRECTION_PHASE_1:
      ROS_INFO("CHECK WALKING DIRECTION PHASE 1 STATE");
      check_walking_direction_phase_1();
      break;

    case STOP_WALKING_PHASE_1:
      ROS_INFO("STOP WALKING PHASE 1 STATE");
      stop_walking_phase_1();
      break;

    case START_CLIMBING_STAIR:
      ROS_INFO("START CLIMBING STAIR STATE");
      start_climbing_stair();
      break;

    case COMPLETE_CLIMBING_STAIR:
      ROS_INFO("COMPLETE CLIMBING STAIR STATE");
      complete_climbing_stair();
      break;

    case START_WALKING_PHASE_2:
      ROS_INFO("START WALKING PHASE 2 STATE");
      start_walking_phase_2();
      break;

    case CHECK_WALKING_DIRECTION_PHASE_2:
      ROS_INFO("CHECK WALKING DIRECTION PHASE 2 STATE");
      check_walking_direction_phase_2();
      break;

    case START_WALKING_PHASE_3:
      ROS_INFO("START WALKING PHASE 3 STATE");
      start_walking_phase_3();
      break;

    case CHECK_WALKING_DIRECTION_PHASE_3:
      ROS_INFO("CHECK WALKING DIRECTION PHASE 3 STATE");
      check_walking_direction_phase_3();
      break;

    case STOP_WALKING_PHASE_3:
      ROS_INFO("STOP WALKING PHASE 3 STATE");
      stop_walking_phase_3();
      break;

    case CHECK_POSITION_FOR_DOWN_STAIR:
      ROS_INFO("CHECK POSITION FOR DOWN STAIR STATE");
      check_position_for_down_stair();
      break;

    case START_GO_BACK:
      ROS_INFO("START GO BACK STATE");
      start_go_back();
      break;

    case CHECK_GO_BACK:
      ROS_INFO("CHECK GO BACK STATE");
      check_go_back();
      break;

    case FINISH_GO_BACK:
      ROS_INFO("FINISH GO BACK STATE");
      finish_go_back();
      break;

    case START_ANGLE_CORRECTION:
      ROS_INFO("START ANGLE CORRECTION STATE");
      start_angle_correction();
      break;

    case CHECK_ANGLE_CORRECTION:
      ROS_INFO("CHECK ANGLE CORRECTION STATE");
      check_angle_correction();
      break;

    case FINISH_ANGLE_CORRECTION:
      ROS_INFO("FINISH ANGLE CORRECTION");
      finish_angle_correction();
      break;

    case DOWN_STAIR:
      ROS_INFO("DOWN STAIR STATE");
      down_stair();
      break;

    case COMPLETE_DOWN_STAIR:
      ROS_INFO("COMPLETE DOWN STAIR STATE");
      complete_down_stair();
      break;

    case START_WALKING_PHASE_4:
      ROS_INFO("START WALKING PHASE 4 STATE");
      start_walking_phase_4();
      break;

    case CHECK_WALKING_DIRECTION_PHASE_4:
      ROS_INFO("CHECK WALKING DIRECTION PHASE 4 STATE");
      check_walking_direction_phase_4();
      break;

    case STOP_WALKING_PHASE_4:
      ROS_INFO("STOP WALKING PHASE 4 STATE");
      stop_walking_phase_4();
      break;

    case FINISH:
      ROS_INFO("FINISH STATE");
      break;
  }
}

void CeabotStairsAlgNode::start_function(void) {
  init_walk_module();
  init_headt_module();
  this->darwin_state = WAIT_START_TIME;
  this->first_bno_lecture = true;
  this->timeout.start(ros::Duration(1.0));
}

void CeabotStairsAlgNode::wait_start_time(void) {
  if (this->timeout.timed_out()) {
    this->timeout.stop();
    this->darwin_state = START_WALKING_PHASE_2;
  }
}

void CeabotStairsAlgNode::start_walking_phase_1(void){
  this->walk.set_steps_size(this->config_.x_step, 0.0, 0.0);
  this->walk.set_steps_size(0.0, 0.0, 0.0);
  this->darwin_state = CHECK_WALKING_DIRECTION_PHASE_1;
}

void CeabotStairsAlgNode::check_walking_direction_phase_1(void) {
  double angle_diff = this->straight_forward_direction - this->bno055_measurement;
  this->walk.set_steps_size(this->config_.x_step, 0.0, saturate_angle(angle_diff));

  if (!this->left_foot["front_left"] && !this->left_foot["front_right"] && !this->right_foot["front_left"] && !this->right_foot["front_right"]) {
    this->darwin_state = STOP_WALKING_PHASE_1;
  }
}

void CeabotStairsAlgNode::stop_walking_phase_1(void) {
  if (this->walk.is_finished()) {
    this->darwin_state = START_CLIMBING_STAIR;
  }
  else this->walk.stop();
}

void CeabotStairsAlgNode::start_climbing_stair(void) {
  this->stairs.start(true);
  this->darwin_state = COMPLETE_CLIMBING_STAIR;
}

void CeabotStairsAlgNode::complete_climbing_stair(void) {
  if (this->stairs.is_finished()) {
    if(!this->fallen) ++this->stairs_counter;
    if (this->stairs_counter < 3) this->darwin_state = START_WALKING_PHASE_1;
    else this->darwin_state = START_WALKING_PHASE_2;
    this->fallen = false;
  }
}

void CeabotStairsAlgNode::start_walking_phase_2(void) {
  this->initial_x_phase_2 = this->odom_x;
  this->initial_y_phase_2 = this->odom_y;
  this->walk.set_steps_size(0.01, 0.0, 0.0);
  this->darwin_state = CHECK_WALKING_DIRECTION_PHASE_2;
}

void CeabotStairsAlgNode::check_walking_direction_phase_2(void) {
  double angle_diff = this->straight_forward_direction - this->bno055_measurement;
  this->walk.set_steps_size(0.01, 0.0, saturate_angle(angle_diff));

  double distance = sqrt(pow(this->initial_x_phase_2 - this->odom_x,2) + pow(this->initial_y_phase_2 - this->odom_y,2));
  std::cout << "Distance to goal: " << distance << std::endl;
  if (distance >= 0.25 - ERROR_PHASE_2) {
    this->darwin_state = START_WALKING_PHASE_3;
  }
}

void CeabotStairsAlgNode::start_walking_phase_3(void) {
  this->initial_x_phase_3 = this->odom_x;
  this->initial_y_phase_3 = this->odom_y;
  double angle_diff = this->straight_forward_direction - this->bno055_measurement;
  this->walk.set_steps_size(this->config_.x_step2, 0.0, saturate_angle(angle_diff));
  this->darwin_state = CHECK_WALKING_DIRECTION_PHASE_3;

  /*double angle_diff = this->straight_forward_direction - this->bno055_measurement;
  this->walk.set_steps_size(this->config_.x_step2, 0.0, saturate_angle(angle_diff));
  this->darwin_state = CHECK_WALKING_DIRECTION_PHASE_3;*/
}

void CeabotStairsAlgNode::check_walking_direction_phase_3(void) {
  double angle_diff = this->straight_forward_direction - this->bno055_measurement;
  this->walk.set_steps_size(this->config_.x_step2, 0.0, saturate_angle(angle_diff));

  double distance = sqrt(pow(this->initial_x_phase_3 - this->odom_x,2) + pow(this->initial_y_phase_3 - this->odom_y,2));
  if (distance >= 0.005 - ERROR_GO_BACK) {
    this->darwin_state = STOP_WALKING_PHASE_3;
  }
  /*double angle_diff = this->straight_forward_direction - this->bno055_measurement;
  this->walk.set_steps_size(this->config_.x_step2, 0.0, saturate_angle(angle_diff));

  if ((this->left_foot["down_left_front"] && this->left_foot["down_right_front"]) && (this->right_foot["down_left_front"] && this->right_foot["down_right_front"])) {
    this->darwin_state = STOP_WALKING_PHASE_3;
  }*/
}

void CeabotStairsAlgNode::stop_walking_phase_3(void) {
  if (this->walk.is_finished()) {
    this->darwin_state = CHECK_POSITION_FOR_DOWN_STAIR;
  }
  else this->walk.stop();
}

void CeabotStairsAlgNode::check_position_for_down_stair(void) {
    if (this->left_foot["down_left_middle"] || this->left_foot["down_right_middle"] || this->right_foot["down_left_middle"] || this->right_foot["down_right_middle"]) {
      this->darwin_state = START_GO_BACK;
    }
    else if (!this->left_foot["down_left_front"] && !this->left_foot["down_right_front"] && !this->right_foot["down_left_front"] && !this->right_foot["down_right_front"]) {
      this->darwin_state = START_WALKING_PHASE_3;
    }
    else if (!this->left_foot["down_left_front"] || !this->left_foot["down_right_front"] || !this->right_foot["down_left_front"] || !this->right_foot["down_right_front"]) {
      this->darwin_state = START_ANGLE_CORRECTION;
    }
    else this->darwin_state = DOWN_STAIR;
}

void CeabotStairsAlgNode::start_go_back(void) {
  this->initial_x_go_back = this->odom_x;
  this->initial_y_go_back = this->odom_y;

  double angle_diff = this->straight_forward_direction - this->bno055_measurement;
  this->walk.set_steps_size(-this->config_.x_step, 0.0, saturate_angle(angle_diff));

  this->darwin_state = CHECK_GO_BACK;
}

void CeabotStairsAlgNode::check_go_back(void) {
  double angle_diff = this->straight_forward_direction - this->bno055_measurement;
  this->walk.set_steps_size(-this->config_.x_step, 0.0, saturate_angle(angle_diff));

  double distance = sqrt(pow(this->initial_x_go_back - this->odom_x,2) + pow(this->initial_y_go_back - this->odom_y,2));
  std::cout << "Distance to goal: " << distance << std::endl;
  if (distance >= 0.07 - ERROR_GO_BACK) {
    this->darwin_state = CHECK_WALKING_DIRECTION_PHASE_3;
  }
}

void CeabotStairsAlgNode::finish_go_back(void) {
  if (this->walk.is_finished()) {
    this->darwin_state = START_WALKING_PHASE_3;
  }
  else this->walk.stop();
}

void CeabotStairsAlgNode::start_angle_correction(void) {
  this->initial_angle_to_correct = this->bno055_measurement;
  if (this->left_foot["down_left_front"] &&  !this->right_foot["down_right_front"]) {
    this->turn_direction = 1.0;
  }
  else {
    this->turn_direction = +1.0;
  }
  this->darwin_state = CHECK_ANGLE_CORRECTION;
}

void CeabotStairsAlgNode::check_angle_correction(void) {
  double angle_diff = this->initial_angle_to_correct - this->bno055_measurement;
  std::cout << "Angle turned: " << angle_diff << std::endl;
  this->walk.set_steps_size(-0.005, 0.0, this->turn_direction * 0.04);
  if (angle_diff >= 0.08 - ERROR_TURN || angle_diff <= -0.08 + ERROR_TURN) {
    this->darwin_state = FINISH_ANGLE_CORRECTION;
  }
}

void CeabotStairsAlgNode::finish_angle_correction(void) {
  if (this->walk.is_finished()) {
    this->darwin_state = CHECK_POSITION_FOR_DOWN_STAIR;
  }
  else this->walk.stop();
}

void CeabotStairsAlgNode::down_stair(void) {
  this->stairs.start(false);
  this->darwin_state = COMPLETE_DOWN_STAIR;
}

void CeabotStairsAlgNode::complete_down_stair(void) {
  if (this->stairs.is_finished()) {
    if (!this->fallen) ++this->stairs_counter;
    if (this->stairs_counter < 6) this->darwin_state = START_WALKING_PHASE_3;
    else this->darwin_state = START_WALKING_PHASE_4;
    this->fallen = false;
  }
}

void CeabotStairsAlgNode::start_walking_phase_4(void) {
  this->initial_x_phase_4 = this->odom_x;
  this->initial_y_phase_4 = this->odom_y;
  this->walk.set_steps_size(0.01, 0.0, 0.0);
  this->darwin_state = CHECK_WALKING_DIRECTION_PHASE_4;
}

void CeabotStairsAlgNode::check_walking_direction_phase_4(void) {
  double angle_diff = this->straight_forward_direction - this->bno055_measurement;
  this->walk.set_steps_size(0.01, 0.0, saturate_angle(angle_diff));

  double distance = sqrt(pow(this->initial_x_phase_4 - this->odom_x,2) + pow(this->initial_y_phase_4 - this->odom_y,2));
  if (distance >= 0.5 - ERROR_PHASE_2) {
    this->darwin_state = START_WALKING_PHASE_4;
  }
}

void CeabotStairsAlgNode::stop_walking_phase_4(void) {
  if (this->walk.is_finished()) {
    this->darwin_state = FINISH;
  }
  else this->walk.stop();
}
//------------------------------------------------------------------------------
void CeabotStairsAlgNode::init_walk_module(void) {
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

void CeabotStairsAlgNode::init_headt_module(void) {
  //ROS_INFO("Darwin Ceabot Vision : HeadTracking Parameters Initialization");
  this->tracking_module.set_pan_range(this->config_.max_pan, this->config_.min_pan);
  this->tracking_module.set_tilt_range(this->config_.max_tilt, this->config_.min_tilt);
  this->tracking_module.set_pan_pid(this->config_.pan_p, this->config_.pan_i, this->config_.pan_d, this->config_.pan_i_clamp);
  this->tracking_module.set_tilt_pid(this->config_.tilt_p, this->config_.tilt_i, this->config_.tilt_d, this->config_.tilt_i_clamp);
}

double CeabotStairsAlgNode::saturate_angle (double alpha) {
  if (alpha >= this->config_.MAX_UPPER_LIMIT) alpha = this->config_.MAX_UPPER_LIMIT;
  else if (alpha <= this->config_.MIN_UPPER_LIMIT and alpha >= this->config_.MIN_LOWER_LIMIT and alpha > 0) alpha = this->config_.MIN_UPPER_LIMIT;
  else if (alpha <= this->config_.MIN_UPPER_LIMIT and alpha >= this->config_.MIN_LOWER_LIMIT and alpha < 0) alpha = this->config_.MIN_LOWER_LIMIT;
  else if (alpha <= this->config_.MAX_LOWER_LIMIT) alpha = this->config_.MAX_LOWER_LIMIT;
  return alpha;
}

//------------------------------------------------------------------------------
/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<CeabotStairsAlgNode>(argc, argv, "ceabot_stairs_alg_node");
}
