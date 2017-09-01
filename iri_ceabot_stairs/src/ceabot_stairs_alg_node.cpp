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
  this->stairs_counter = 0;
  this->darwin_state = START;

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
}

void CeabotStairsAlgNode::mainNodeThread(void)
{
  // [fill msg structures]

  // [fill srv structure and make request to the server]

  // [fill action structure and make request to the action server]

  // [publish messages]

  state_machine();
}

/*  [subscriber callbacks] */
void CeabotStairsAlgNode::odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
  this->odom_x = msg->pose.pose.position.x;
  this->odom_y = msg->pose.pose.position.y;
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

    case STOP_WALKING_PHASE_2:
      ROS_INFO("STOP WALKING PHASE 2 STATE");
      stop_walking_phase_2();
      break;

    case CHECK_POSITION_FOR_DOWN_STAIR:
      ROS_INFO("CHECK POSITION FOR DOWN STAIR STATE");
      check_position_for_down_stair();
      break;

    case GO_BACK:
      ROS_INFO("GO_BACK STATE");
      go_back();
      break;

    case CHECK_GO_BACK:
      ROS_INFO("CHECK GO BACK STATE");
      check_go_back();
      break;

    case STOP_GO_BACK:
      ROS_INFO("STOP GO BACK STATE");
      stop_go_back();
      break;

    case DOWN_STAIR:
      ROS_INFO("DOWN STAIR STATE");
      down_stair();
      break;

    case COMPLETE_DOWN_STAIR:
      ROS_INFO("COMPLETE DOWN STAIR STATE");
      complete_down_stair();
      break;

    case FINISH:
      ROS_INFO("FINISH STATE");
      finish();
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
  this->darwin_state = CHECK_WALKING_DIRECTION_PHASE_1;
}

void CeabotStairsAlgNode::check_walking_direction_phase_1(void) {
  double angle_diff = this->straight_forward_direction - this->bno055_measurement;
  this->walk.set_steps_size(this->config_.x_step, 0.0, saturate_angle(angle_diff));

  if ((!this->left_foot["front_left"] || !this->left_foot["front_right"]) && (!this->right_foot["front_left"] || !this->right_foot["front_right"])) {
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
    ++this->stairs_counter;
    if (this->stairs_counter < 3) this->darwin_state = START_WALKING_PHASE_1;
    else this->darwin_state = START_WALKING_PHASE_2;
  }
}

void CeabotStairsAlgNode::start_walking_phase_2(void) {
  this->walk.set_steps_size(this->config_.x_step, 0.0, 0.0);
  this->darwin_state = CHECK_WALKING_DIRECTION_PHASE_2;
}

void CeabotStairsAlgNode::check_walking_direction_phase_2(void) {
  double angle_diff = this->straight_forward_direction - this->bno055_measurement;
  this->walk.set_steps_size(this->config_.x_step, 0.0, saturate_angle(angle_diff));

  if ((this->left_foot["down_left_front"] && this->left_foot["down_right_front"]) && (this->right_foot["down_left_front"] && this->right_foot["down_right_front"])) {
    if ((this->left_foot["down_left_middle"] || this->left_foot["down_right_middle"]) && (this->right_foot["down_left_middle"] || this->right_foot["down_right_middle"])) {
      this->darwin_state = STOP_WALKING_PHASE_2;
    }
  }
}

void CeabotStairsAlgNode::stop_walking_phase_2(void) {
  if (this->walk.is_finished()) {
    this->darwin_state = CHECK_POSITION_FOR_DOWN_STAIR;
  }
  else this->walk.stop();
}

void CeabotStairsAlgNode::check_position_for_down_stair(void) {
  if ((this->left_foot["down_left_front"] && this->left_foot["down_right_front"]) && (this->right_foot["down_left_front"] && this->right_foot["down_right_front"])) {
    if ((this->left_foot["down_left_middle"] || this->left_foot["down_right_middle"]) && (this->right_foot["down_left_middle"] || this->right_foot["down_right_middle"])) {
      if (!(this->left_foot["down_left_rear"] || this->left_foot["down_right_rear"] || this->right_foot["down_left_rear"] || this->right_foot["down_right_rear"])) {
        this->darwin_state = DOWN_STAIR;
      }
      else this->darwin_state = GO_BACK;
    }
    else this->darwin_state = GO_BACK;
  }
  else this->darwin_state = GO_BACK;
}

void CeabotStairsAlgNode::go_back(void) {
  this->initial_x_go_back = this->odom_x;
  this->initial_y_go_back = this->odom_y;
  this->darwin_state = CHECK_GO_BACK;

  double angle_diff = this->straight_forward_direction - this->bno055_measurement;
  this->walk.set_steps_size(-this->config_.x_step, 0.0, saturate_angle(angle_diff));
}

void CeabotStairsAlgNode::check_go_back(void) {
  double distance = sqrt(pow(this->initial_x_go_back - this->odom_x, 2) + pow(this->initial_y_go_back - this->odom_y, 2));
  if (distance >= 0.1 + ERROR_BACK || distance <= 0.1 - ERROR_BACK) {
    this->darwin_state = STOP_GO_BACK;
  }
  else {
    double angle_diff = this->straight_forward_direction - this->bno055_measurement;
    this->walk.set_steps_size(-this->config_.x_step, 0.0, saturate_angle(angle_diff));
  }
}

void CeabotStairsAlgNode::stop_go_back(void) {
  if (this->walk.is_finished()) {
    this->darwin_state = START_WALKING_PHASE_2;
  }
  else this->walk.stop();
}

void CeabotStairsAlgNode::down_stair(void) {
  this->stairs.start(false);
  this->darwin_state = COMPLETE_DOWN_STAIR;
}

void CeabotStairsAlgNode::complete_down_stair(void) {
  if (this->stairs.is_finished()) {
    ++this->stairs_counter;
    if (this->stairs_counter < 6) this->darwin_state = START_WALKING_PHASE_2;
    else this->darwin_state = FINISH;
  }
}

void CeabotStairsAlgNode::finish(void){}
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
  std::cout << "pre:ALpha: " << alpha << std::endl;
  if (alpha >= this->config_.MAX_UPPER_LIMIT) alpha = this->config_.MAX_UPPER_LIMIT;
  else if (alpha <= this->config_.MIN_UPPER_LIMIT and alpha >= this->config_.MIN_LOWER_LIMIT and alpha > 0) alpha = this->config_.MIN_UPPER_LIMIT;
  else if (alpha <= this->config_.MIN_UPPER_LIMIT and alpha >= this->config_.MIN_LOWER_LIMIT and alpha < 0) alpha = this->config_.MIN_LOWER_LIMIT;
  else if (alpha <= this->config_.MAX_LOWER_LIMIT) alpha = this->config_.MAX_LOWER_LIMIT;
  std::cout << "La saturacion nos da:: " << alpha << ' ' << std::endl;
  return alpha;
}

//------------------------------------------------------------------------------
/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<CeabotStairsAlgNode>(argc, argv, "ceabot_stairs_alg_node");
}
