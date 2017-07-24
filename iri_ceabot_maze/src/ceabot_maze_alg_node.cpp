#include "ceabot_maze_alg_node.h"

CeabotMazeAlgNode::CeabotMazeAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<CeabotMazeAlgorithm>(),
  walk("ceabot_maze_walk"),
  tracking_module("ceabot_maze_track"){
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]
  this->event_start = true;
  this->darwin_state = SCAN_MAZE;
  this->half_maze_achieved = false;

  this->search_started=false;

  this->direction = 0;
  this->turn_left = 1;
  // [init publishers]

  // [init subscribers]
  this->fallen_state_subscriber_ = this->public_node_handle_.subscribe("fallen_state", 1, &CeabotMazeAlgNode::fallen_state_callback, this);
  pthread_mutex_init(&this->fallen_state_mutex_,NULL);

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

CeabotMazeAlgNode::~CeabotMazeAlgNode(void){
  // [free dynamic memory]
  pthread_mutex_destroy(&this->fallen_state_mutex_);
  pthread_mutex_destroy(&this->imu_mutex_);
  pthread_mutex_destroy(&this->odom_mutex_);
  pthread_mutex_destroy(&this->joint_states_mutex_);
  pthread_mutex_destroy(&this->qr_pose_mutex_);
}

void CeabotMazeAlgNode::mainNodeThread(void){
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
void CeabotMazeAlgNode::fallen_state_callback(const std_msgs::Int8::ConstPtr& msg)
{
  //ROS_INFO("CeabotMazeAlgNode::fallen_state_callback: New Message Received");

  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
  //this->fallen_state_mutex_enter();
  this->fallen_state = msg->data;
  //std::cout << msg->data << std::endl;
  //unlock previously blocked shared variables
  //this->alg_.unlock();
  //this->fallen_state_mutex_exit();
}

void CeabotMazeAlgNode::fallen_state_mutex_enter(void)
{
  pthread_mutex_lock(&this->fallen_state_mutex_);
}

void CeabotMazeAlgNode::fallen_state_mutex_exit(void)
{
  pthread_mutex_unlock(&this->fallen_state_mutex_);
}

void CeabotMazeAlgNode::imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {
  //ROS_INFO("CeabotMazeAlgNode::imu_callback: New Message Received");

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
  this->odom_x = msg->pose.pose.position.x;
  this->odom_y = msg->pose.pose.position.y;
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
    /*if (this->searching_for_qr) {
      int zone_to_scan = actual_zone_to_scan();
      for (int i = 0; i < msg->tags.size(); ++i) {
        qr_info aux;
        aux.qr_tag = msg->tags[i].tag_id << " " <<
        aux.pos.x = msg->tags[i].position.x;    aux.pos.y = msg->tags[i].position.y;    aux.pos.z = msg->tags[i].position.z;
        aux.ori.x = msg->tags[i].orientation.x; aux.ori.y = msg->tags[i].orientation.y; aux.ori.z = msg->tags[i].orientation.z; aux.ori.w = msg->tags[i].orientation.w;

        qr_information[zone_to_scan][i] = aux;
      }
    }*/
    std::cout << msg->tags[0].tag_id << std::endl;
    std::cout << msg->tags[0].position.x << " " << msg->tags[0].position.y << " " << msg->tags[0].position.z << std::endl;
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
      this->searching_for_qr = false;
      this->tracking_module.start_tracking(goal_x, goal_y);
      this->search_started = true;
    }
    else {
      this->searching_for_qr = false;
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
        this->searching_for_qr = true;
        ros::Duration(0.5).sleep();
      }
      this->darwin_state = SCAN_MAZE;
    }
    if (this->current_angle_travelled == DegtoRad(360.0)) {
      this->search_started = false;
      this->darwin_state = SEARCH_FOR_GOAL_QR;
      this->direction = 0;
    }
  }
}

void CeabotMazeAlgNode::calculate_next_move(void) {}

void CeabotMazeAlgNode::darwin_movement_alpha(double alpha) { //angle to perform
    if (alpha < 0) {this->turn_left = -1; alpha = fabs(alpha);}
    else this->turn_left = 1;
    //alpha = fabs(alpha); //...
    this->mov_alpha_goal = get_goal_alpha(alpha, this->bno055_measurement);
    double alpha_sat = saturate_alpha(get_magnitude_alpha(this->mov_alpha_goal, this->bno055_measurement));

    this->walk.set_steps_size(0.0, 0.0, this->config_.p * alpha_sat * this->turn_left);
    this->darwin_state = CHECK_GOAL_ALPHA;

}

void CeabotMazeAlgNode::darwin_movement_x(double x) {
  std::pair<double, double> parr = get_goal_xy(x, this->odom_x, this->odom_y, this->nm_alpha);
  this->mov_x_goal = parr.first;
  this->mov_y_goal = parr.second;

  double x_sat = saturate_movement(x);
  this->walk.set_steps_size(x_sat, 0.0, 0.0);
  this->darwin_state = CHECK_GOAL_XY;
}

void CeabotMazeAlgNode::check_goal_alpha(double goal) {
  double diff = fabs(goal - this->bno055_measurement);
  if (diff >= -this->config_.ERROR_PERMES and diff <= this->config_.ERROR_PERMES) {
    ROS_INFO("Alpha goal achieved, soon I'll be moving on!");
    this->walk.stop();
    this->darwin_state = MOVEMENT_X; //We first perform the rotation, the the transition is made between MOVEMENT_ALPHA and MOVEMENT_X
  }
  else {
    double tom = saturate_alpha(this->config_.p * diff * this->turn_left);
    std::cout << "tom: " << tom << " goal: " << goal << " diff: " << diff << std::endl;
    this->walk.set_steps_size(0.0, 0.0, tom);
  }
}

void CeabotMazeAlgNode::check_goal_xy(double goalx, double goaly) {
  double diffx = fabs(goalx - this->odom_x);
  double diffy = fabs(goaly - this->odom_y);
  if ((diffx >= -this->config_.ERROR_PERMES and diffx <= this->config_.ERROR_PERMES) and (diffy >= -this->config_.ERROR_PERMES and diffy <= this->config_.ERROR_PERMES)) {
    ROS_INFO("X goal achieved, soon I'll be moving on!");
    this->walk.stop();
    this->darwin_state = IDLE;
    //cambio de estado HERE PLZ
  }
  else {
    double tom = saturate_movement(this->config_.p * ((diffx+diffy)/2.0));
    this->walk.set_steps_size(tom, 0.0, 0.0);
  }
}

void CeabotMazeAlgNode::state_machine(void) {
  switch(this->darwin_state) {
    case IDLE:
      ROS_INFO("Darwin Ceabot Vision : state IDLE");
      break;

    case SCAN_MAZE:
      //ROS_INFO("Scanning Maze...");
      //scan_maze();
      break;

    case WAIT_FOR_SCAN:
      ROS_INFO("Waiting for scan...");
      wait_for_scan();
      break;

    case SEARCH_FOR_GOAL_QR:
      ROS_INFO("Trying to find the goal QR...");
      search_for_goal_qr ();
      break;

    case CALCULATE_DENSITY:
      ROS_INFO("Calculating density of each zone...");
      calculate_density();
      break;

    case FIND_HOLES:
      ROS_INFO("Finding holes for each zone...");
      find_holes();
      break;

    case CALCULATE_MOVEMENT:
      ROS_INFO("Calculating next move...");
      calculate_next_move();
      this->nm_alpha = 3.141519;
      this->nm_x = 100.0;
      this->darwin_state = MOVEMENT_ALPHA;
      break;

    case MOVEMENT_ALPHA:
      ROS_INFO("Performing a rotation...");
      //nm_alpha stands for next movement alpha, etc
      darwin_movement_alpha(this->nm_alpha);
      break;

    case MOVEMENT_X:
      ROS_INFO("Moving...");
      darwin_movement_x(this->nm_x);
      break;

    case CHECK_GOAL_ALPHA:
      ROS_INFO("Checking alpha goal...");
      check_goal_alpha(this->mov_alpha_goal);
      break;

    case CHECK_GOAL_XY:
      ROS_INFO("Checking x goal...");
      check_goal_xy(this->mov_x_goal, this->mov_y_goal);
      break;
  }
}

double CeabotMazeAlgNode::DegtoRad (double degree) {
  return (degree * PI)/180.0;
}

double CeabotMazeAlgNode::get_magnitude_alpha (double alpha, double beta) {
  //std::cout << alpha*180.0/PI << ' ' << beta*180.0/PI << std::endl;
  double mgn = alpha - beta;
  if (mgn > PI)  //That means we are not performing the best move
    mgn -= 2*PI;
  else if (mgn < (-PI)) //Same case
    mgn += 2*PI;
  //std::cout << mgn << std::endl;
  return fabs(mgn); //We do not return the abs, just because the direction matters

}

double CeabotMazeAlgNode::get_goal_alpha (double alpha, double beta) { // "Calculate and Normalize"
  /*double goal = alpha + beta; //Alpha is magnitude and beta is the actual yaw...
  if (goal < (-PI)) goal += 2*PI;
  else if (goal > PI) goal -= 2*PI;

  return goal;*/

  double goal = alpha + beta;
  if (goal < 0) goal += (2*PI);
  else if (goal > (2*PI)) goal -= (2*PI);

  return goal;

}

double CeabotMazeAlgNode::get_magnitude_x (double fin, double ini) {
  return fin - ini;
}

std::pair <double, double> CeabotMazeAlgNode::get_goal_xy (double x, double inix, double iniy, double alpha) {
  std::pair <double, double> ret;

  ret.first = (cos(alpha) * x) + inix;
  ret.second = (sin(alpha) * x) + iniy;

  return ret;
}

double CeabotMazeAlgNode::saturate_alpha (double alpha) {
  /*
    MAX_UPPER_LIMIT
    MIN_UPPER_LIMIT
    MAX_LOWER_LIMIT
    MIN_LOWER_LIMIT
   */
  //std::cout << "pre:ALpha: " << alpha << std::endl;
  if (alpha >= this->config_.MAX_UPPER_LIMIT) alpha = this->config_.MAX_UPPER_LIMIT;
  else if (alpha <= this->config_.MIN_UPPER_LIMIT and alpha >= this->config_.MIN_LOWER_LIMIT and alpha > 0) alpha = this->config_.MIN_UPPER_LIMIT;
  else if (alpha <= this->config_.MIN_UPPER_LIMIT and alpha >= this->config_.MIN_LOWER_LIMIT and alpha < 0) alpha = this->config_.MIN_LOWER_LIMIT;
  else if (alpha <= this->config_.MAX_LOWER_LIMIT) alpha = this->config_.MAX_LOWER_LIMIT;
  //std::cout << "La saturacion nos da:: " << alpha << ' ' << std::endl;

  return alpha;
}

double CeabotMazeAlgNode::saturate_movement (double x) {
  double sat = x;
  if (x > 0.05) sat = 0.05;
  else if (x < -0.05) sat = -0.05;

  return sat; //Obviously incomplete
}

void CeabotMazeAlgNode::search_for_goal_qr (void) {
    this->wall_qr_goal_found = false;
    if (!this->half_maze_achieved) {
        for (int i = 0; i < this->qr_information.size(); ++i) {
            for (int j = 0; j < this->qr_information[i].size(); ++j) {
                std::pair<std::string, int> aux = divide_qr_tag (this->qr_information[i][j].qr_tag);

                if (aux.second > 6 and aux.first == "N") {
                    this->wall_qr_goal_found = true;
                    this->darwin_state = CALCULATE_MOVEMENT;
                }
            }
        }
    }
    else {
        for (int i = 0; i < this->qr_information.size(); ++i) {
            for (int j = 0; j < this->qr_information[i].size(); ++j) {
                std::pair<std::string, int> aux = divide_qr_tag (this->qr_information[i][j].qr_tag);

                if (aux.second > 6 and aux.first == "S") {
                    this->wall_qr_goal_found = true;
                    this->darwin_state = CALCULATE_MOVEMENT;
                }
            }
        }
    }
    this->darwin_state = CALCULATE_DENSITY;
}

void CeabotMazeAlgNode::calculate_density(void) {
    for (int i = 0; i < this->qr_information.size(); ++i) {
        double obstacles_on_the_zone = 0.0;
        std::vector<bool> obstacles (6, false);
        for (int j = 0; j < this->qr_information[i].size(); ++j) {
            std::pair<std::string, int> actual_qr = divide_qr_tag (this->qr_information[i][j].qr_tag);
            if (actual_qr.second <= 6 and !obstacles[actual_qr.second - 1]) {
                obstacles_on_the_zone += 1.0;
                obstacles[actual_qr.second - 1] = true;
            }
        }
        this->ocupation[i] = obstacles_on_the_zone / 6.0;
    }
    this->darwin_state = FIND_HOLES;
}

void CeabotMazeAlgNode::find_holes(void) {

}

std::pair<std::string, int> CeabotMazeAlgNode::divide_qr_tag (std::string qr_tag) {
    std::pair<std::string, int> aux;

    aux.first = qr_tag[0];
    qr_tag.erase(0, 1);
    aux.second = atoi(qr_tag.c_str());

    return aux;
}

int CeabotMazeAlgNode::actual_zone_to_scan(void) {
    if (this->current_pan_angle >= DegtoRad(-112.5) and this->current_pan_angle <= DegtoRad(-67.5)) {
        return 0;
    }
    else if (this->current_pan_angle > DegtoRad(-67.5) and this->current_pan_angle <= DegtoRad(-22.5)) {
        return 1;
    }
    else if (this->current_pan_angle > DegtoRad(-22.5) and this->current_pan_angle <= DegtoRad(22.5)) {
        return 2;
    }
    else if (this->current_pan_angle > DegtoRad(22.5) and this->current_pan_angle <= DegtoRad(67.5)) {
        return 3;
    }
    else if (this->current_pan_angle > DegtoRad(67.5) and this->current_pan_angle <= DegtoRad(112.5)) {
        return 4;
    }
}

/* main function */
int main(int argc,char *argv[]) {
  return algorithm_base::main<CeabotMazeAlgNode>(argc, argv, "ceabot_maze_alg_node");
}
