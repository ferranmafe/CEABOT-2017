#include "ceabot_maze_alg_node.h"

CeabotMazeAlgNode::CeabotMazeAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<CeabotMazeAlgorithm>(),
  walk("ceabot_maze_walk"),
  tracking_module("ceabot_maze_track"),
  action("action_client"){
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]
  this->event_start = true;
  this->time_to_wait = 5.0;
  this->darwin_state = SCAN_MAZE;
  this->state_stn = CHECK_NORTH;
  this->half_maze_achieved = false;
  this->first_bno_lecture = true;
  this->half_maze_achieved = false;
  this->next_x_mov = 0.0;
  this->next_z_mov = 0.0;
  this->nm_x = 1.0;
  this->nm_alpha =  -0.45;
  this->search_started=false;
  this->direction = 0;
  this->turn_left = 1;
  this->fallen_state = 0;
  this->way_zaxis_grow = 1.0;
  this->way_xaxis_grow = 0.0;

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

    if (this->half_maze_achieved) {this->way_zaxis_grow = 1.0; this->way_xaxis_grow = 1.0;}
    else {this->way_zaxis_grow = -1.0; this->way_xaxis_grow = -1.0;}
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
void CeabotMazeAlgNode::fallen_state_callback(const std_msgs::Int8::ConstPtr& msg) {
  //ROS_INFO("CeabotMazeAlgNode::fallen_state_callback: New Message Received");

  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
  //this->fallen_state_mutex_enter();
  this->fallen_state = msg->data;
  /*if (msg->data == 0 or msg->data == 1) {
    this->darwin_state = FALLEN_DARWIN; *///No creo que lo mejor sea saltar directamente, habria que guardarse el estado anterior o algo por el estilo... (idk)
    //std::cout << "Fallen detectado con el fallen state callback" << std::endl;
  //}

  //std::cout << msg->data << std::endl;
  //unlock previously blocked shared variables
  //this->alg_.unlock();
  //this->fallen_state_mutex_exit();
}

void CeabotMazeAlgNode::fallen_state_mutex_enter(void) {
  pthread_mutex_lock(&this->fallen_state_mutex_);
}

void CeabotMazeAlgNode::fallen_state_mutex_exit(void) {
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
  if (this->first_bno_lecture) {
    this->first_bno_lecture = false;
    this->north_of_the_maze = bnoaux;
  }
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
  /*if (this->fallen_state != 0) {
    this->odom_xpre_fall = msg->pose.pose.position.x;
    this->odom_ypre_fall = msg->pose.pose.position.y;
  }
  else {
    this->darwin_state = FALLEN_DARWIN;
    std::cout << "Fallen detectado con el odom callback" << std::endl;
  }*/

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

  std::cout << "entro " << "searching_for_qr: " << this->searching_for_qr  << std::endl;
  geometry_msgs::PoseStamped transformed_pose;
  geometry_msgs::PoseStamped pose;
  if (msg->tags.size()>0) {
    if (this->searching_for_qr) {
      std::cout << "I'm searching for QR!" << std::endl;

      for (int i = 0; i < msg->tags.size(); ++i) {
        bool ready_to_transform = false;
        fill_PoseStamped(i, msg, pose);
        //std::cout << msg->tags[i].header.frame_id << std::endl;
        ready_to_transform = listener.waitForTransform("darwin/base_link", msg->tags[i].header.frame_id , msg->tags[i].header.stamp, ros::Duration(0.3), ros::Duration(0.08333));
        //std::cout << "ready_to_transform??: " << ready_to_transform << std::endl;
        if (ready_to_transform) {
          listener.transformPose("darwin/base_link", pose, transformed_pose);

          qr_info aux;
          aux.header = msg->tags[i].header;
          aux.qr_tag = msg->tags[i].tag_id;

          aux.pos.x = transformed_pose.pose.position.x; aux.pos.y = transformed_pose.pose.position.y; aux.pos.z = transformed_pose.pose.position.z;
          aux.ori.x = transformed_pose.pose.orientation.x; aux.ori.y = transformed_pose.pose.orientation.y; aux.ori.z = transformed_pose.pose.orientation.z; aux.ori.w = transformed_pose.pose.orientation.w;

          std::cout << std::endl;
          std::cout << "Tag ID: " << msg->tags[i].tag_id << std::endl;
          std::cout <<  "Old X pos: " << msg->tags[i].position.x << " Old Y pos: " << msg->tags[i].position.y << " Old Z pos: " <<  msg->tags[i].position.z << std::endl;
          std::cout << "New X pos: " << aux.pos.x << " New Y pos: " << aux.pos.y << " New Z pos: " << aux.pos.z << std::endl;
          std::cout << std::endl;

          qr_info_pre_processing[aux.qr_tag].push_back(aux);
        }
      }
      //std::sort(vec_aux.begin(), vec_aux.end(), distance_sort);
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
      qr_info_pre_processing.clear();
      this->direction = 1;
      this->goal_x = PI/2.0;
      this->goal_y = PI/4.5;
      this->current_angle_travelled = 0.0;
      this->tracking_module.start_tracking(goal_x, goal_y);
      this->search_started = true;
      ROS_INFO ( "WOLOLOOOO");
    }
    else {
      this->tracking_module.update_target(goal_x, goal_y);
    }

    this->searching_for_qr = false;
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
      if (this->direction == -1 or (aux_pan >= (-PI/2.0 - ERROR) and aux_pan <= (-PI/2.0 + ERROR))) {
        //std::cout << "wait for scan entro" << std::endl;
        this->searching_for_qr = true;
        this->time_to_wait = 2.0;
        this->timeout.start(ros::Duration(this->time_to_wait)); //Start a time out and go to wait
        this->darwin_state = WAIT;
      }
      if (this->darwin_state != WAIT) this->darwin_state = SCAN_MAZE;
    }
    if (this->current_angle_travelled == DegtoRad(360.0)) {
      if (this->tracking_module.is_finished()) {
        this->search_started = false;
        this->darwin_state = PROCESS_DATA;
        this->direction = 0;
      }
      else {
        this->tracking_module.stop_tracking();
        this->darwin_state = WAIT_TRACKING;
      }


    }
  }
}

void CeabotMazeAlgNode::calculate_next_move(void) {
    std::cout << "voy a calcular el siguiente movimiento con: " << this->next_x_mov << ' ' << this->next_z_mov << std::endl;
    this->nm_x = distance_to_xy(this->next_x_mov, this->next_z_mov);
    if (this->nm_x > 1.0) this->nm_x = 1.0;
    else this->nm_x *= 0.8;
    this->nm_alpha =  (atan2(this->next_x_mov, this->next_z_mov));

    this->darwin_state = MOVEMENT_ALPHA;

    std::cout << "rad: " << nm_x << " alpha: " << nm_alpha << std::endl;
}

void CeabotMazeAlgNode::darwin_movement_alpha(double alpha) { //angle to perform
    if (alpha < 0) this->turn_left = -1;
    else this->turn_left = 1;

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
    this->walk.set_steps_size(0.0, 0.0, tom);

  }
}

void CeabotMazeAlgNode::check_goal_xy(double goalx, double goaly) {
  double diffx = fabs(goalx - this->odom_x);
  double diffy = fabs(goaly - this->odom_y);
  std::cout << "Diferència X: " << diffx << " Diferència Y: " << diffy << std::endl;
  if ((diffx >= -this->config_.ERROR_PERMES and diffx <= this->config_.ERROR_PERMES) and (diffy >= -this->config_.ERROR_PERMES and diffy <= this->config_.ERROR_PERMES)) {
    ROS_INFO("XY goal achieved, soon I'll be moving on!");
    this->walk.stop();
    this->darwin_state = IDLE;
    this->state_stn = CHECK_NORTH;
  }
  else {
    double tom = saturate_movement(this->config_.p * ((diffx+diffy)/2.0));
    this->walk.set_steps_size(tom, 0.0, 0.0);
  }
}

void CeabotMazeAlgNode::state_machine(void) { //Yo pondria fuera de la funcion los cambios de estado que son unicos, o sea, que son transiciones simples...
  switch(this->darwin_state) {
    case IDLE:
      ROS_INFO("Darwin Ceabot Maze : state IDLE");
      straight_to_north();
      //this->darwin_state = SCAN_MAZE;
      break;

    case SCAN_MAZE:
      ROS_INFO("Scanning Maze...");
      scan_maze();
      break;

    case WAIT_FOR_SCAN:
      ROS_INFO("Waiting for scan...");
      wait_for_scan();
      break;

    case PROCESS_DATA:
      ROS_INFO("Processing data...");
      process_data();
      break;

    case SEARCH_FOR_GOAL_QR:
      ROS_INFO("Trying to find the goal QR...");
      search_for_goal_qr ();
      break;

    case FIND_HOLES:
      ROS_INFO("Finding holes for each zone...");
      find_holes();
      break;

    case CALCULATE_MOVEMENT:
      ROS_INFO("Calculating next move...");
      calculate_next_move();
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
      ROS_INFO("Checking XY goal...");
      check_goal_xy(this->mov_x_goal, this->mov_y_goal);
      break;

    case FALLEN_DARWIN:
      ROS_INFO("Checking Darwin integrity...");
      /*if (!this->walk.is_finished()) this->walk.stop();
      if (this->action.is_finished()) {
        if (this->fallen_state == 0) {
          this->action.execute(10);
        }
        else if (this->fallen_state == 1) {
          this->action.execute(11);
        }
      }

      this->darwin_state = IS_DARWIN_STANDING;
      */
      break;

    case IS_DARWIN_STANDING:
      ROS_INFO("Waiting for Darwin to be upwards...");
      /*if (this->action.is_finished()) {
        this->darwin_state = MOVEMENT_ALPHA;
        if (!this->half_maze_achieved){
          this->nm_alpha = this->north_of_the_maze - this->bno055_measurement;
        }
        else {
          double south_of_the_maze = PI - this->north_of_the_maze;
          if (south_of_the_maze < 0.0) south_of_the_maze += 2.0 * PI;
          else if (south_of_the_maze >= 2.0 * PI) south_of_the_maze -= 2.0 * PI;

          this->nm_alpha = south_of_the_maze - this->bno055_measurement;
        }
        this->nm_x = 0.0;
      }*/
      break;
    case WAIT :
      ROS_INFO("Waiting...");
      if (this->timeout.timed_out()) {
        this->timeout.stop();
        this->searching_for_qr = false;
        this->darwin_state = SCAN_MAZE;
      }
      break;
    case WAIT_TRACKING :
      ROS_INFO("Waiting to stop Tracking...");
      if (this->tracking_module.is_finished()) this->darwin_state = SCAN_MAZE;

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
  /* HARDCODED LIMITS */
  double max_upper_limit = 0.015;
  double min_upper_limit = 0.0075;
  double sat = x;

  if (x > max_upper_limit) sat = max_upper_limit;
  else if (x < -max_upper_limit) sat = -max_upper_limit;
  else if (x < min_upper_limit and x > -min_upper_limit and x > 0) sat = min_upper_limit;
  else if (x < min_upper_limit and x > -min_upper_limit and x < 0) sat = -min_upper_limit;

  return sat; //Obviously incomplete
}

void CeabotMazeAlgNode::search_for_goal_qr (void) {
    this->wall_qr_goal_found = false;
    double best_dis = 5.0; //5 metros como primer valor imposible en el laberinto
    if (not this->half_maze_achieved) {
        for (int i = 0; i < this->qr_information.size(); ++i) {
          std::pair <std::string, int> aux = divide_qr_tag (this->qr_information [i].qr_tag);
          double dis_to_goal = distance_to_xy (this->qr_information [i].pos.x, this->qr_information [i].pos.z);
          if (is_goal_wall(&this->qr_information [i]) and aux.first == "N" and dis_to_goal <= best_dis) {
            this->wall_qr_goal_found = true;

            this->next_x_mov = this->qr_information [i].pos.x;
            this->next_z_mov = this->qr_information [i].pos.z + way_zaxis_grow*0.6;

            this->darwin_state = CALCULATE_MOVEMENT;
          }
        }
    }
    else {
        for (int i = 0; i < this->qr_information.size(); ++i) {
          std::pair <std::string, int> aux = divide_qr_tag (this->qr_information [i].qr_tag);
          double dis_to_goal = distance_to_xy (this->qr_information [i].pos.x, this->qr_information [i].pos.z);
          if (is_goal_wall(&this->qr_information [i]) and aux.first == "S" and dis_to_goal <= best_dis) {
            this->wall_qr_goal_found = true;

            this->next_x_mov = this->qr_information [i].pos.x;
            this->next_z_mov = this->qr_information [i].pos.z + way_zaxis_grow*0.6;

            this->darwin_state = CALCULATE_MOVEMENT;
          }
        }
    }

    if (not this->wall_qr_goal_found) {std::cout << "I can't see any Goal QR" << std::endl; this->darwin_state = FIND_HOLES;}

}

bool CeabotMazeAlgNode::distance_sort (qr_info o, qr_info p) {
  double xo, zo, xp, zp;
  xo = o.pos.x; xp = p.pos.x;
  zo = o.pos.z; zp = p.pos.z;

  if (xo > xp) return true;
  else if (xo < xp) return false;
  else {
    if (zo > zp) return true;
    else if (zo < zp) return false;
    else return true;
  }

}

void CeabotMazeAlgNode::find_holes(void) {
  hole best_hole;
  best_hole.first.x = -1.0; best_hole.first.z = -1.0;
  best_hole.second = +9999.0; //Infinite distance

  for (int i = 0; i < qr_information.size(); ++i) {
    qr_info k = qr_information [i];
    double distance;
    if (not is_wall(&k)) { //If it's not a wall...
      qr_info obs1, obs2;
      get_immediate_obs(i, obs1, obs2);
      distance=is_hole(&k, &obs1);
      if ((distance >= 0.7 - ERROR and distance <= 0.8 + ERROR) or obs1.qr_tag == "NULL") {
        DDPOINT h = calculate_point_to_move(&k, &obs1);
        double d_to_darwin = distance_to_xy(h.x, h.z);

        if (d_to_darwin <= best_hole.second) {
          best_hole.first = h;
          best_hole.second = d_to_darwin;
        }
      }

      distance=is_hole(&obs2, &k);
      if ((distance >= 0.7 - ERROR and distance <= 0.8 + ERROR) or obs2.qr_tag == "NULL") {
        DDPOINT h = calculate_point_to_move(&obs2, &k);
        double d_to_darwin = distance_to_xy(h.x, h.z);

        if (d_to_darwin <= best_hole.second) {
          best_hole.first = h;
          best_hole.second = d_to_darwin;
        }
      }
    }
  }

  this->next_x_mov = best_hole.first.x;
  this->next_z_mov = best_hole.first.z;

  this->darwin_state = CALCULATE_MOVEMENT;
}

bool CeabotMazeAlgNode::is_wall(qr_info* obs) {
    std::pair<std::string, int> aux = divide_qr_tag(obs->qr_tag);
    if (aux.second > 6) return true;
    else return false;

}

bool CeabotMazeAlgNode::is_goal_wall(qr_info* obs) {
  std::pair<std::string, int> aux = divide_qr_tag(obs->qr_tag);
  std::cout << "QR_TAG: " << aux.first << std::endl;
  if ((aux.first == "N" or aux.first == "S") and aux.second > 6) return true;
  else return false;

}


double CeabotMazeAlgNode::is_hole(qr_info* obs1, qr_info* obs2) {
    std::cout << obs1->qr_tag << ' ' << obs2->qr_tag << ' ' << (is_goal_wall(obs1) or is_goal_wall(obs2)) << std::endl;
    if (obs1 != NULL and obs2 != NULL and obs1->qr_tag != "NULL" and obs2->qr_tag != "NULL" and not (is_goal_wall(obs1) or is_goal_wall(obs2))) {
      double distance = distance_to_xy(obs1->pos.x - obs2->pos.x, obs1->pos.z - obs2->pos.z);
      bool rtt = false;
      geometry_msgs::PoseStamped transformed_pose;
      geometry_msgs::PoseStamped pose;

      if (is_wall(obs1)) {
        pose = get_PoseStamped(obs2);
        rtt = listener.waitForTransform(pose.header.frame_id, pose.header.frame_id , ros::Time::now(), ros::Duration(0.2), ros::Duration(0.08333));
        if (rtt) listener.transformPose(pose.header.frame_id, pose, transformed_pose);
        //std::cout << "distance: " << distance << " x: " << transformed_pose.pose.position.x << std::endl;
        distance = sqrt(pow(distance, 2) - pow(transformed_pose.pose.position.x, 2));
      }
      else if (is_wall(obs2)) {
        pose = get_PoseStamped(obs1);
        rtt = listener.waitForTransform(pose.header.frame_id, pose.header.frame_id , ros::Time::now(), ros::Duration(0.2), ros::Duration(0.08333));
        if (rtt) listener.transformPose(pose.header.frame_id, pose, transformed_pose);
        std::cout << "distance: " << distance << " x: " << transformed_pose.pose.position.x << std::endl;
        distance = sqrt(pow(distance, 2) - pow(transformed_pose.pose.position.x, 2));
      }
      /*std::cout << obs1->qr_tag << ' ' << obs2->qr_tag << std::endl;
      std::cout << "La distancia de obstaculo a obstaculo es: " << rtt << ' ' << distance << std::endl;
      std::cout << distance << " la condicion se evalua a :" << !(distance >= 0.5 - this->config_.ERROR_PERMES) << std::endl;*/

      return distance;
    }

    return +9999.0; //Es OBJ y NULL O VICEVERSA
}

DDPOINT CeabotMazeAlgNode::calculate_point_to_move(qr_info* obs1, qr_info* obs2) {
    DDPOINT ret;
    if (obs1 != NULL and obs2 != NULL) {
      if (obs1->qr_tag != "NULL" and obs2->qr_tag != "NULL") {
          std::cout << "------------" << std::endl;
          std::cout << obs1->qr_tag << ' ' << obs2->qr_tag << std::endl;
          std::cout << "------------" << std::endl;
          ret.x = ((obs1->pos.x + obs2->pos.x)/2.0);
          ret.z = ((obs1->pos.z + obs2->pos.z + 0.3*way_zaxis_grow)/2.0);

      }
      else if (obs1->qr_tag == "NULL") {
        std::cout << "------------" << std::endl;
        std::cout << "------------ " << obs2->qr_tag << std::endl;
        std::cout << "------------" << std::endl;
        ret.x = obs2->pos.x + 0.5*way_xaxis_grow;
        ret.z = obs2->pos.z + 0.3*way_zaxis_grow;
      }
      else if (obs2->qr_tag == "NULL") {
        std::cout << "------------" << std::endl;
        std::cout << obs1->qr_tag << "------------" << std::endl;
        std::cout << "------------" << std::endl;
        ret.x = obs1->pos.x - 0.5*way_xaxis_grow;
        ret.z = obs1->pos.z + 0.3*way_zaxis_grow;
      }
    }

    std::cout << ret.x << " " << ret.z << std::endl;
    std::cout << "------------------" << std::endl;

    return ret;

}

void CeabotMazeAlgNode::get_immediate_obs (int i, qr_info &obs1, qr_info &obs2) {
  if (i == 0) {
    obs1.qr_tag = "NULL";
  }
  else {
    qr_info aux = qr_information [i - 1];
    obs1 = aux;
  }
  if (i == qr_information.size() - 1) {
    obs2.qr_tag = "NULL";
  }
  else {
    qr_info aux = qr_information [i + 1];
    obs2 = aux;
  }
}

std::pair<std::string, int> CeabotMazeAlgNode::divide_qr_tag (std::string qr_tag) {
    std::pair<std::string, int> aux;

    aux.first = qr_tag[0];
    qr_tag.erase(0, 1);
    aux.second = atoi(qr_tag.c_str());

    return aux;
}

void CeabotMazeAlgNode::fill_PoseStamped (int i, const humanoid_common_msgs::tag_pose_array::ConstPtr &in, geometry_msgs::PoseStamped &out) {
  out.header = in->header;

  out.pose.orientation = in->tags [i].orientation;
  out.pose.position = in->tags [i].position;
}

void CeabotMazeAlgNode::straight_to_north () {
  double diff = this->north_of_the_maze - this->bno055_measurement;
  switch (this->state_stn) {
    case CHECK_NORTH :
      std::cout << "Checking north " << diff << std::endl;
      if (diff >= -ERROR and diff <= +ERROR) {
        ROS_INFO ("Now I'm straight to the North!");
        this->state_stn = STOP_SPINNING;
      }
      else this->walk.set_steps_size(0.0, 0.0, 0.25*saturate_alpha(diff));
      break;
    case STOP_SPINNING :
      ROS_INFO ("Stop Spinning");
      this->walk.stop();
      this->state_stn = CHECK_STOP_SPINNING;

      break;
    case CHECK_STOP_SPINNING :
      ROS_INFO ("Check Stop Spinning");
      if (this->walk.is_finished()) {
        this->darwin_state = SCAN_MAZE;
        this->search_started = false;
      }
      else this->walk.stop();
      break;
  }
}

void CeabotMazeAlgNode::straight_to_south () {
  double diff = this->south_of_the_maze - this->bno055_measurement;
}

double CeabotMazeAlgNode::distance_to_xy (double x, double y) {
  return sqrt(pow(x,2)+pow(y,2));
}

void CeabotMazeAlgNode::process_data(void) {
  std::vector <qr_info> vec_aux;

  for (std::map<std::string,std::vector<qr_info> >::iterator it=this->qr_info_pre_processing.begin(); it!=this->qr_info_pre_processing.end(); ++it) {
    qr_info qr_aux;
    qr_aux.qr_tag = it->first;
    for (int i = 0; i < it->second.size(); ++i) {
      qr_aux.pos.x += it->second [i].pos.x;
      qr_aux.pos.y += it->second [i].pos.y;
      qr_aux.pos.z += it->second [i].pos.z;
      /*qr_aux.ori.x += it->second [i].ori.x;
      qr_aux.ori.y += it->second [i].ori.y;
      qr_aux.ori.z += it->second [i].ori.z;*/
      if (i == it->second.size() - 1) {
        qr_aux.ori.x  = it->second [i].ori.x;
        qr_aux.ori.y  = it->second [i].ori.y;
        qr_aux.ori.z  = it->second [i].ori.z;
        qr_aux.ori.w  = it->second [i].ori.w;
      }
    }

    qr_aux.pos.x /= it->second.size();
    qr_aux.pos.y /= it->second.size();
    qr_aux.pos.z /= it->second.size();


    vec_aux.push_back(qr_aux);
  }

  std::sort(vec_aux.begin(), vec_aux.end(), distance_sort);

  this->qr_information = vec_aux;
  this->darwin_state = SEARCH_FOR_GOAL_QR;

  for (int i = 0; i < qr_information.size(); ++i) {
    std::cout << std::endl;
    std::cout << qr_information[i].qr_tag << std::endl;
    std::cout << "X pos: " << qr_information[i].pos.x << " Y pos: " << qr_information[i].pos.y << " Z pos: " << qr_information[i].pos.z << std::endl;
    std::cout << "---------------------------------" << std::endl;
    std::cout << std::endl;
  }
}

bool CeabotMazeAlgNode::ddpoint_goet (DDPOINT a, DDPOINT b, bool n) {
  if ((a.x >= b.x and a.z >= b.z) and n) return true;
  else if ((a.x <= b.x and a.x <= b.z) and not n) return true;

  return false;
}

geometry_msgs::PoseStamped CeabotMazeAlgNode::get_PoseStamped (qr_info* obs1) {
  geometry_msgs::PoseStamped ret;
  ret.header = obs1->header;
  ret.pose.position.x = obs1->pos.x;
  ret.pose.position.y = obs1->pos.y;
  ret.pose.position.z = obs1->pos.z;
  ret.pose.orientation.x = obs1->ori.x;
  ret.pose.orientation.y = obs1->ori.y;
  ret.pose.orientation.z = obs1->ori.z;
  ret.pose.orientation.w = obs1->ori.w;

  return ret;

}

/* main function */
int main(int argc,char *argv[]) {
  return algorithm_base::main<CeabotMazeAlgNode>(argc, argv, "ceabot_maze_alg_node");
}
