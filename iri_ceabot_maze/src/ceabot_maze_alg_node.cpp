#include "ceabot_maze_alg_node.h"

CeabotMazeAlgNode::CeabotMazeAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<CeabotMazeAlgorithm>(),
  walk("ceabot_maze_walk"),
  tracking_module("ceabot_maze_track"),
  action("action_client"){
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]
  this->event_start = true;
  this->old_darwin_state = IDLE; //To avoid some possible bugs...
  this->darwin_state = SCAN_MAZE;
  this->half_maze_achieved = false;

  this->search_started=false;

  this->direction = 0;
  this->turn_left = 1;
  this->fallen_state = 0;
  this->qr_information = std::vector < std::vector <qr_info> > (5);
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
void CeabotMazeAlgNode::fallen_state_callback(const std_msgs::Int8::ConstPtr& msg) {
  //ROS_INFO("CeabotMazeAlgNode::fallen_state_callback: New Message Received");

  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
  //this->fallen_state_mutex_enter();
  this->fallen_state = msg->data;
  if (msg->data == 0 or msg->data == 1) this->darwin_state = FALLEN_DARWIN; //No creo que lo mejor sea saltar directamente, habria que guardarse el estado anterior o algo por el estilo... (idk)

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
  if (this->fallen_state == 0) {
    this->odom_xpre_fall = msg->pose.pose.position.x;
    this->odom_ypre_fall = msg->pose.pose.position.y;
  }
  else this->darwin_state = FALLEN_DARWIN;

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
  tf::TransformListener listener;
  geometry_msgs::PoseStamped::ConstPtr transformed_pose;
  geometry_msgs::PoseStamped::ConstPtr pose;
  if (msg->tags.size()>0) {
    if (this->searching_for_qr) {
      int zone_to_scan = actual_zone_to_scan();
      std::vector<qr_info> vec_aux;
      for (int i = 0; i < msg->tags.size(); ++i) {
        fill_PoseStamped(i, msg, pose);
        listener.waitForTransform("/base_link", msg->tags[i].header.frame_id , ros::Time::now(), ros::Duration(0.08333), ros::Duration(0.01));
        tf::TransformListener::transformPose("/base_link", pose, transformed_pose);
        //tf::Vector3 in(msg->tags[i].position.z, msg->tags[i].position.x, msg->tags[i].position.y);
        //tf::Vector3 out = t.operator()(in);

        qr_info aux;
        aux.qr_tag = msg->tags[i].tag_id;
        //aux.pos.x = out.x(); aux.pos.y = out.y(); aux.pos.z = out.z();
        aux.pos.x = transformed_pose->pose.position.x; aux.pos.y = transformed_pose->pose.position.y; aux.pos.z = transformed_pose->pose.position.z;
        aux.ori.x = transformed_pose->pose.orientation.x; aux.ori.y = transformed_pose->pose.orientation.y; aux.ori.z = transformed_pose->pose.orientation.z; aux.ori.w = transformed_pose->pose.orientation.w;

        std::cout << std::endl;
        std::cout << "Rotation angle: " << this->current_pan_angle << std::endl;
        std::cout << "Tag ID: " << msg->tags[i].tag_id << std::endl;
        std::cout <<  "Old X pos: " << msg->tags[i].position.x << " Old Y pos: " << msg->tags[i].position.y << " Old Z pos: " <<  msg->tags[i].position.z << std::endl;
        std::cout << "New X pos: " << aux.pos.x << " New Y pos: " << aux.pos.y << " New Z pos: " << aux.pos.z << std::endl;
        std::cout << std::endl;

        vec_aux.push_back(aux);
      }
      std::sort(vec_aux.begin(), vec_aux.end(), distance_sort);
      qr_information [zone_to_scan] = vec_aux;
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

    this->searching_for_qr = false;
    this->darwin_state = WAIT_FOR_SCAN;
    this->old_darwin_state = SCAN_MAZE;
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
        this->searching_for_qr = true;
        ros::Duration(1.0).sleep();
      }
      this->darwin_state = SCAN_MAZE;
    }
    if (this->current_angle_travelled == DegtoRad(360.0)) {
      this->search_started = false;
      this->darwin_state = SEARCH_FOR_GOAL_QR;
      this->direction = 0;

      for (int i = 0; i < qr_information.size(); ++i) {
        std::cout << " Sector i: " << i << std::endl;
        for (int j = 0; j < qr_information[i].size(); ++j) {
          std::cout << std::endl;
          std::cout << qr_information[i][j].qr_tag << std::endl;
          std::cout << "X pos: " << qr_information[i][j].pos.x << " Y pos: " << qr_information[i][j].pos.y << " Z pos: " << qr_information[i][j].pos.z << std::endl;
          std::cout << "---------------------------------" << std::endl;
          std::cout << std::endl;
        }
      }

    }
  }
  this->old_darwin_state = WAIT_FOR_SCAN;
}

void CeabotMazeAlgNode::calculate_next_move(void) {
    this->nm_x = sqrt(pow(this->next_x_mov, 2) + pow(this->next_z_mov, 2));
    this->nm_alpha =  -(PI/2.0 - atan2(this->next_z_mov, this->next_x_mov));

    this->darwin_state = MOVEMENT_ALPHA;
    this->old_darwin_state = CALCULATE_MOVEMENT;

    std::cout << "rad: " << nm_x << " alpha: " << nm_alpha << std::endl;
}

void CeabotMazeAlgNode::darwin_movement_alpha(double alpha) { //angle to perform
    if (alpha < 0) this->turn_left = -1;
    else this->turn_left = 1;

    this->mov_alpha_goal = get_goal_alpha(alpha, this->bno055_measurement);
    double alpha_sat = saturate_alpha(get_magnitude_alpha(this->mov_alpha_goal, this->bno055_measurement));

    this->walk.set_steps_size(0.0, 0.0, this->config_.p * alpha_sat * this->turn_left);

    this->darwin_state = CHECK_GOAL_ALPHA;
    this->old_darwin_state = MOVEMENT_ALPHA;


}

void CeabotMazeAlgNode::darwin_movement_x(double x) {
  std::pair<double, double> parr = get_goal_xy(x, this->odom_x, this->odom_y, this->nm_alpha);
  this->mov_x_goal = parr.first;
  this->mov_y_goal = parr.second;

  double x_sat = saturate_movement(x);
  this->walk.set_steps_size(x_sat, 0.0, 0.0);

  this->darwin_state = CHECK_GOAL_XY;
  this->old_darwin_state = MOVEMENT_X;

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
    this->old_darwin_state = CHECK_GOAL_ALPHA;

  }
}

void CeabotMazeAlgNode::check_goal_xy(double goalx, double goaly) {
  double diffx = fabs(goalx - this->odom_x);
  double diffy = fabs(goaly - this->odom_y);
  std::cout << "Diferència X: " << diffx << " Diferència Y: " << diffy << std::endl;
  if ((diffx >= -this->config_.ERROR_PERMES and diffx <= this->config_.ERROR_PERMES) and (diffy >= -this->config_.ERROR_PERMES and diffy <= this->config_.ERROR_PERMES)) {
    ROS_INFO("XY goal achieved, soon I'll be moving on!");
    this->walk.stop();
    this->darwin_state = FALLEN_DARWIN; //ANADIR QUE SE HAGA LA TRANSICION DE ESTADOS EN LA INTERRUPCION!

  }
  else {
    double tom = saturate_movement(this->config_.p * ((diffx+diffy)/2.0));
    this->walk.set_steps_size(tom, 0.0, 0.0);
  }

    this->darwin_state = CHECK_GOAL_XY;
}

void CeabotMazeAlgNode::state_machine(void) { //Yo pondria fuera de la funcion los cambios de estado que son unicos, o sea, que son transiciones simples...
  switch(this->darwin_state) {
    case IDLE:
      ROS_INFO("Darwin Ceabot Vision : state IDLE");
      break;

    case SCAN_MAZE:
      ROS_INFO("Scanning Maze...");
      scan_maze();
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
      //this->nm_alpha = -0.7;
      //this->nm_x = 0.5;
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
      if (!this->walk.is_finished()) this->walk.stop();
      if (this->action.is_finished()) {
        if (this->fallen_state == 0) {
          this->action.execute(10);
        }
        else if (this->fallen_state == 1) {
          this->action.execute(11);
        }
      }

      this->darwin_state = IS_DARWIN_STANDING;

      break;

    case IS_DARWIN_STANDING:
      ROS_INFO("Waiting for Darwin to be upwards...");
      if (this->action.is_finished()) {
        this->darwin_state = this->old_darwin_state;
        this->old_darwin_state = IS_DARWIN_STANDING;
      }
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
    if (!this->half_maze_achieved) {
        for (int i = 0; i < this->qr_information.size(); ++i) {
            for (int j = 0; j < this->qr_information[i].size(); ++j) {
                std::pair<std::string, int> aux = divide_qr_tag (this->qr_information[i][j].qr_tag);

                if (aux.second > 6 and aux.first == "N") { //First step on the maze
                    this->wall_qr_goal_found = true;

                    this->next_x_mov = this->qr_information[i][j].pos.x;
                    this->next_z_mov = this->qr_information[i][j].pos.z;

                    std::cout << this->next_x_mov << " " << this->next_z_mov << std::endl;
                    this->darwin_state = CALCULATE_MOVEMENT;
                }
            }
        }
    }
    else {
        for (int i = 0; i < this->qr_information.size(); ++i) {
            for (int j = 0; j < this->qr_information[i].size(); ++j) {
                std::pair<std::string, int> aux = divide_qr_tag (this->qr_information[i][j].qr_tag);

                if (aux.second > 6 and aux.first == "S") { //Second and last step on the maze
                    this->wall_qr_goal_found = true;

                    this->next_x_mov = this->qr_information[i][j].pos.x ;
                    this->next_z_mov = this->qr_information[i][j].pos.z;

                    std::cout << this->next_x_mov << " " << this->next_z_mov << std::endl;
                    this->darwin_state = CALCULATE_MOVEMENT;
                }
            }
        }
    } //Incoherente, haga lo que haga se va a calcular la densidad...
    this->darwin_state = CALCULATE_DENSITY;
    this->old_darwin_state = SEARCH_FOR_GOAL_QR;

}

void CeabotMazeAlgNode::calculate_density(void) {
    std::vector<std::pair<int, double> > vec_aux (5);
    double obstacles_on_the_zone;
    for (int i = 0; i < this->qr_information.size(); ++i) {
        obstacles_on_the_zone = 0.0;
        std::vector<bool> obstacles (6, false);
        for (int j = 0; j < this->qr_information[i].size(); ++j) {
            std::pair<std::string, int> actual_qr = divide_qr_tag (this->qr_information[i][j].qr_tag);
            if (actual_qr.second <= 6 and !obstacles[actual_qr.second - 1]) { //Obstacle not already read
                obstacles_on_the_zone += 1.0;
                obstacles[actual_qr.second - 1] = true; //No usas tu funcion, is_wall()??
            }
        }
        vec_aux[i].first = i;
        vec_aux[i].second = obstacles_on_the_zone / 6.0;
    }

    this->ocupation = vec_aux;
    this->darwin_state = FIND_HOLES;
    this->old_darwin_state = CALCULATE_DENSITY;

}

bool CeabotMazeAlgNode::density_sort (std::pair <int,double> k, std::pair <int,double> l) {
    double i = k.second; double j = l.second;
    if (i > 0.0 and j > 0.0) {
        return i < j;
    }
    else if (i == 0.0 and j > 0.0) {
        return false;
    }
    else if (i > 0.0 and j == 0.0) {
        return true;
    }
    else return true;
}

bool CeabotMazeAlgNode::distance_sort (qr_info o, qr_info p) {
  double xo, zo, xp, zp;
  xo = o.pos.x; xp = p.pos.x;
  zo = o.pos.z; zp = p.pos.z;

  if (xo > xp) return false;
  else if (xo < xp) return true;
  else {
    if (zo > zp) return false;
    else if (zo < zp) return true;
    else return true;
  }

}

void CeabotMazeAlgNode::find_holes(void) {
    std::sort (this->ocupation.begin(), this->ocupation.end(), density_sort);
    for (int i = 0; i < this->ocupation.size(); ++i)
    std::cout << this->ocupation [i].first << ' ' << this->ocupation [i].second << std::endl;
    //ocupation is a vector which indicates the sector concerning to the
    //in the first element of the pair and indicates the ocupation coeficient (equal for
    //each obstacle in a sector) in the second element of the pair

    std::pair <int, double> min_density; min_density.second = 1.0;
    int i = 0;
    bool hole_found = false;
    while(i < this->ocupation.size() and not hole_found) {
        if (ocupation [i].second < min_density.second) {min_density = ocupation [i];}
        int sector = this->ocupation [i].first;
        int k = 0;
        while (k < this->qr_information [sector].size() and not hole_found) {
          qr_info m_obs, l_obs, r_obs;
          m_obs = this->qr_information [sector][k];
          if (!is_wall(&m_obs)) {
            get_immediate_obs (sector, k, l_obs, r_obs);
            //Now l_obs and r_obs are fullfilled with the desired obstacles
            //So we can check if there are some wholes between them
            if (l_obs.qr_tag != "NULL" and is_hole(&m_obs, &l_obs)){

              calculate_point_to_move(&m_obs, &l_obs);

              hole_found = true;
            }
            else if (r_obs.qr_tag != "NULL" and is_hole(&m_obs, &r_obs)) {
              //How we choose the best movement?
              //Random? By now we're choosing the first...
              calculate_point_to_move(&m_obs, &r_obs);
              //calculate_point_to_move(m_obs, r_obs);
              hole_found = true;
            }
            else {
              if (l_obs.qr_tag == "NULL") {
                calculate_point_to_move(NULL, &m_obs);

                hole_found = true;
              }
              else if (r_obs.qr_tag == "NULL") {
                calculate_point_to_move(&m_obs, NULL);

                hole_found = true;
              }
              //That means there's no hole, nor left or right
              //So we move towards to the less occupied sector (at the end of both loops)
              //We can avoid that else and if we don't enter the 'if' above
              //Use the 'min_density' pair to calculate the next movement...
            }
          }
          ++k;
        }
        ++i;
    }
    //salto directamente al siguiente estado!
    this->darwin_state = CALCULATE_MOVEMENT;
    this->old_darwin_state = FIND_HOLES;

}

bool CeabotMazeAlgNode::is_wall(qr_info* obs1) {
    std::pair<std::string, int> aux = divide_qr_tag(obs1->qr_tag);
    if (aux.second > 6) return true;
    else return false;
}


bool CeabotMazeAlgNode::is_hole(qr_info* obs1, qr_info* obs2) { //No miras que uno de los dos sea nulo?
    double distance = sqrt(pow(obs1->pos.x - obs2->pos.x, 2) + pow(obs1->pos.z - obs2->pos.z, 2));

    if (is_wall(obs1) or is_wall(obs2)) return false;
    return (distance >= 0.7 - this->config_.ERROR_PERMES and distance >= 0.7 + this->config_.ERROR_PERMES);
}

void CeabotMazeAlgNode::calculate_point_to_move(qr_info* obs1, qr_info* obs2) {
    if (obs1 != NULL and obs2 != NULL) {
        std::cout << "------------" << std::endl;
        std::cout << obs1->qr_tag << ' ' << obs2->qr_tag << std::endl;
        std::cout << "------------" << std::endl;
        this->next_x_mov = (obs1->pos.x + obs2->pos.x) / 2.0;
        this->next_z_mov = (obs1->pos.z + obs2->pos.z) / 2.0;
    }
    else if (obs1 == NULL) {
      std::cout << "------------" << std::endl;
      std::cout << "------------ " << obs2->qr_tag << std::endl;
      std::cout << "------------" << std::endl;
        this->next_x_mov = obs2->pos.x;
        this->next_z_mov = obs2->pos.z;
    }
    else if (obs2 == NULL) {
      std::cout << "------------" << std::endl;
      std::cout << obs1->qr_tag << "------------" << std::endl;
      std::cout << "------------" << std::endl;
        this->next_x_mov = obs1->pos.x;
        this->next_z_mov = obs1->pos.z;
    }
    std::cout << this->next_x_mov << " " << this->next_z_mov << std::endl;

}

void CeabotMazeAlgNode::get_immediate_obs (int m, int i, qr_info &obs1, qr_info &obs2) {
  //Given a sector 'i' and the QR of that sector we want to return via parameters obs1 and obs2
  obs1.qr_tag = obs2.qr_tag = "NULL"; //Before using obs1 and obs2 check the qr_tag...
  //i - 1, i, i + 1
  int l, r, j;
  bool found = false;
  l = r = m;

  if (i == 0) l -= 1;
  if (i == 0 and l >= 0) j = qr_information[l].size() - 1;
  else j = i - 1;

  while (l >= 0 and not found) {
    std::cout << "l : " << l << std::endl;
    while (j >= 0 and not found) {
      std::cout << "j : " << j << std::endl;
      if (qr_information[l][j].qr_tag != qr_information[m][i].qr_tag) {
        obs1 = qr_information[l][j];
        found = true;
      }
      --j;
    }
    --l;
    if (l >= 0 and j < 0) j = qr_information[l].size() - 1;
  }

  found = false;

  if (i == qr_information[m].size() - 1) r += 1;
  if (i == qr_information[m].size() - 1 and r <= qr_information.size() - 1) j = 0;
  else j = i + 1;

  while (r < qr_information.size() and not found) {
    std::cout << "r : " << r << std::endl;
    while (j < qr_information[r].size() and not found) {
      std::cout << "j : " << j << std::endl;
      if (qr_information[r][j].qr_tag != qr_information[m][i].qr_tag) {
        obs2 = qr_information[r][j];
        found = true;
      }
      ++j;
    }
    ++r;
    j = 0;
  }
  std::cout << std::endl;
  std::cout << "QR found:" << std::endl;
  std::cout << "left QR: " << obs1.qr_tag << std::endl;
  std::cout <<  " middle QR: " << this->qr_information[m][i].qr_tag <<  std::endl;
  std::cout << " right QR: " << obs2.qr_tag << std::endl;
  std::cout << std::endl;
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
        return 4;
    }
    else if (this->current_pan_angle > DegtoRad(-67.5) and this->current_pan_angle <= DegtoRad(-22.5)) {
        return 3;
    }
    else if (this->current_pan_angle > DegtoRad(-22.5) and this->current_pan_angle <= DegtoRad(22.5)) {
        return 2;
    }
    else if (this->current_pan_angle > DegtoRad(22.5) and this->current_pan_angle <= DegtoRad(67.5)) {
        return 1;
    }
    else if (this->current_pan_angle > DegtoRad(67.5) and this->current_pan_angle <= DegtoRad(112.5)) {
        return 0;
    }
}

void fill_PoseStamped (int i, onst humanoid_common_msgs::tag_pose_array::ConstPtr &in, geometry_msgs::PoseStamped::ConstPtr &out) {
  out->header.seq = in->header.seq;
  out->header.stamp = in->header.stamp;
  out->header.frame_id = in->header.frame_id;

  out->pose.position = in->tags [i].position;
  out->pose.orientation = in->tags [i].orientation;
}

/* main function */
int main(int argc,char *argv[]) {
  return algorithm_base::main<CeabotMazeAlgNode>(argc, argv, "ceabot_maze_alg_node");
}
