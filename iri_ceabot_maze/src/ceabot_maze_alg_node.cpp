#include "ceabot_maze_alg_node.h"

CeabotMazeAlgNode::CeabotMazeAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<CeabotMazeAlgorithm>(),
  walk("ceabot_maze_walk"),
  tracking_module("ceabot_maze_track"),
  action("action_client"){
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]
  this->event_start = true;
  this->darwin_state = SCAN_MAZE;
  this->state_stn = CHECK_NORTH;
  this->state_sts = CHECK_SOUTH;
  this->first_bno_lecture = true;
  this->first_lecture = true;
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
  this->goalqr_vecpos = 0;
  this->goalqr_x = 0.0;
  this->goalqr_z = 0.0;
  this->str_forward_dis = 0.0;
  this->get_wall_qr = true;

  // [init publishers]

  // [init subscribers]
  this->buttons_subscriber_ = this->public_node_handle_.subscribe("buttons", 1, &CeabotMazeAlgNode::buttons_callback, this);
  pthread_mutex_init(&this->buttons_mutex_,NULL);

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
  set_qr_size_client_ = this->public_node_handle_.serviceClient<humanoid_common_msgs::set_qr_size>("set_qr_size");


  // [init action servers]

  // [init action clients]
}

CeabotMazeAlgNode::~CeabotMazeAlgNode(void){
  // [free dynamic memory]
  pthread_mutex_destroy(&this->buttons_mutex_);
  pthread_mutex_destroy(&this->fallen_state_mutex_);
  pthread_mutex_destroy(&this->imu_mutex_);
  pthread_mutex_destroy(&this->odom_mutex_);
  pthread_mutex_destroy(&this->joint_states_mutex_);
  pthread_mutex_destroy(&this->qr_pose_mutex_);
}

void CeabotMazeAlgNode::mainNodeThread(void){
  // [fill msg structures]

  // [fill srv structure and make request to the server]
  //set_qr_size_srv_.request.data = my_var;
  //ROS_INFO("CeabotMazeAlgNode:: Sending New Request!");
  //if (set_qr_size_client_.call(set_qr_size_srv_))
  //{
    //ROS_INFO("CeabotMazeAlgNode:: Response: %s", set_qr_size_srv_.response.result);
  //}
  //else
  //{
    //ROS_INFO("CeabotMazeAlgNode:: Failed to Call Server on topic set_qr_size ");
  //}



  // [fill action structure and make request to the action server]    init_walk_module();

  // [publish messages]

  if (not this->event_start) {

    //We must set down the start flag in terms of dont enter to this condition again

    state_machine();

    if (this->half_maze_achieved) {this->way_zaxis_grow = 1.0; this->way_xaxis_grow = 1.0;}
    else {this->way_zaxis_grow = -1.0; this->way_xaxis_grow = -1.0;}

    /*std::cout << "searching for qr: " << this->searching_for_qr << std::endl;
    std::cout << "half_maze_achieved == " << half_maze_achieved << std::endl;
    std::cout << "way_xaxis_grow : " << this->way_xaxis_grow << std::endl;
    std::cout << "way_zaxis_grow : " << this->way_zaxis_grow << std::endl;*/
  }
  else {
    init_walk_module();
    init_headt_module();
  }

  //Code for the start. It sets all the configuration from the .cfg to the modules.
  //Then, waits for 5 sec (start condition - read CEABOT rules) and change the Darwin
  //state to start searching for the first QR code

}

/*  [subscriber callbacks] */
void CeabotMazeAlgNode::buttons_callback(const humanoid_common_msgs::buttons::ConstPtr& msg)
{
  for (int i = 0; i < msg->name.size(); ++i) {
    if (msg->name[i] == "start" && msg->state[i] == true) {
  	this->darwin_state = SCAN_MAZE;
  	this->event_start = false;
    ros::Duration(5.0).sleep();
    }
  }
  //ROS_INFO("CeabotMazeAlgNode::buttons_callback: New Message Received");
}

void CeabotMazeAlgNode::fallen_state_callback(const std_msgs::Int8::ConstPtr& msg) {
  //ROS_INFO("CeabotMazeAlgNode::fallen_state_callback: New Message Received");

  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
  //this->fallen_state_mutex_enter();
  this->fallen_state = msg->data;
  /*if (msg->data == 0 or msg->data == 1) {
    this->darwin_state = FALLEN_DARWIN; //No creo que lo mejor sea saltar directamente, habria que guardarse el estado anterior o algo por el estilo... (idk)
    std::cout << "Fallen detectado con el fallen state callback" << std::endl;
  }*/

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
    this->south_of_the_maze = bnoaux - PI;
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

  //std::cout << this->times_timedout << " timedout.." << std::endl;
  if (msg->tags.size()>0) {
    if (this->searching_for_qr) {
      std::cout << "FIRST LECTURE: " << this->first_lecture << std::endl;
      if (not this->first_lecture) {
        for (int i = 0; i < msg->tags.size(); ++i) {
            bool ready_to_transform = false;

            qr_info aux;
            aux.header = msg->tags[i].header;
            aux.qr_tag = msg->tags[i].tag_id;

            if ((get_wall_qr and is_wall(&aux)) or (not get_wall_qr and not is_wall(&aux))) {
              geometry_msgs::PoseStamped transformed_pose;
              geometry_msgs::PoseStamped pose;

              fill_PoseStamped(msg->tags[i], pose);
              //std::cout << msg->tags[i].header.frame_id << std::endl;
              this->qr_pose_mutex_exit();
              ready_to_transform = listener.waitForTransform("darwin/base_link", msg->tags[i].header.frame_id , msg->tags[i].header.stamp, ros::Duration(0.3), ros::Duration(0.08333));
              this->qr_pose_mutex_enter();
              //std::cout << "ready_to_transform??: " << ready_to_transform << std::endl;
              if (ready_to_transform) {

                listener.transformPose("darwin/base_link", pose, transformed_pose);

                aux.pos.x = transformed_pose.pose.position.x; aux.pos.y = transformed_pose.pose.position.y; aux.pos.z = transformed_pose.pose.position.z;
                aux.ori.x = transformed_pose.pose.orientation.x; aux.ori.y = transformed_pose.pose.orientation.y; aux.ori.z = transformed_pose.pose.orientation.z; aux.ori.w = transformed_pose.pose.orientation.w;

                std::cout << std::endl;
                std::cout << "Tag ID: " << msg->tags[i].tag_id << std::endl;
                std::cout << "Frame_id: " << msg->tags[i].header.frame_id << std::endl;
                std::cout <<  "Old X pos: " << msg->tags[i].position.x << " Old Y pos: " << msg->tags[i].position.y << " Old Z pos: " <<  msg->tags[i].position.z << std::endl;
                std::cout << "New X pos: " << aux.pos.x << " New Y pos: " << aux.pos.y << " New Z pos: " << aux.pos.z << std::endl;
                std::cout << std::endl;

                this->qr_info_pre_processing[aux.qr_tag].push_back(aux);

              }
            }
        }
        //std::sort(vec_aux.begin(), vec_aux.end(), distance_sort);
      }
      else this->first_lecture = false;
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
      this->qr_info_pre_processing.clear();
      this->qr_info_pre_processing = std::map < std::string, std::vector <qr_info> >();
      this->direction = 1;
      this->goal_x = PI/2.0;
      this->goal_y = PI/4.0;
      this->current_angle_travelled = 0.0;
      this->tracking_module.start_tracking(goal_x, goal_y);
      this->search_started = true;
      ROS_INFO ("WOLOLOOOO");
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
        this->goal_x -= PI/6.0;
        this->current_angle_travelled += PI/6.0;
      }
      if (this->direction == -1 or (aux_pan >= (-PI/2.0 - ERROR) and aux_pan <= (-PI/2.0 + ERROR))) {
        //std::cout << "wait for scan entro" << std::endl;
        this->darwin_state = START_WAITING1;
      }
      if (this->darwin_state != START_WAITING1) this->darwin_state = SCAN_MAZE;
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

    std::cout << "rad: " << this->nm_x << " alpha: " << this->nm_alpha << std::endl;
}

void CeabotMazeAlgNode::darwin_movement_alpha(double alpha) { //angle to perform
    if (alpha < 0 and not this->half_maze_achieved) this->turn_left = -1;
    else if (alpha > 0 and not this->half_maze_achieved) this->turn_left = +1;
    else if (alpha < 0 and this->half_maze_achieved) this->turn_left = +1;
    else if (alpha > 0 and this->half_maze_achieved) this->turn_left = -1;

    this->mov_alpha_goal = get_goal_alpha(alpha, this->bno055_measurement);
    double alpha_sat = saturate_alpha(get_magnitude_alpha(this->mov_alpha_goal, this->bno055_measurement));

    if (alpha >= -this->config_.ERROR_PERMES and alpha <= +this->config_.ERROR_PERMES) this->darwin_state = MOVEMENT_X;
    else {this->walk.set_steps_size(0.0, 0.0, this->config_.p * alpha_sat * this->turn_left); this->darwin_state = CHECK_GOAL_ALPHA;}

}

void CeabotMazeAlgNode::darwin_movement_x(double x) {
  std::pair<double, double> parr = get_goal_xy(x, this->odom_x, this->odom_y, this->nm_alpha);
  this->mov_x_goal = parr.first;
  this->mov_y_goal = parr.second;
  this->ini_x = this->odom_x;
  this->ini_z = this->odom_y;
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
  //double diffx = fabs(goalx - this->odom_x);
  //double diffy = fabs(goaly - this->odom_y);
  double distance_to_ini = distance_to_xy(this->ini_x - this->odom_x, this->ini_z - this->odom_y);
  double diff = this->nm_x - distance_to_ini;
  std::cout << "Diferencia de distancia: " << diff << std::endl;
  //std::cout << "Diferència X: " << diffx << " Diferència Y: " << diffy << std::endl;
  //bool x_achieved = diffx >= -this->config_.ERROR_PERMES and diffx <= this->config_.ERROR_PERMES;
  //bool y_achieved = diffy >= -this->config_.ERROR_PERMES and diffy <= this->config_.ERROR_PERMES;
  if (diff >= -this->config_.ERROR_PERMES and diff <= +this->config_.ERROR_PERMES) {
    ROS_INFO("XY goal achieved, soon I'll be moving on!");
    this->walk.stop();
    this->darwin_state = IDLE;
    //double dis_to_qr = distance_to_xy(this->next_x_mov - this->odom_x, this->next_z_mov - this->odom_y); //Es diff...
    /*if (this->wall_qr_goal_found and not this->half_maze_achieved and diff >= 0.4 - this->config_.ERROR_PERMES and diff <= 0.4 + this->config_.ERROR_PERMES) {
      this->wall_qr_goal_found = false;
      this->half_maze_achieved = true;
    }
    else if (this->wall_qr_goal_found and this->half_maze_achieved and diff >= 0.4 - this->config_.ERROR_PERMES and diff <= 0.4 + this->config_.ERROR_PERMES) {
      std::cout << "Maze ended, we must check that's true" << std::endl;
    }*/
  }
  else {
    double tom = saturate_movement(this->config_.p * diff);
    this->walk.set_steps_size(tom, 0.0, 0.0);
  }
}

void CeabotMazeAlgNode::state_machine(void) { //Yo pondria fuera de la funcion los cambios de estado que son unicos, o sea, que son transiciones simples...
  switch(this->darwin_state) {
    case IDLE:
      ROS_INFO("Darwin Ceabot Maze : state IDLE");
    /*if (half_maze_achieved) {this->darwin_state = TURN_TO_SOUTH; this->state_sts = CHECK_SOUTH;}
      else {this->darwin_state = TURN_TO_NORTH; this->state_stn = CHECK_NORTH;}*/
      if (not this->half_maze_achieved and straight_to_north()) this->darwin_state = SCAN_MAZE;
      else if (this->half_maze_achieved and straight_to_south()) this->darwin_state = SCAN_MAZE;

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
      if (not this->walk.is_finished()) this->walk.stop();
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
        if (not this->half_maze_achieved and straight_to_north()) this->darwin_state = SCAN_MAZE;
        else if (this->half_maze_achieved and straight_to_south()) this->darwin_state = SCAN_MAZE;
      }
      break;

    case START_WAITING1 :
      ROS_INFO("Start Waiting1...");
      this->searching_for_qr = true;
      this->first_lecture = true;
      this->get_wall_qr = true;
      //set_qr_size_srv_.request.qr_x = 0.172;
      //set_qr_size_srv_.request.qr_y = 0.172;
      //set_qr_size_client_.call(set_qr_size_srv_);

      this->timeout1.start(ros::Duration(1.0));
      this->darwin_state = WAIT1;

      break;

    case WAIT1 :
      ROS_INFO("Waiting1...");
      if (this->timeout1.timed_out()) {
        this->timeout1.stop();
        this->darwin_state = START_WAITING2;
      }
      break;
    case START_WAITING2 :
      ROS_INFO("Start Waiting2...");
      this->searching_for_qr = true;
      this->first_lecture = true;
      this->get_wall_qr = false;
      //set_qr_size_srv_.request.qr_x = 0.077;
      //set_qr_size_srv_.request.qr_y = 0.077;
     // set_qr_size_client_.call(set_qr_size_srv_);

      this->timeout2.start(ros::Duration(1.0));
      this->darwin_state = WAIT2;

      break;
    case WAIT2 :
      ROS_INFO("Waiting2...");
      if (this->timeout2.timed_out()) {
        this->timeout2.stop();
        this->searching_for_qr = false;
        this->darwin_state = SCAN_MAZE;
      }
      break;
    case WAIT_TRACKING :
      ROS_INFO("Waiting to stop Tracking...");
      if (this->tracking_module.is_finished()) this->darwin_state = SCAN_MAZE;

      break;

    case TURN_TO_NORTH :
      if (straight_to_north()) this->darwin_state = SCAN_MAZE;

      break;

    case TURN_TO_SOUTH :
      if (straight_to_south()) this->darwin_state = SCAN_MAZE;
      break;

    case CALCULATE_MOVEMENT_FOR_GOAL_QR :
      ROS_INFO ("Calculating Movement for goal qr...");
      calculate_movement_for_goal_qr();
      break;

    case MOVEMENT_ALPHA_FOR_GOAL_QR :
      ROS_INFO ("Alpha movement!");
      movement_alpha_for_goal_qr();
      break;

    case CHECK_ALPHA_GOAL_MFGQR :
      ROS_INFO ("Check Alpha!");
      check_alpha_goal_mfgqr();
      break;

    case MOVEMENT_X_FOR_GOAL_QR :
      ROS_INFO ("Movement X for goal QR...");
      movement_x_for_goal_qr();
      break;

    case CHECK_GOAL_MFGQR :
      ROS_INFO ("Check goal MFGQR...");
      check_goal_mfgqr();
      break;

    case STRAIGHT_FORWARD :
      ROS_INFO ("Straight Forward...");
      straight_forward();
      break;
    case CHECK_STRAIGHT_FORWARD :
      ROS_INFO ("Checking straight forward...");
      check_straight_forward();
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
  double max_upper_limit = 0.02;
  double min_upper_limit = 0.075;
  double sat = x;

  if (x > max_upper_limit) sat = max_upper_limit;
  else if (x < -max_upper_limit) sat = -max_upper_limit;
  else if (x < min_upper_limit and x > -min_upper_limit and x > 0) sat = min_upper_limit;
  else if (x < min_upper_limit and x > -min_upper_limit and x < 0) sat = -min_upper_limit;

  return sat; //Obviously incomplete
}

void CeabotMazeAlgNode::search_for_goal_qr (void) {
  std::string goal;
    this->wall_qr_goal_found = false;
    double best_dis = 5.0; //5 metros como primer valor imposible en el laberinto
    if (not this->half_maze_achieved) {
        for (int i = 0; i < this->qr_information.size(); ++i) {
          std::pair <std::string, int> aux = divide_qr_tag (this->qr_information [i].qr_tag);
          double dis_to_goal = distance_to_xy (this->qr_information [i].pos.x, this->qr_information [i].pos.z);
          if (is_goal_wall(&this->qr_information [i]) and aux.first == "N" and dis_to_goal <= best_dis) {
            this->wall_qr_goal_found = true;
            this->goalqr_vecpos = i;
            this->goalqr_x = this->qr_information [i].pos.x;
            this->goalqr_z = this->qr_information [i].pos.z;
            best_dis = dis_to_goal;
            this->darwin_state = CALCULATE_MOVEMENT_FOR_GOAL_QR;
          }
        }
    }
    else {
        for (int i = 0; i < this->qr_information.size(); ++i) {
          std::pair <std::string, int> aux = divide_qr_tag (this->qr_information [i].qr_tag);
          double dis_to_goal = distance_to_xy (this->qr_information [i].pos.x, this->qr_information [i].pos.z);
          if (is_goal_wall(&this->qr_information [i]) and aux.first == "S" and dis_to_goal <= best_dis) {
            this->wall_qr_goal_found = true;
            this->goalqr_vecpos = i;
            this->goalqr_x = this->qr_information [i].pos.x;
            this->goalqr_z = this->qr_information [i].pos.z;
            best_dis = dis_to_goal;
            this->darwin_state = CALCULATE_MOVEMENT_FOR_GOAL_QR;
          }
        }
    }
    //Afegir casuistica corresponent a la col·lisio amb altres obstacles pel cami
    if (not this->wall_qr_goal_found) {std::cout << "I can't see any Goal QR" << std::endl; this->darwin_state = FIND_HOLES;}
    else std::cout << "I found a Goal QR: " << goal << std::endl;

}

bool CeabotMazeAlgNode::distance_sort (qr_info o, qr_info p) {
  double xo, xp;
  xo = o.pos.x; xp = p.pos.x;

  if (xo > xp) return true;
  else if (xo < xp) return false;

  return true;

}

void CeabotMazeAlgNode::find_holes(void) {
  hole best_hole, bestwall_hole;
  best_hole.first.x = bestwall_hole.first.x = -1.0; best_hole.first.z = bestwall_hole.first.z = -1.0;
  best_hole.second = bestwall_hole.second = +9999.0; //Infinite distance


  for (int i = 0; i < qr_information.size(); ++i) {
    qr_info k = qr_information [i];
    double distance;
    if (not is_wall(&k)) { //If it's not a wall...
      qr_info obs1, obs2;
      get_immediate_obs(i, obs1, obs2);
      distance=is_hole(&k, &obs1);
      std::cout << k.qr_tag << ' ' << obs1.qr_tag << std::endl;
      std::cout << "Dis between: " << distance << std::endl;
      if ((distance >= 0.75 - ERROR and distance <= 0.75 + ERROR) or obs1.qr_tag == "NULL") {
        DDPOINT h = calculate_point_to_move(&k, &obs1);
        double d_to_darwin = distance_to_xy(h.x, h.z);

        if (d_to_darwin < best_hole.second) {
          best_hole.first = h;
          best_hole.second = d_to_darwin;
        }
      }

      distance=is_hole(&obs2, &k);
      std::cout << obs2.qr_tag << ' ' << k.qr_tag << std::endl;
      std::cout << "Dis between: " << distance << std::endl;
      if ((distance >= 0.75 - ERROR and distance <= 0.75 + ERROR) or obs2.qr_tag == "NULL") {
        DDPOINT h = calculate_point_to_move(&obs2, &k);
        double d_to_darwin = distance_to_xy(h.x, h.z);

        if (d_to_darwin < best_hole.second) {
          best_hole.first = h;
          best_hole.second = d_to_darwin;
        }
      }
    }
    else if (best_hole.first.x == -1.0 and best_hole.first.z == -1.0) { //No holes found..
      double d_to_darwin = distance_to_xy(bestwall_hole.first.x, bestwall_hole.first.z);

      if (d_to_darwin < bestwall_hole.second) {
        bestwall_hole.second = d_to_darwin;
        bestwall_hole.first.x = k.pos.x;
        bestwall_hole.first.z = k.pos.z*0.65;
      }
    }
  }
  if (best_hole.first.x != -1.0 and best_hole.first.z != -1.0) {
    this->next_x_mov = best_hole.first.x;
    this->next_z_mov = best_hole.first.z;
  }
  else if (bestwall_hole.first.x != -1.0 and bestwall_hole.first.z != 1.0) {
    this->next_x_mov = bestwall_hole.first.x;
    this->next_z_mov = bestwall_hole.first.z;
  }
  else {
    ROS_INFO("No Hole found...");
    /*this->next_x_mov = this->odom_x;
    this->next_z_mov = this->odom_y + way_zaxis_grow*0.4; //Go backwards to take a better position for qr_detector*/
  }

  this->darwin_state = CALCULATE_MOVEMENT;
}

bool CeabotMazeAlgNode::is_wall(qr_info* obs) {
    std::pair<std::string, int> aux = divide_qr_tag(obs->qr_tag);
    if (aux.second > 6) return true;
    else return false;

}

bool CeabotMazeAlgNode::is_goal_wall(qr_info* obs) {
  std::pair<std::string, int> aux = divide_qr_tag(obs->qr_tag);
  if ((aux.first == "N" or aux.first == "S") and aux.second > 6) return true;
  else return false;

}


double CeabotMazeAlgNode::is_hole(qr_info* obs1, qr_info* obs2) {
    if (obs1 != NULL and obs2 != NULL and obs1->qr_tag != "NULL" and obs2->qr_tag != "NULL" and not (is_goal_wall(obs1) or is_goal_wall(obs2))) {

      if (is_wall(obs1) or is_wall(obs2)) {
        return fabs(obs1->pos.x - obs2->pos.x);
      }

      double distance = distance_to_xy(obs1->pos.x - obs2->pos.x, obs1->pos.z - obs2->pos.z);
      return distance;
    }

    return +9999.0; //Es OBJ y NULL O VICEVERSA
}

DDPOINT CeabotMazeAlgNode::calculate_point_to_move(qr_info* obs1, qr_info* obs2) {
    DDPOINT ret;
    if (obs1 != NULL and obs2 != NULL) {
      if (obs1->qr_tag != "NULL" and obs2->qr_tag != "NULL") {
        double min_z = 0.0;
        if (obs1->pos.z > obs2->pos.z) min_z = obs2->pos.z;
        else min_z = obs1->pos.z;

        if (not is_wall(obs1) and not is_wall(obs2)) {
          if (NS_orientation(*obs1) and not NS_orientation(*obs2)) {
            ret.x = (obs1->pos.x + obs2->pos.x + 0.15*this->way_xaxis_grow)/2.0;
            ret.z = min_z;
          }
          else if (not NS_orientation(*obs1) and NS_orientation(*obs2)) {
            ret.x = (obs1->pos.x + obs2->pos.x - 0.15*this->way_xaxis_grow)/2.0;
            ret.z = min_z;
          }
          else {
            ret.x = (obs1->pos.x + obs2->pos.x)/2.0;
            ret.z = min_z; //Cas que recull tant el no son NS o el que son NS
          }
        }

        else if (not is_wall(obs1) and is_wall(obs2)) {
          if (NS_orientation(*obs1)) {
            ret.x = (obs1->pos.x + obs2->pos.x + 0.15*this->way_xaxis_grow)/2.0;
            ret.z = min_z;
          }
          else {
            ret.x = (obs1->pos.x + obs2->pos.x)/2.0;
            ret.z = min_z;
          }
        }

        else if (is_wall(obs1) and not is_wall(obs2)) {
          if (NS_orientation(*obs2)) {
            ret.x = (obs1->pos.x + obs2->pos.x - 0.15*this->way_xaxis_grow)/2.0;
            ret.z = min_z;
          }
          else {
            ret.x = (obs1->pos.x + obs2->pos.x)/2.0;
            ret.z = min_z;
          }
        }

        else if (is_wall(obs1) and is_wall(obs2)) {
          ret.x = (obs1->pos.x + obs2->pos.x)/2.0;
          ret.z = min_z;
        }

      }
      else if (obs1->qr_tag == "NULL" and obs2->qr_tag != "NULL") {
        if (not is_wall(obs2) and NS_orientation(*obs2)) {
          ret.x = obs2->pos.x + 0.25*this->way_xaxis_grow;
        }
        else ret.x = obs2->pos.x + 0.15*this->way_xaxis_grow;
        ret.z = obs2->pos.z;
      }
      else if (obs1->qr_tag != "NULL" and obs2->qr_tag == "NULL") {
        if (not is_wall(obs1) and NS_orientation(*obs1)) {
          ret.x = obs1->pos.x - 0.25*this->way_xaxis_grow;
        }
        else ret.x = obs1->pos.x - 0.15*this->way_xaxis_grow;
        ret.z = obs1->pos.z;
      }
    }

    /*std::cout << "------------------" << std::endl;
    std::cout << ret.x << " " << ret.z << std::endl;
    std::cout << "------------------" << std::endl;*/
    std::cout << "Distance to: " << distance_to_xy(ret.x, ret.z);
    std::cout << "------------------" << std::endl;

    return ret;

}

void CeabotMazeAlgNode::get_immediate_obs (int i, qr_info &obs1, qr_info &obs2) { //or (is_goal_wall(&this->qr_information [i]) and is_goal_wall(&aux)) or (is_goal_wall(&this->qr_information [i]) and is_goal_wall(&aux)
  if (i == 0) {
    obs1.qr_tag = "NULL";
  }
  else {
    qr_info aux = this->qr_information [i - 1];
    double dis = distance_to_xy(this->qr_information [i].pos.x - aux.pos.x, this->qr_information [i].pos.z - aux.pos.z);
    if (is_wall(&aux) and wall_too_far(aux, this->qr_information [i]) or (is_goal_wall(&this->qr_information [i]) and dis > 1.5 + this->config_.ERROR_PERMES)) {
      obs1.qr_tag = "NULL";
    }
    else obs1 = aux;
  }
  if (i == qr_information.size() - 1) {
    obs2.qr_tag = "NULL";
  }
  else {
    qr_info aux = this->qr_information [i + 1];
    double dis = distance_to_xy(this->qr_information [i].pos.x - aux.pos.x, this->qr_information [i].pos.z - aux.pos.z);
    if ((is_wall(&aux) and wall_too_far(aux, this->qr_information [i])) or (is_goal_wall(&this->qr_information [i]) and dis > 1.5 + this->config_.ERROR_PERMES)) {
      obs2.qr_tag = "NULL";
    }
    else obs2 = aux;
  }
  if (is_goal_wall(&this->qr_information [i])) { //Debugging
    std::cout << obs1.qr_tag << ' ' << this->qr_information [i].qr_tag << ' ' << obs2.qr_tag << std::endl;
  }
}

std::pair<std::string, int> CeabotMazeAlgNode::divide_qr_tag (std::string qr_tag) {
    std::pair<std::string, int> aux;

    aux.first = qr_tag[0];
    qr_tag.erase(0, 1);
    aux.second = atoi(qr_tag.c_str());

    return aux;
}

void CeabotMazeAlgNode::fill_PoseStamped (const humanoid_common_msgs::tag_pose &in, geometry_msgs::PoseStamped &out) {
  out.header = in.header;

  out.pose.orientation = in.orientation;
  out.pose.position = in.position;
}

bool CeabotMazeAlgNode::straight_to_north () {
  double diff = this->north_of_the_maze - this->bno055_measurement;
  switch (this->state_stn) {
    case CHECK_NORTH :
      //std::cout << "Checking north " << diff << std::endl;
      if (diff >= -ERROR and diff <= +ERROR) {
        ROS_INFO ("Now I'm straight to the North!");
        this->state_stn = STOP_SPINNING_NORTH;
      }
      else this->walk.set_steps_size(0.0, 0.0, 0.25*saturate_alpha(diff));
      break;
    case STOP_SPINNING_NORTH :
      ROS_INFO ("Stop Spinning North...");
      this->walk.stop();
      this->state_stn = CHECK_STOP_SPINNING_NORTH;

      break;
    case CHECK_STOP_SPINNING_NORTH :
      ROS_INFO ("Check Stop Spinning North...");
      if (this->walk.is_finished()) {
        this->search_started = false;
        return true;
      }
      else this->walk.stop();
      break;
  }

  return false;
}

bool CeabotMazeAlgNode::straight_to_south () {
  double diff = this->south_of_the_maze - this->bno055_measurement;
  switch (this->state_sts) {
    case CHECK_SOUTH :
      std::cout << "Checking south " << diff << std::endl;
      if (diff >= -ERROR and diff <= +ERROR) {
        ROS_INFO ("Now I'm straight to the North!");
        this->state_sts = STOP_SPINNING_SOUTH;
      }
      else this->walk.set_steps_size(0.0, 0.0, 0.25*saturate_alpha(diff));
      break;
    case STOP_SPINNING_SOUTH :
      ROS_INFO ("Stop Spinning South...");
      this->walk.stop();
      this->state_sts = CHECK_STOP_SPINNING_SOUTH;

      break;
    case CHECK_STOP_SPINNING_SOUTH :
      ROS_INFO ("Check Stop Spinning South...");
      if (this->walk.is_finished()) {
        this->search_started = false;
        return true;
      }
      else this->walk.stop();
      break;

  }

  return false;
}

double CeabotMazeAlgNode::distance_to_xy (double x, double y) {
  return sqrt(pow(x,2)+pow(y,2));
}

void CeabotMazeAlgNode::process_data(void) {
  std::vector <qr_info> vec_aux;

  for (std::map<std::string,std::vector<qr_info> >::iterator it=this->qr_info_pre_processing.begin(); it!=this->qr_info_pre_processing.end(); ++it) {
    qr_info qr_aux;
    qr_aux.pos.x = 0.0;
    qr_aux.pos.y = 0.0;
    qr_aux.pos.z = 0.0;
    qr_aux.ori.x = 0.0;
    qr_aux.ori.y = 0.0;
    qr_aux.ori.z = 0.0;
    qr_aux.ori.w = 0.0;
    qr_aux.qr_tag = it->first;
    for (int i = 0; i < it->second.size(); ++i) {
      qr_aux.pos.x += it->second [i].pos.x;
      qr_aux.pos.y += it->second [i].pos.y;
      qr_aux.pos.z += it->second [i].pos.z;

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

bool CeabotMazeAlgNode::wall_too_far (qr_info wall, qr_info obs) { //To prevent picking too far walls...
  double x_wall, x_obs, z_wall, z_obs, dis;
  x_wall = wall.pos.x;
  x_obs  = obs.pos.x;
  z_wall = wall.pos.z;
  z_obs  = obs.pos.z;
  dis = distance_to_xy(x_obs - x_wall, z_obs - z_wall);
  std::cout << wall.qr_tag << ' ' << obs.qr_tag << std::endl;
  std::cout << "Wall too far??: " << dis << std::endl;
  if (not is_goal_wall (&wall) and dis >= 0.75 + this->config_.ERROR_PERMES) return true;
  else if (is_goal_wall (&wall) and dis >= 1.5 + this->config_.ERROR_PERMES) return true;

  return false;
}

void CeabotMazeAlgNode::calculate_movement_for_goal_qr () {
  qr_info obs1, obs2;
  get_immediate_obs(this->goalqr_vecpos, obs1, obs2);
  this->darwin_state = MOVEMENT_ALPHA_FOR_GOAL_QR;
  std::cout << obs1.qr_tag << ' ' << obs2.qr_tag << std::endl;

  if (obs1.qr_tag != "NULL" and obs2.qr_tag != "NULL") {
    double min_z = 0.0;
    if (obs1.pos.z > obs2.pos.z) min_z = obs2.pos.z;
    else min_z = obs1.pos.z;
    //min_z += way_zaxis_grow*0.15;

    if (not is_wall(&obs1) and not is_wall(&obs2)) {
      if (NS_orientation(obs1) and not NS_orientation(obs2)) {
        this->next_x_mov = (obs1.pos.x + obs2.pos.x + 0.15*this->way_xaxis_grow)/2.0;
        this->next_z_mov = min_z;
      }
      else if (not NS_orientation(obs1) and NS_orientation(obs2)) {
        this->next_x_mov = (obs1.pos.x + obs2.pos.x - 0.15*this->way_xaxis_grow)/2.0;
        this->next_z_mov = min_z;
      }
      else {
        this->next_x_mov = (obs1.pos.x + obs2.pos.x)/2.0;
        this->next_z_mov = min_z; //Cas que recull tant el no son NS o el que son NS
      }
    }
    else if (not is_wall(&obs1) and is_wall(&obs2)) {
      if (NS_orientation(obs1)) {
        this->next_x_mov = (obs1.pos.x + obs2.pos.x + 0.15*this->way_xaxis_grow)/2.0;
        this->next_z_mov = min_z;
      }
      else {
        this->next_x_mov = (obs1.pos.x + obs2.pos.x)/2.0;
        this->next_z_mov = min_z;
      }
    }
    else if (is_wall(&obs1) and not is_wall(&obs2)) {
      if (NS_orientation(obs2)) {
        this->next_x_mov = (obs1.pos.x + obs2.pos.x - 0.15*this->way_xaxis_grow)/2.0;
        this->next_z_mov = min_z;
      }
      else {
        this->next_x_mov = (obs1.pos.x + obs2.pos.x)/2.0;
        this->next_z_mov = min_z;
      }
    }
    else if (is_wall(&obs1) and is_wall(&obs2)) {
        this->next_x_mov = (obs1.pos.x + obs2.pos.x)/2.0;
        this->next_z_mov = min_z;
    }

    this->str_forward_dis = fabs(this->goalqr_z - this->next_z_mov);
  }
  else if (obs1.qr_tag != "NULL" and obs2.qr_tag == "NULL") {
    if (not is_wall(&obs1) and NS_orientation(obs1)) {
      this->next_x_mov = this->goalqr_x - 0.15*this->way_xaxis_grow;
    }
    else this->next_x_mov = this->goalqr_x;
    this->next_z_mov = obs1.pos.z;

    this->str_forward_dis = fabs(this->goalqr_z - this->next_z_mov);
  }
  else if (obs1.qr_tag == "NULL" and obs2.qr_tag != "NULL") {
    if (not is_wall(&obs2) and NS_orientation(obs2)) {
      this->next_x_mov = this->goalqr_x + 0.15*this->way_xaxis_grow;
    }
    else this->next_x_mov = this->goalqr_x;
    this->next_z_mov = obs2.pos.z;

    this->str_forward_dis = fabs(this->goalqr_z - this->next_z_mov);
  }
  else {
    this->next_x_mov = this->goalqr_x;
    this->next_z_mov = this->goalqr_z + 0.15*this->way_zaxis_grow;

    this->str_forward_dis = -1;
    this->darwin_state = STRAIGHT_FORWARD;
  }

  if (is_goal_wall(&obs1) and is_goal_wall(&obs2)) {
    this->next_x_mov = this->goalqr_x;
    this->next_z_mov = this->goalqr_z + 0.4*this->way_zaxis_grow;
    std::cout << "El punto de goal or goal es: " << this->next_x_mov << ' ' << this->next_z_mov << std::endl;
    this->str_forward_dis = -1;
  }
  else if (is_goal_wall(&obs1) and not is_goal_wall(&obs2)) {
    this->next_x_mov = (this->goalqr_x + obs1.pos.x)/2.0;
    this->next_z_mov = obs2.pos.z;
    this->str_forward_dis = fabs(this->goalqr_z - this->next_z_mov);
  }
  else if (not is_goal_wall(&obs1) and is_goal_wall(&obs2)) {
    this->next_x_mov = (this->goalqr_x + obs2.pos.x)/2.0;
    this->next_z_mov = obs1.pos.z;
    this->str_forward_dis = fabs(this->goalqr_z - this->next_z_mov);
  }

  //if (is_goal_wall(&obs1) or is_goal_wall(&obs2)) this->darwin_state = STRAIGHT_FORWARD;
  //this->next_x_mov = this->goalqr_x; this->next_z_mov = this->goalqr_z + 0.2*way_zaxis_grow;



  this->nm_x = distance_to_xy(this->next_x_mov, this->next_z_mov);
  if (this->nm_x > 1.0) this->nm_x = 1.0;
  this->nm_alpha = (atan2(this->next_x_mov, this->next_z_mov));

  std::cout << "Me iba a mover a: " << this->next_x_mov << ' ' << this->next_z_mov << std::endl;

  /*int i = 0;
  bool found = false;
  DDPOINT nearest_conflictive_point;
  nearest_conflictive_point.x = +9999.0; nearest_conflictive_point.z = +9999;
  while (i < qr_information.size()) {
    qr_info qr;
    qr = qr_information [i];

    double xqr, zqr;
    xqr = qr.pos.x;
    zqr = qr.pos.z;

    double dis = distance_to_xy(this->next_x_mov, this->next_z_mov);
    double dis_to_line = (fabs(this->next_x_mov*xqr + this->next_z_mov*zqr)/dis);

    if (dis >= 0.0 and dis <= this->nm_x + this->config_.ERROR_PERMES) {
      std::cout << "-------------------------------" << std::endl;
      std::cout << "Distancia de la recta de trayectoria: " << dis_to_line << std::endl;
      std::cout << "Con el QR: " << qr.qr_tag << std::endl;

      if (xqr < nearest_conflictive_point.x and zqr < nearest_conflictive_point.z and dis_to_line <= 0.6 - this->config_.ERROR_PERMES) {
        nearest_conflictive_point.x = xqr;
        nearest_conflictive_point.z = zqr;
        std::cout << " Hey, es punto conflictivo..." << std::endl;
        std::cout << xqr << ' ' << zqr << std::endl;
        if (not found) found = true;
      }
      std::cout << "-------------------------------" << std::endl;
    }

    ++i;
  }

  if (found) {this->next_x_mov = this->goalqr_x; this->next_z_mov = nearest_conflictive_point.z;}
  std::cout << "Me muevo a: " << this->next_x_mov << ' ' << this->next_z_mov << std::endl;
  std::cout << "Ademas lo hare de tal manera: " << this->nm_x << ' ' << this->nm_alpha << std::endl;
  std::cout << "Una vez llegado al punto, me quedara: " << this->str_forward_dis << " para llegar a la meta." << std::endl;
*/
}

void CeabotMazeAlgNode::movement_alpha_for_goal_qr () {
  if (this->nm_alpha < 0 and not this->half_maze_achieved) this->turn_left = -1;
  else if (this->nm_alpha > 0 and not this->half_maze_achieved) this->turn_left = +1;
  else if (this->nm_alpha < 0 and this->half_maze_achieved) this->turn_left = +1;
  else if (this->nm_alpha > 0 and this->half_maze_achieved) this->turn_left = -1;

  std::cout << "Angle a moure: " << this->nm_alpha << std::endl;
  this->mov_alpha_goal = get_goal_alpha(this->nm_alpha, this->bno055_measurement);
  double alpha_sat = saturate_alpha(get_magnitude_alpha(this->mov_alpha_goal, this->bno055_measurement));
  double diff = fabs(this->mov_alpha_goal - this->bno055_measurement);
  if (this->nm_alpha >= -this->config_.ERROR_PERMES and this->nm_alpha <= +this->config_.ERROR_PERMES) this->darwin_state = STRAIGHT_FORWARD;
  else {this->walk.set_steps_size(0.0, 0.0, this->config_.p * alpha_sat * this->turn_left);   this->darwin_state = CHECK_ALPHA_GOAL_MFGQR;}

}

void CeabotMazeAlgNode::check_alpha_goal_mfgqr () {
  std::cout << "mov_alpha_goal: " << this->mov_alpha_goal << std::endl;
  std::cout << "bno055: " << this->bno055_measurement << std::endl;

  double diff = fabs(this->mov_alpha_goal - this->bno055_measurement);
  std::cout << "diff: " << diff << std::endl;
  if (diff >= -this->config_.ERROR_PERMES and diff <= +this->config_.ERROR_PERMES) {
    ROS_INFO ("Alpha QRGOAL achieved...");
    this->walk.stop();
    this->darwin_state = MOVEMENT_X_FOR_GOAL_QR;
  }
  else {
    double tom = saturate_alpha(this->config_.p * diff * this->turn_left);
    this->walk.set_steps_size(0.0, 0.0, tom);
  }
}

void CeabotMazeAlgNode::movement_x_for_goal_qr () {
  this->ini_x = this->odom_x;
  this->ini_z = this->odom_y;
  double x_sat = saturate_movement(this->nm_x);
  this->walk.set_steps_size(x_sat, 0.0, 0.0);

  this->darwin_state = CHECK_GOAL_MFGQR;
}

void CeabotMazeAlgNode::check_goal_mfgqr () {
  double distance_to_ini = distance_to_xy(this->ini_x - this->odom_x, this->ini_z - this->odom_y);
  double diff = this->nm_x - distance_to_ini;

  if (diff >= -this->config_.ERROR_PERMES and diff <= +this->config_.ERROR_PERMES) {
    this->state_stn = CHECK_NORTH;
    this->state_sts = CHECK_SOUTH;
    ROS_INFO("MFQR goal achieved...");
    this->walk.stop();
    this->darwin_state = STRAIGHT_FORWARD;
  }
  else {
    double tom = saturate_movement(this->config_.p * diff);
    this->walk.set_steps_size(tom, 0.0, 0.0);
  }
}

void CeabotMazeAlgNode::straight_forward () {
  std::cout << "hfmaze: " << this->half_maze_achieved << std::endl;
  if (this->str_forward_dis < 0) this->str_forward_dis = distance_to_xy(this->goalqr_x - this->odom_x, this->goalqr_z - this->odom_y);
  if ((not this->half_maze_achieved and straight_to_north()) or (this->half_maze_achieved and straight_to_south())) {
    double goal = this->str_forward_dis;
    double sat = saturate_movement(goal);
    this->ini_x = this->odom_x;
    this->ini_z = this->odom_y;

    this->walk.set_steps_size (sat, 0.0, 0.0);

    this->darwin_state = CHECK_STRAIGHT_FORWARD;
  }

}

void CeabotMazeAlgNode::check_straight_forward () {
  double distance_to_ini = distance_to_xy(this->ini_x - this->odom_x, this->ini_z - this->odom_y);
  double diff = fabs(this->str_forward_dis - distance_to_ini);
  std::cout << "Diferencia de distancia al QRRRRR: " << diff << std::endl;
  if (diff >= -this->config_.ERROR_PERMES and diff <= +this->config_.ERROR_PERMES) {
    ROS_INFO("QR_GOAL or partial QR_GOAL achieved ...");
    //double distance_to_goal = distance_to_xy(this->goalqr_x - (this->ini_x - this->odom_x), this->goalqr_z - (this->ini_z - this->odom_y));
    std::cout << "final distance to goal: " << diff << std::endl;
    std::cout << "nueva supuesta: " << distance_to_xy(this->goalqr_z, this->odom_y) << std::endl;
    this->walk.stop();
    this->darwin_state = IDLE;
    if (diff >= 0.2 - this->config_.ERROR_PERMES and diff <= 0.4 + this->config_.ERROR_PERMES) this->half_maze_achieved = true;
  }
  else {
    double tom = saturate_movement (diff);
    this->walk.set_steps_size(tom, 0.0, 0.0);
  }
}

bool CeabotMazeAlgNode::NS_orientation (qr_info obs) {
  std::pair < std::string, int > aux;
  aux = divide_qr_tag (obs.qr_tag);
  if (aux.first == "N" or aux.first == "S") return true;

  return false;
}

/* main function */
int main(int argc,char *argv[]) {
  return algorithm_base::main<CeabotMazeAlgNode>(argc, argv, "ceabot_maze_alg_node");
}
