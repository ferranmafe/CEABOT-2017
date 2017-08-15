#include "ceabot_vision_alg_node.h"

CeabotVisionAlgNode::CeabotVisionAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<CeabotVisionAlgorithm>(),
  walk("ceabot_vision_walk"),
  tracking_module("ceabot_vision_track")
{
  //init class attributes if necessary
  //Initialization of Darwin Vision Event parameters
  this->event_start = false;
  this->movement_started = false;

  this->search_started=false;
  this->pan_angle=0.0;
  this->current_pan_angle=config_.PAN_ANGLE;
  this->tilt_angle=PI/2;
  this->current_tilt_angle=config_.TILT_ANGLE;
  this->turn_angle = 0.0;
  this->QR_identifier = "None";
  this->turn_left = 1;
  this->bno_readed_first_time = false;
  //this->loop_rate_ = 2;//in [Hz]

  // [init publishers]

  // [init subscribers]
  this->buttons_subscriber_ = this->public_node_handle_.subscribe("buttons", 1, &CeabotVisionAlgNode::buttons_callback, this);
  pthread_mutex_init(&this->buttons_mutex_,NULL);

  this->imu_subscriber_ = this->public_node_handle_.subscribe("imu", 1, &CeabotVisionAlgNode::imu_callback, this);
  pthread_mutex_init(&this->imu_mutex_,NULL);

  this->odom_subscriber_ = this->public_node_handle_.subscribe("odom", 1, &CeabotVisionAlgNode::odom_callback, this);
  pthread_mutex_init(&this->odom_mutex_,NULL); //By the moment we're just going to use the bno055 sensor

  this->joint_states_subscriber_ = this->public_node_handle_.subscribe("joint_states", 1, &CeabotVisionAlgNode::joint_states_callback, this);
  pthread_mutex_init(&this->joint_states_mutex_,NULL);

  this->qr_pose_subscriber_ = this->public_node_handle_.subscribe("qr_pose", 1, &CeabotVisionAlgNode::qr_pose_callback, this);
  pthread_mutex_init(&this->qr_pose_mutex_,NULL);

  // [init services]

  // [init clients]

  // [init action servers]

  // [init action clients]
}

CeabotVisionAlgNode::~CeabotVisionAlgNode(void) {
  // [free dynamic memory]
  pthread_mutex_destroy(&this->buttons_mutex_);
  pthread_mutex_destroy(&this->imu_mutex_);
  pthread_mutex_destroy(&this->odom_mutex_);
  pthread_mutex_destroy(&this->joint_states_mutex_);
  pthread_mutex_destroy(&this->qr_pose_mutex_);
}


void CeabotVisionAlgNode::mainNodeThread(void) {
  // [fill msg structures]

  // [fill srv structure and make request to the server]

  // [fill action structure and make request to the action server]

  // [publish messages]

  //State Machine. After the start the idea is to iterate between the different states
  //SEARCH_QR -> DECODE_QR -> MOVEMENT -> MOVEMENT_ERROR_COMPROBATION -> SEARCH_QR
  if (this->event_start) {
    state_machine();
  }
}

/*  [subscriber callbacks] */
void CeabotVisionAlgNode::buttons_callback(const dynamic_reconfigure::Config::ConstPtr& msg)
{
  //ROS_INFO("CeabotVisionAlgNode::buttons_callback: New Message Received");

  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
  //this->buttons_mutex_enter();
  bool value = false;
  for (int k = 0; k < msg->bools.size(); ++k) {
    if (msg->bools[k].name == "start_button" and msg->bools[k].value) value = true;
  }
  if (value and !this->event_start) {
    //Code for the start. It sets all the configuration from the .cfg to the modules.
    //Then, waits for 5 sec (start condition - read CEABOT rules) and change the Darwin
    //state to start searching for the first QR code
    //Initialization of Darwin parameters
    init_walk_module();
    init_headt_module();

    //5 sec sleep. It's for the init of the event, the rules says that Darwin must
    //wait this amount of time in seconds
    ros::Duration(5.0).sleep();

    //Change of Darwin State to start searching for the first QR
    //this->darwin_state = SEARCH_QR;
    //this->darwin_state = darwin_states(config_.darwin_st);

    //We must set down the start flag in terms of dont enter to this condition again
    this->event_start = true;
    this->darwin_state = IDLE;
  }
  //std::cout << msg->data << std::endl;
  //unlock previously blocked shared variables
  //this->alg_.unlock();
  //this->buttons_mutex_exit();
}

void CeabotVisionAlgNode::buttons_mutex_enter(void)
{
  pthread_mutex_lock(&this->buttons_mutex_);
}

void CeabotVisionAlgNode::buttons_mutex_exit(void)
{
  pthread_mutex_unlock(&this->buttons_mutex_);
}

void CeabotVisionAlgNode::imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  //ROS_INFO("CeabotVisionAlgNode::imu_callback: New Message Received");

  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
  //this->imu_mutex_enter();
  double bnoaux = tf::getYaw(msg->orientation);
  if (bnoaux < 0) bnoaux += 2*PI;
  this->bno055_measurement = bnoaux; //We normalize the measurement...
  if (!this->bno_readed_first_time) {
    this->old_goal_bno = bnoaux;
    this->bno_readed_first_time = true;
  }
  //ROS_INFO("Darwin Ceabot Vision : The actual Yaw is : %f", this->bno055_measurement);
  //std::cout << msg->data << std::endl;
  //unlock previously blocked shared variables
  //this->alg_.unlock();
  //this->imu_mutex_exit();
}

void CeabotVisionAlgNode::imu_mutex_enter(void)
{
  pthread_mutex_lock(&this->imu_mutex_);
}

void CeabotVisionAlgNode::imu_mutex_exit(void)
{
  pthread_mutex_unlock(&this->imu_mutex_);
}

void CeabotVisionAlgNode::odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  //ROS_INFO("CeabotVisionAlgNode::odom_callback: New Message Received");

  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
  //this->odom_mutex_enter();

  //std::cout << msg->data << std::endl;
  //unlock previously blocked shared variables
  //this->alg_.unlock();
  //this->odom_mutex_exit();
}

void CeabotVisionAlgNode::odom_mutex_enter(void)
{
  pthread_mutex_lock(&this->odom_mutex_);
}

void CeabotVisionAlgNode::odom_mutex_exit(void)
{
  pthread_mutex_unlock(&this->odom_mutex_);
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                              ESTAS FUNCIONES ESTÁN COPIADAS TAL CUAL DEL QR_HEAD_TRACKING_NODE (POR COMENTAR)
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void CeabotVisionAlgNode::joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg) {
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
  //ROS_INFO("angle_pan: %f angle_tilt: %f",this->current_pan_angle,this->current_tilt_angle);
  //unlock previously blocked shared variables
  //this->alg_.unlock();
  this->joint_states_mutex_exit();
}


void CeabotVisionAlgNode::joint_states_mutex_enter(void) {
  pthread_mutex_lock(&this->joint_states_mutex_);
}


void CeabotVisionAlgNode::joint_states_mutex_exit(void) {
  pthread_mutex_unlock(&this->joint_states_mutex_);
}


void CeabotVisionAlgNode::qr_pose_callback(const humanoid_common_msgs::tag_pose_array::ConstPtr& msg) {
  this->qr_pose_mutex_enter();
  if(msg->tags.size()>0)
  {
    std::cout << obtain_angle_against_darwins_head(msg->tags[0].position.x, msg->tags[0].position.z) << std::endl;
    if (this->darwin_state == IDLE and this->event_start and this->QR_identifier == "None") {


      if (msg->tags.size() == 1) {
        this->darwin_state = MOVEMENT;
        this->QR_identifier = msg->tags[0].tag_id; //Se puede añadir en un and que la posicion sea distinta a todos los leidos anteriormente
        double aux_old_goal_bno = this->bno055_measurement + obtain_angle_against_darwins_head(msg->tags[0].position.x, msg->tags[0].position.z);
        if (aux_old_goal_bno >= 2.0 * PI) aux_old_goal_bno -= 2.0 * PI;
        this->old_goal_bno = aux_old_goal_bno;
      }
      else {
          /*std::cout << "----------------------------" << std::endl;
          std::cout << "More than 1 QR tags detected" << std::endl;*/
          std::string aux_tag_id = msg->tags[0].tag_id;
          double x_tag_aux = msg->tags[0].position.x;
          double z_tag_aux = msg->tags[0].position.z;

          double goal_calculated_old_qr = this->old_goal_bno - this->turn_angle;
          double goal_calculated_new_qr = this->bno055_measurement + obtain_angle_against_darwins_head(x_tag_aux, z_tag_aux);
          double qr_difference_respect_goal = abs(goal_calculated_new_qr - goal_calculated_old_qr);
          /*std::cout << "TAG ID: " << aux_tag_id << std::endl;
          std::cout << "Angle against darwin: " << - this->turn_angle << std::endl;
          std::cout << "Actual position for the QR: " << (this->bno055_measurement - obtain_angle_against_darwins_head(x_tag_aux, z_tag_aux)) << std::endl; // + obtain_angle_against_darwins_head(x_tag_aux, z_tag_aux)) << std::endl;
          std::cout << "Calculo ideal del QR for the QR: " << (this->old_goal_bno + this->turn_angle) << std::endl;
          std::cout << -PI/2.0 << std::endl;
          std::cout << "X TAG: " << x_tag_aux << std::endl;
          std::cout << "Z TAG: " << z_tag_aux << std::endl;
          std::cout << "QR DIFFERENCE RESPECT GOAL: " << qr_difference_respect_goal << std::endl;
          std::cout << std::endl;
          std::cout << std::endl;*/

          for (int i = 1; i < msg->tags.size(); ++i) {
              goal_calculated_new_qr = this->bno055_measurement + obtain_angle_against_darwins_head(msg->tags[i].position.x, msg->tags[i].position.z);
              double new_qr_difference_to_test = abs(goal_calculated_new_qr - goal_calculated_old_qr);
              /*std::cout << "TAG ID: " << msg->tags[i].tag_id << std::endl;
              std::cout << "NEW QR DIFFERENCE RESPECT GOAL: " << new_qr_difference_to_test << std::endl;*/

              if (new_qr_difference_to_test < qr_difference_respect_goal) {
                  qr_difference_respect_goal = new_qr_difference_to_test;
                  aux_tag_id = msg->tags[i].tag_id;
                  x_tag_aux = msg->tags[i].position.x;
                  z_tag_aux = msg->tags[i].position.z;
                  /*std::cout << "TAG ID: " << aux_tag_id << std::endl;
                  std::cout << "X TAG: " << x_tag_aux << std::endl;
                  std::cout << "Z TAG: " << z_tag_aux << std::endl;
                  std::cout << "QR DIFFERENCE RESPECT GOAL: " << qr_difference_respect_goal << std::endl;
                  std::cout << std::endl;
                  std::cout << std::endl;*/
              }
          }
          this->darwin_state = MOVEMENT;
          this->QR_identifier = aux_tag_id;
          double aux_old_goal_bno = this->bno055_measurement + obtain_angle_against_darwins_head(x_tag_aux, z_tag_aux);
          if (aux_old_goal_bno >= 2.0 * PI) aux_old_goal_bno -= 2.0 * PI;
          this->old_goal_bno = aux_old_goal_bno;
          //std::cout << "-----------------" << std::endl;
      }

    }
    //this->pan_angle=this->current_pan_angle+atan2(msg->tags[0].position.x,msg->tags[0].position.z);
    //this->tilt_angle=this->current_tilt_angle+atan2(msg->tags[0].position.y,msg->tags[0].position.z);
    //ROS_INFO("Next target pan angle: %f (%f,%f)",this->pan_angle,this->current_pan_angle,atan2(msg->tags[0].position.x,msg->tags[0].position.z));
    //ROS_INFO("Next target tilt angle: %f (%f,%f)",this->tilt_angle,this->current_tilt_angle,atan2(msg->tags[0].position.y,msg->tags[0].position.z));
  }
  this->qr_pose_mutex_exit();
}


void CeabotVisionAlgNode::qr_pose_mutex_enter(void) {
  pthread_mutex_lock(&this->qr_pose_mutex_);
}


void CeabotVisionAlgNode::qr_pose_mutex_exit(void) {
  pthread_mutex_unlock(&this->qr_pose_mutex_);
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void CeabotVisionAlgNode::node_config_update(Config &config, uint32_t level) {
  this->alg_.lock();
  this->config_=config;
  this->alg_.unlock();
}


void CeabotVisionAlgNode::addNodeDiagnostics(void){}

//Function that initialize all Darwin Walk Parameters
void CeabotVisionAlgNode::init_walk_module(void) {
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

void CeabotVisionAlgNode::init_headt_module(void) {
  //ROS_INFO("Darwin Ceabot Vision : HeadTracking Parameters Initialization");
  this->tracking_module.set_pan_range(config_.max_pan, config_.min_pan);
  this->tracking_module.set_tilt_range(config_.max_tilt, config_.min_tilt);
  this->tracking_module.set_pan_pid(config_.pan_p, config_.pan_i, config_.pan_d, config_.pan_i_clamp);
  this->tracking_module.set_tilt_pid(config_.tilt_p, config_.tilt_i, config_.tilt_d, config_.tilt_i_clamp);
}

//State Machine. After the start the idea is to iterate between the different states
//SEARCH_QR -> DECODE_QR -> MOVEMENT -> MOVEMENT_ERROR_COMPROBATION -> SEARCH_QR
void CeabotVisionAlgNode::state_machine(void) {
  switch(this->darwin_state) {
    //Case of no movement and no scanning
    case IDLE: //0 ------------------------------------------------------------------------------------------------------------------------------------
      //ROS_INFO("Darwin Ceabot Vision : state IDLE");
      //if (this->movement_started) {ros::duration.sleep(0.1); this->movement_started = false;} //We wait to get the correct QR...
      //this->darwin_state = MOVEMENT;

      break;
    case SEARCH_QR: //1 -------------------------------------------------------------------------------------------------------------------------------
      //ROS_INFO("Darwin Ceabot Vision : state SEARCH_QR");
      this->pan_angle = config_.PAN_ANGLE;
      this->tilt_angle = config_.TILT_ANGLE;
      if (!search_started) {
        //ROS_INFO("HEY, I JUST STARTED SEARCHING!!");
        this->tracking_module.start_tracking(this->pan_angle,this->tilt_angle);
        this->search_started=true;
      }
      else this->tracking_module.update_target(this->pan_angle,this->tilt_angle);

      break;

    case MOVEMENT: //2 ----------------------------------------------------------------------------------------------------------------------------------
      //Only enter to this if-condition in the first iteration of the mainNodeThread in
      //MOVEMENT case
      if (!this->movement_started) {
        //ROS_INFO("Darwin Ceabot Vision : state MOVEMENT -> Starting Movement");
        //We have to control manually if the movement is started
        this->movement_started = true;
        //Walk works in rad. We must convert the angle from degrees (QR Code returns
        //the angle in degrees)
        //std::cout << this->QR_identifier << std::endl;
        setPanFromQR_id();
        //ROS_INFO("Darwin Ceabot Vision : turn_angle %f, turn_left %f", this->turn_angle, this->turn_left);
        this->turn_angle = this->turn_left*(this->turn_angle * PI) / 180.0;

        this->goal = get_goal(this->turn_angle, this->bno055_measurement);
        this->turn_angle = this->turn_left*get_magnitude(this->goal, this->bno055_measurement);

        this->turn_angle = saturate(this->turn_angle); //We must saturate it
        std::cout << this->turn_angle << std::endl;
        this->walk.set_steps_size(0.0, 0.0, (this->config_.p*(this->turn_angle))); //We multiply the turn angle by a coeficient to deal with the end movement of Darwin (p)
        this->darwin_state = CHECK_GOAL;
      }


      /*else {
        //We must wait until the movement is finished
        if (this->walk.is_finished()) {
          ROS_INFO("Darwin Ceabot Vision : state MOVEMENT -> Move Finished");
          this->darwin_state = MOVEMENT_ERROR_COMPROBATION;
          this->movement_started = false;
        }
        else ROS_INFO("Darwin Ceabot Vision : state MOVEMENT -> Moving");
      }*/

      break;

    //Case after movement. Before the next scan we must verify if we've made the
    //movement correctly
    case CHECK_GOAL: {//3 ------------------------------------------------------------------------------------------------------------------
      //ROS_INFO("Darwin Ceabot Vision : state CHECK_GOAL");
      double diff = fabs(this->goal - (this->bno055_measurement));

      if (diff >= -(this->config_.ERROR_PERMES) and diff <= this->config_.ERROR_PERMES) {
        //ROS_INFO("Darwin Ceabot Vision : Goal achieved, moving on for the next one!");
        this->walk.stop(); //Stop Darwin
        this->darwin_state = WAIT_STOP_WALKING;
      }
      else {
        double tom = saturate(this->config_.p * diff * this->turn_left);
        this->walk.set_steps_size(0.0, 0.0, tom);
      }

      break;
    }

    case WAIT_STOP_WALKING:
      //ROS_INFO("Darwin Ceabot Vision : state WAIT_STOP_WALKING");
      if (this->walk.is_finished() and this->walk.get_status() == walk_module_status_t(WALK_MODULE_SUCCESS)) { //If we finished waliing successfully!
        this->movement_started = false;
        this->darwin_state = IDLE;
        this->QR_identifier = "None";
      }
      //Check if the robot is fallen?
      break;
  }
}

void CeabotVisionAlgNode::setPanFromQR_id() {
  bool es_letra = true; //Use sscanf from the C++ libraries?
  std::string auxiliar = this->QR_identifier;
  int beg, end;
  beg = end = 0;
  char* final_string;
  for (int k = 0; k < auxiliar.size(); ++k) {
    if (es_letra and (auxiliar [k] >= '0' and auxiliar [k] <= '9')) //If we've been taking letters and we find a number...
      { beg = k; es_letra = false; }
    else if (!es_letra and !(auxiliar [k] >= '0' and auxiliar [k] <= '9')) { //If it's a number and we find a letter...
       end = k - 1;
       es_letra = true;
       if (auxiliar [k] == 'R') this->turn_left = -1;
       else if (auxiliar [k] == 'L') this->turn_left = 1;
    }
  }
  auxiliar = auxiliar.substr(beg, end - beg + 1);
  final_string = &auxiliar[0];
  this->turn_angle = std::atoi(final_string);
}

double CeabotVisionAlgNode::get_magnitude (double alpha, double beta) {
  //std::cout << alpha*180.0/PI << ' ' << beta*180.0/PI << std::endl;
  double mgn = alpha - beta;
  if (mgn > PI)  //That means we are not performing the best move
    mgn -= 2*PI;
  else if (mgn < (-PI)) //Same case
    mgn += 2*PI;
  //std::cout << mgn << std::endl;
  return fabs(mgn); //We do not return the abs, just because the direction matters

}

double CeabotVisionAlgNode::get_goal (double alpha, double beta) { // "Calculate and Normalize"
  /*double goal = alpha + beta; //Alpha is magnitude and beta is the actual yaw...
  if (goal < (-PI)) goal += 2*PI;
  else if (goal > PI) goal -= 2*PI;

  return goal;*/

  double goal = alpha + beta;
  if (goal < 0) goal += (2*PI);
  else if (goal > (2*PI)) goal -= (2*PI);

  return goal;

}

double CeabotVisionAlgNode::saturate (double alpha) {
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

double CeabotVisionAlgNode::obtain_angle_against_darwins_head(double x, double z){
    return atan2(x, z);
}


/* main function */
int main(int argc,char *argv[]){
  return algorithm_base::main<CeabotVisionAlgNode>(argc, argv, "ceabot_vision_alg_node");
}
