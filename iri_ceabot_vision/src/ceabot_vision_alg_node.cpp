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
  this->tilt_angle=0.0;
  this->turn_angle = 0.0;
  this->QR_identifier = "None";
  this->turn_left = 1;
  this->old_turn_angle = 0.0;
  this->head_search_started = false;
  this->bno_readed_first_time = false;
  this->head_state = 0;
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
    std::cout << "Event Started!" << std::endl;
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
  std::cout << "I entered to the joint_states_callback()" << std::endl;
  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
  this->joint_states_mutex_enter();

  for (i = 0; i < msg->name.size() ; ++i){
    if (msg->name[i]=="j_pan"){
      this->current_pan_angle=msg->position[i];
      std::cout << "Current Pan Angle seen as a msg: " << msg->position[i] << std::endl;
    }
    else if (msg->name[i]=="j_tilt"){
      this->current_tilt_angle=(msg->position[i]);
            std::cout << "Current Tilt Angle seen as a msg: " << msg->position[i] << std::endl;
    }
  }
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
  if (this->darwin_state == IDLE and this->event_start and this->QR_identifier == "None") {
      //From the QR Code that we are seeing, we discard the ones which we've seen before.
      std::cout << "-------------------------------" << std::endl;
      std::vector<qr_info> vec_aux;
      for (int i = 0; i < msg->tags.size(); ++i) {
        if (!qr_seen_before(msg->tags[i].tag_id)) {
          qr_info aux;
          aux.tag_id =  msg->tags[i].tag_id; aux.x = msg->tags[i].position.x; aux.z = msg->tags[i].position.z;
          std::cout << "TAG ID: " << aux.tag_id << std::endl;
          std::cout << std::endl;
          vec_aux.push_back(aux);
        }
      }
      qr_to_process = vec_aux;
      //When we have the QR Code that we haven't seen before, we have to choose the one closest to
      //our ideal target.
      if (qr_to_process.size() > 0) {
          qr_info next_qr_to_go = choose_the_correct_qr();

          this->darwin_state = MOVEMENT;
          this->QR_identifier = next_qr_to_go.tag_id;

          double aux_old_goal_bno = this->bno055_measurement + obtain_angle_against_darwins_head(next_qr_to_go.x, next_qr_to_go.z);
          normalize_angle(aux_old_goal_bno);

          qr_seen.insert(next_qr_to_go.tag_id);
          std::cout << "FINAL TAG ID: " << next_qr_to_go.tag_id << std::endl;

          this->old_goal_bno = aux_old_goal_bno;
          if (head_search_started) {
            head_search_started = false;
            head_state = 0;
            this->tracking_module.stop_tracking();
          }
      }
      else {
        std::cout << "NOT QR DETECTED" << std::endl;
        if (!head_search_started) {
          head_state = 1;
          update_pan_and_tilt();
        }
        darwin_state = SEARCH_QR;
      }
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
  std::cout << "Current Pan: " << this->current_pan_angle << " Current Tilt: " << this->current_tilt_angle << std::endl;
  switch(this->darwin_state) {
    //Case of no movement and no scanning
    case IDLE: //0 ------------------------------------------------------------------------------------------------------------------------------------
      //ROS_INFO("Darwin Ceabot Vision : state IDLE");
      //if (this->movement_started) {ros::duration.sleep(0.1); this->movement_started = false;} //We wait to get the correct QR...
      //this->darwin_state = MOVEMENT;

      break;
    case SEARCH_QR: //1 -------------------------------------------------------------------------------------------------------------------------------
      if (!head_search_started) {
        ROS_INFO("HEY, I JUST STARTED SEARCHING!!");
        this->tracking_module.start_tracking(this->pan_angle,this->tilt_angle);
        this->head_search_started=true;
      }
      else this->tracking_module.update_target(this->pan_angle,this->tilt_angle);
      darwin_state = WAIT_SEARCH;
      break;

    case WAIT_SEARCH:
        ROS_INFO("WAITING SEARCH");
        std::cout << "-------------------------" << std::endl;
        std::cout << "Current Pan: " << this->current_pan_angle << " Current Tilt: " << this->current_tilt_angle << std::endl;
        std::cout << "Pan: " << this->pan_angle << " Tilt: " << this->tilt_angle << std::endl;
        std::cout << "-------------------------" << std::endl;

        if (this->current_pan_angle >= (this->pan_angle - ERROR) and this->current_pan_angle <= (this->pan_angle + ERROR)) {
            if (this->current_tilt_angle >= (this->tilt_angle - ERROR) and this->current_tilt_angle <= (this->tilt_angle + ERROR)){
                ROS_INFO("GOAL ACHIEVED");
                ++head_state;
                if (head_state <= 4) update_pan_and_tilt();
                else head_state = 0;
                darwin_state = IDLE;
            }
        }
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
        this->old_turn_angle = this->turn_angle;
        std::cout << "Angle to turn: " << this->turn_angle << std::endl;
        this->goal = get_goal(this->turn_angle, this->bno055_measurement);
        this->turn_angle = this->turn_left*get_magnitude(this->goal, this->bno055_measurement);
        this->turn_angle = saturate(this->turn_angle); //We must saturate it

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

void CeabotVisionAlgNode::normalize_angle(double &old_angle) {
    if (old_angle >= 2.0 * PI) old_angle -= 2.0 * PI;
    else if (old_angle < 0.0) old_angle += 2.0 * PI;
}

bool CeabotVisionAlgNode::qr_seen_before(std::string qr_tag) {
  return (!(qr_seen.find(qr_tag) == qr_seen.end()));
}

qr_info CeabotVisionAlgNode::choose_the_correct_qr(void) {
  int next_qr = 0;

  double goal_calculated_old_qr = this->old_goal_bno + this->old_turn_angle;
  normalize_angle(goal_calculated_old_qr);

  double goal_calculated_new_qr = this->bno055_measurement + obtain_angle_against_darwins_head(qr_to_process[0].x, qr_to_process[0].z);
  normalize_angle(goal_calculated_new_qr);

  double qr_difference_respect_goal = goal_calculated_new_qr - goal_calculated_old_qr;
  obtain_absolute_value(qr_difference_respect_goal);

  for (int i = 1; i < qr_to_process.size(); ++i) {
    goal_calculated_new_qr = this->bno055_measurement + obtain_angle_against_darwins_head(qr_to_process[i].x, qr_to_process[i].z);

    normalize_angle(goal_calculated_new_qr);

    double new_qr_difference_to_test = goal_calculated_new_qr - goal_calculated_old_qr;
    obtain_absolute_value(new_qr_difference_to_test);

    if (new_qr_difference_to_test < qr_difference_respect_goal) {
        qr_difference_respect_goal = new_qr_difference_to_test;
        next_qr = i;
    }
  }
  return qr_to_process[next_qr];
}

void CeabotVisionAlgNode::obtain_absolute_value (double &angle){
  if (angle < 0.0) angle *= -1;
  else if (angle > PI) angle = 2 * PI - angle;
  else if (angle < -PI) angle += 2 * PI;
}

void CeabotVisionAlgNode::update_pan_and_tilt(void) {
  switch (head_state) {
    case 0:
        this->pan_angle = 0.0;
        this->tilt_angle = PI / 4;
        break;

    case 1:
        this->pan_angle = 0.0;
        this->tilt_angle = PI / 2;
        break;

    case 2:
        this->pan_angle = 0.0;
        this->tilt_angle = 0.0;
        break;

    case 3:
        this->pan_angle = PI / 8;
        this->tilt_angle = PI / 4;
        break;

    case 4:
        this->pan_angle = - PI / 8;
        this->tilt_angle = PI / 4;
        break;
  }
}

/* main function */
int main(int argc,char *argv[]){
  return algorithm_base::main<CeabotVisionAlgNode>(argc, argv, "ceabot_vision_alg_node");
}
