// Copyright (C) 2010-2011 Institut de Robotica i Informatica Industrial, CSIC-UPC.
// Author
// All rights reserved.
//
// This file is part of iri-ros-pkg
// iri-ros-pkg is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// IMPORTANT NOTE: This code has been generated through a script from the
// iri_ros_scripts. Please do NOT delete any comments to guarantee the correctness
// of the scripts. ROS topics can be easly add by using those scripts. Please
// refer to the IRI wiki page for more information:
// http://wikiri.upc.es/index.php/Robotics_Lab

#ifndef _ceabot_vision_alg_node_h_
#define _ceabot_vision_alg_node_h_

#include <iri_base_algorithm/iri_base_algorithm.h>
#include "ceabot_vision_alg.h"

#include <humanoid_modules/walk_module.h>
#include <humanoid_modules/head_tracking_module.h>
#define PI 3.1415926535
#define GRADIENT 0.159154943

// [publisher subscriber headers]
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <humanoid_common_msgs/tag_pose_array.h>
#include <string>
#include <tf/tf.h>
// [service client headers]

// [action server client headers]

/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */

 typedef enum {IDLE,
               SEARCH_QR,
               MOVEMENT,
               CHECK_GOAL} darwin_states;

class CeabotVisionAlgNode : public algorithm_base::IriBaseAlgorithm<CeabotVisionAlgorithm>
{
  private:
    // [publisher attributes]

    // [subscriber attributes]
    ros::Subscriber imu_subscriber_;
    void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
    pthread_mutex_t imu_mutex_;
    void imu_mutex_enter(void);
    void imu_mutex_exit(void);

    ros::Subscriber odom_subscriber_;
    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    pthread_mutex_t odom_mutex_;
    void odom_mutex_enter(void);
    void odom_mutex_exit(void);

    ros::Subscriber joint_states_subscriber_;
    void joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg);
    pthread_mutex_t joint_states_mutex_;
    void joint_states_mutex_enter(void);
    void joint_states_mutex_exit(void);

    ros::Subscriber qr_pose_subscriber_;
    void qr_pose_callback(const humanoid_common_msgs::tag_pose_array::ConstPtr& msg);
    pthread_mutex_t qr_pose_mutex_;
    void qr_pose_mutex_enter(void);
    void qr_pose_mutex_exit(void);
    // [service attributes]

    // [client attributes]

    // [action server attributes]

    // [action client attributes]

   /**
    * \brief config variable
    *
    * This variable has all the driver parameters defined in the cfg config file.
    * Is updated everytime function config_update() is called.
    */

    //double concerning to the Yaw of Darwin
    double bno055_measurement;

    //tag_id obtained while decoding
    std::string QR_identifier;

    //Flag which allows Darwin to wait the first 5 sec of the event
    bool event_start;

    // Variable which controls the next action to do for Darwin
    darwin_states darwin_state;

    //Parameters of the next move of Darwin (direction and angle)
    int turn_left; //If false, turn right

    //Goal to achieve
    double goal;

    //Angle we set to turn!
    double turn_angle;

    //Class variable that allows Ceabot Vision node to interact with Darwin low level controllers
    //In terms of walking configuration and parameters
    CWalkModule walk;

    //Flag which indicates if a movement is started or not (but it doesn't indicate if the movement is finished)
    bool movement_started;

    //Class variable that allows Ceabot Vision node to interact with Darwin low level controllers
    //In terms of moving the head camera
    CHeadTrackingModule tracking_module;

    //Pan angle stored in Darwin. It's usually refered to the last pan angle moved
    double pan_angle;

    //Current pan angle returned by the low level controllers
    double current_pan_angle;

    //Tilt angle stored in Darwin. It's usually refered to the last pan angle moved
    double tilt_angle;

    //Current tilt angle returned by the low level controllers
    double current_tilt_angle;

    bool search_started;

    //Parameter to access to the data of the .cfg file
    Config config_;
  public:
   /**
    * \brief Constructor
    *
    * This constructor initializes specific class attributes and all ROS
    * communications variables to enable message exchange.
    */
    CeabotVisionAlgNode(void);

   /**
    * \brief Destructor
    *
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
    ~CeabotVisionAlgNode(void);

  protected:
   /**
    * \brief main node thread
    *
    * This is the main thread node function. Code written here will be executed
    * in every node loop while the algorithm is on running state. Loop frequency
    * can be tuned by modifying loop_rate attribute.
    *
    * Here data related to the process loop or to ROS topics (mainly data structs
    * related to the MSG and SRV files) must be updated. ROS publisher objects
    * must publish their data in this process. ROS client servers may also
    * request data to the corresponding server topics.
    */
    void mainNodeThread(void);

   /**
    * \brief dynamic reconfigure server callback
    *
    * This method is called whenever a new configuration is received through
    * the dynamic reconfigure. The derivated generic algorithm class must
    * implement it.
    *
    * \param config an object with new configuration from all algorithm
    *               parameters defined in the config file.
    * \param level  integer referring the level in which the configuration
    *               has been changed.
    */
    void node_config_update(Config &config, uint32_t level);

   /**
    * \brief node add diagnostics
    *
    * In this abstract function additional ROS diagnostics applied to the
    * specific algorithms may be added.
    */
    void addNodeDiagnostics(void);

    // [diagnostic functions]

    // [test functions]

   /**
    * \brief state machine
    *
    * State Machine. After the start the idea is to iterate between the different states
    * SEARCH_QR -> DECODE_QR -> MOVEMENT -> MOVEMENT_ERROR_COMPROBATION -> SEARCH_QR
    */
    void state_machine(void);

    /**
     * \brief init walk module
     *
     * Function which initialize all the walk parameters of Darwin from the .cfg file
     */
    void init_walk_module(void);

    /**
     * \brief init head tracking module
     *
     * Function which initialize all the head tracking parameters of Darwin from the .cfg file
     */
     void init_headt_module(void);

    /**
     * \brief obtain a radian angle with a tag_id
     *
     * Function to get the angle and the direction from a string
     */
     void setPanFromQR_id();

    /**
     * \brief obtain the magnitude to get in normalized radians
     *
     * Function to get the yaw_mgn to move, where
     * alpha is the start angle and beta the final angle
     */
     double get_magnitude(double alpha, double beta);

     /**
      * \brief obtain the goal to get in normalized radians
      *
      * Function to get the yaw_goal to achieve, where
      * alpha is the angle desired to move and beta the actual yaw
      */
      double get_goal(double alpha, double beta);

      /**
       * \brief saturate the value between its limits
       *
       */
       double saturate(double alpha);
};

#endif
