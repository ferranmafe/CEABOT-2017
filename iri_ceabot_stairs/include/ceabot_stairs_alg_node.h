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

#ifndef _ceabot_stairs_alg_node_h_
#define _ceabot_stairs_alg_node_h_

#include <iri_base_algorithm/iri_base_algorithm.h>
#include "ceabot_stairs_alg.h"

#include <humanoid_modules/walk_module.h>
#include <humanoid_modules/head_tracking_module.h>
#include <humanoid_modules/action_module.h>
#include <humanoid_modules/stairs_module.h>
#include <iri_ros_tools/timeout.h>
#include <map>
#include <string>
#include <math.h>

#define PI 3.1415926535
#define ERROR 0.12
#define ERROR_PHASE_2 0.1
#define ERROR_GO_BACK 0.01
#define ERROR_TURN 0.01

// [publisher subscriber headers]
#include <humanoid_common_msgs/ir_foot_data.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
// [service client headers]

// [action server client headers]

/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */

 typedef enum {IDLE,
               START,
               WAIT_START_TIME,
               START_WALKING_PHASE_1,
               CHECK_WALKING_DIRECTION_PHASE_1,
               STOP_WALKING_PHASE_1,
               START_CLIMBING_STAIR,
               COMPLETE_CLIMBING_STAIR,
               START_WALKING_PHASE_2,
               CHECK_WALKING_DIRECTION_PHASE_2,
               START_WALKING_PHASE_3,
               CHECK_WALKING_DIRECTION_PHASE_3,
               STOP_WALKING_PHASE_3,
               CHECK_POSITION_FOR_DOWN_STAIR,
               START_GO_BACK,
               CHECK_GO_BACK,
               FINISH_GO_BACK,
               START_ANGLE_CORRECTION,
               CHECK_ANGLE_CORRECTION,
               FINISH_ANGLE_CORRECTION,
               DOWN_STAIR,
               COMPLETE_DOWN_STAIR,
               FINISH
               } darwin_states;

class CeabotStairsAlgNode : public algorithm_base::IriBaseAlgorithm<CeabotStairsAlgorithm>
{
  private:
    // [publisher attributes]

    // [subscriber attributes]
    ros::Subscriber right_foot_data_subscriber_;
    void right_foot_data_callback(const humanoid_common_msgs::ir_foot_data::ConstPtr& msg);
    pthread_mutex_t right_foot_data_mutex_;
    void right_foot_data_mutex_enter(void);
    void right_foot_data_mutex_exit(void);

    ros::Subscriber left_foot_data_subscriber_;
    void left_foot_data_callback(const humanoid_common_msgs::ir_foot_data::ConstPtr& msg);
    pthread_mutex_t left_foot_data_mutex_;
    void left_foot_data_mutex_enter(void);
    void left_foot_data_mutex_exit(void);

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
    darwin_states darwin_state;

    bool first_bno_lecture;

    int stairs_counter;

    double bno055_measurement;
    double straight_forward_direction;
    double odom_x;
    double odom_y;
    double initial_x_phase_2;
    double initial_y_phase_2;
    double initial_x_phase_3;
    double initial_y_phase_3;
    double initial_x_go_back;
    double initial_y_go_back;
    double initial_angle_to_correct;
    double turn_direction;

    std::map<std::string, bool> left_foot;
    std::map<std::string, bool> right_foot;

    Config config_;
    CStairsModule stairs;
    CWalkModule walk;
    CActionModule action;
    CHeadTrackingModule tracking_module;
    CROSTimeout timeout;
  public:
   /**
    * \brief Constructor
    *
    * This constructor initializes specific class attributes and all ROS
    * communications variables to enable message exchange.
    */
    CeabotStairsAlgNode(void);

   /**
    * \brief Destructor
    *
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
    ~CeabotStairsAlgNode(void);

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

    void state_machine(void);

    void start_function(void);

    void wait_start_time(void);

    void start_walking_phase_1(void);

    void check_walking_direction_phase_1(void);

    void stop_walking_phase_1(void);

    void start_climbing_stair(void);

    void complete_climbing_stair(void);

    void start_walking_phase_2(void);

    void check_walking_direction_phase_2(void);

    void start_walking_phase_3(void);

    void check_walking_direction_phase_3(void);

    void stop_walking_phase_3(void);

    void check_position_for_down_stair(void);

    void start_go_back(void);

    void check_go_back(void);

    void finish_go_back(void);

    void start_angle_correction(void);

    void check_angle_correction(void);

    void finish_angle_correction(void);

    void down_stair(void);

    void complete_down_stair(void);

    void finish(void);
//------------------------------------------------------------------------------
    void init_walk_module(void);

    void init_headt_module(void);

    double saturate_angle(double alpha);

//------------------------------------------------------------------------------

    // [diagnostic functions]

    // [test functions]
};

#endif
