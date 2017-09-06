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

#ifndef _ceabot_maze_alg_node_h_
#define _ceabot_maze_alg_node_h_

#include <iri_base_algorithm/iri_base_algorithm.h>
#include "ceabot_maze_alg.h"

#include <humanoid_modules/walk_module.h>
#include <humanoid_common_msgs/buttons.h>
#include <humanoid_modules/head_tracking_module.h>
#include <humanoid_modules/action_module.h>
#include <iri_ros_tools/timeout.h>


#define PI 3.1415926535
#define ERROR 0.12

// [publisher subscriber headers]
#include <std_msgs/Int8.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <humanoid_common_msgs/tag_pose_array.h>
#include <string>
#include <map>
#include <utility>
#include <iostream>
#include <algorithm>
#include <vector>
#include <map>
#include <math.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>


// [service client headers]
#include <humanoid_common_msgs/set_qr_size.h>

// [action server client headers]

/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */

 typedef enum {IDLE,
               SCAN_MAZE,
               WAIT_FOR_SCAN,
               PROCESS_DATA,
               SEARCH_FOR_GOAL_QR,
               CALCULATE_MOVEMENT_FOR_GOAL_QR,
               MOVEMENT_ALPHA_FOR_GOAL_QR,
               CHECK_ALPHA_GOAL_MFGQR,
               MOVEMENT_X_FOR_GOAL_QR,
               CHECK_GOAL_MFGQR,
               STRAIGHT_FORWARD,
               CHECK_STRAIGHT_FORWARD,
               FIND_HOLES,
               CALCULATE_MOVEMENT,
               MOVEMENT_ALPHA,
               MOVEMENT_X,
               CHECK_GOAL_ALPHA,
               CHECK_GOAL_XY,
               FALLEN_DARWIN,
               IS_DARWIN_STANDING,
               CHECK_NORTH,
               STOP_SPINNING_NORTH,
               CHECK_STOP_SPINNING_NORTH,
               CHECK_SOUTH,
               STOP_SPINNING_SOUTH,
               CHECK_STOP_SPINNING_SOUTH,
               START_WAITING1,
               WAIT1,
               START_WAITING2,
               WAIT2,
               WAIT_TRACKING,
               TURN_TO_SOUTH,
               TURN_TO_NORTH
               } darwin_states;

struct position {
  double x; double y; double z;
};

struct orientation {
  double x; double y; double z; double w;
};

struct DDPOINT {
  double x; double z;
};

struct qr_info {
  std_msgs::Header header;
  std::string qr_tag; position pos; orientation ori;
};

typedef std::pair<DDPOINT, double> hole;

class CeabotMazeAlgNode : public algorithm_base::IriBaseAlgorithm<CeabotMazeAlgorithm>
{
  private:
    // [publisher attributes]

    // [subscriber attributes]
    ros::Subscriber buttons_subscriber_;
    void buttons_callback(const humanoid_common_msgs::buttons::ConstPtr& msg);
    pthread_mutex_t buttons_mutex_;
    void buttons_mutex_enter(void);
    void buttons_mutex_exit(void);
    
    ros::Subscriber fallen_state_subscriber_;
    void fallen_state_callback(const std_msgs::Int8::ConstPtr& msg);
    pthread_mutex_t fallen_state_mutex_;
    void fallen_state_mutex_enter(void);
    void fallen_state_mutex_exit(void);

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
    ros::ServiceClient set_qr_size_client_;
    humanoid_common_msgs::set_qr_size set_qr_size_srv_;


    // [action server attributes]

    // [action client attributes]

   /**
    * \brief config variable
    *
    * This variable has all the driver parameters defined in the cfg config file.
    * Is updated everytime function config_update() is called.
    */
    bool event_start;
    darwin_states darwin_state; //Actual
    darwin_states state_stn;
    darwin_states state_sts;
    bool half_maze_achieved;

    bool search_started;
    bool goal_achieved;
    bool searching_for_qr;
    bool wall_qr_goal_found;
    bool first_bno_lecture;
    bool first_lecture;
    bool get_wall_qr;

    double pan_angle;
    double goal_x;
    double goal_y;
    double current_pan_angle;
    double tilt_angle;
    double current_tilt_angle;
    double current_angle_travelled;
    double nm_x;
    double nm_alpha;
    double bno055_measurement;
    double odom_x;
    double odom_y;
    double odom_xpre_fall;
    double odom_ypre_fall;
    double mov_alpha_goal;
    double mov_x_goal;
    double mov_y_goal;
    double north_of_the_maze;
    double south_of_the_maze;
    int    turn_left;
    int    fallen_state;
    int    direction;
    int    way_zaxis_grow;
    int    way_xaxis_grow;
    double next_x_mov;
    double next_z_mov;
    double ini_x;
    double ini_z;
    int    goalqr_vecpos;
    double goalqr_x;
    double goalqr_z;
    double str_forward_dis;
    CROSTimeout timeout1;
    CROSTimeout timeout2;
    tf::TransformListener listener;

    std::map<std::string, std::vector<qr_info> > qr_info_pre_processing;
    std::vector<qr_info> qr_information;
    std::vector<std::pair <DDPOINT, double> > holes_magn;

    Config config_;
    CWalkModule walk;
    CHeadTrackingModule tracking_module;
    CActionModule action;

  public:
   /**
    * \brief Constructor
    *
    * This constructor initializes specific class attributes and all ROS
    * communications variables to enable message exchange.
    */
    CeabotMazeAlgNode(void);

   /**
    * \brief Destructor
    *
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
    ~CeabotMazeAlgNode(void);

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

      void scan_maze(void);

      void wait_for_scan(void);

      void calculate_next_move(void);

      void darwin_movement_alpha(double alpha);

      void darwin_movement_x(double x);

      void check_goal_alpha(double goal);

      void check_goal_xy(double goalx, double goaly);

      double DegtoRad(double degree);

      double get_magnitude_alpha (double alpha, double beta);

      double get_magnitude_x (double x, double ini);

      double get_goal_alpha (double alpha, double beta);

      std::pair <double,double> get_goal_xy (double x, double inix, double iniy, double alpha);

      double saturate_alpha (double alpha);

      double saturate_movement (double x);

      void search_for_goal_qr (void);

      static bool distance_sort (qr_info o, qr_info p);

      void find_holes(void);

      double is_hole(qr_info* obs1, qr_info* obs2);

      bool is_wall(qr_info* obs);

      bool is_goal_wall(qr_info* obs);

      DDPOINT calculate_point_to_move(qr_info* obs1, qr_info* obs2);

      void get_immediate_obs (int i, qr_info& obs1, qr_info &obs2);

      void fill_PoseStamped (const humanoid_common_msgs::tag_pose &in, geometry_msgs::PoseStamped &out);

      bool straight_to_north ();

      bool straight_to_south ();

      double distance_to_xy (double x, double y);

      void process_data(void);

      bool ddpoint_goet (DDPOINT a, DDPOINT b, bool n);

      bool wall_too_far (qr_info wall, qr_info obs);

      bool NS_orientation (qr_info obs);

      std::pair<std::string, int> divide_qr_tag (std::string qr_tag);

      geometry_msgs::PoseStamped get_PoseStamped (qr_info* obs1);

      void calculate_movement_for_goal_qr ();

      void movement_alpha_for_goal_qr ();

      void check_alpha_goal_mfgqr ();

      void movement_x_for_goal_qr ();

      void check_goal_mfgqr ();

      void straight_forward ();

      void check_straight_forward();

    // [diagnostic functions]

    // [test functions]
};

#endif
