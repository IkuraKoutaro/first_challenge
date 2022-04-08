#ifndef FIRST_CHALLENGE_H
#define FIRST_CHALLENGE_H

#include <ros/ros.h>
#include "roomba_500driver_meiji/RoombaCtrl.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/tf.h"

class FirstChallenge
{
    public:
        FirstChallenge();
        void process();
        int i;
        int j;

    private:
        void odometry_callback(const nav_msgs::Odometry::ConstPtr&);
        void laser_callback(const sensor_msgs::LaserScan::ConstPtr&);

        void run();
        void run_2();
        void show_odom();
        bool show_scan();
        void VetQuaternionMsg(double roll, double pitch, double yaw, geometry_msgs::Quaternion &q);

        double odometry_x;
        double odometry_y;
        double yaw;
        double old_yaw;
        double r;
        double p;
        double y;
        double count;

        double range_min;
        double range_upper_limit;
        int range_size;
        int hz_;

        nav_msgs::Odometry odometry_;
        sensor_msgs::LaserScan laser_;
        roomba_500driver_meiji::RoombaCtrl cmd_vel_;

        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        ros::Subscriber sub_odom_;
        ros::Subscriber sub_laser_;
        ros::Publisher pub_cmd_vel_;


};

#endif
