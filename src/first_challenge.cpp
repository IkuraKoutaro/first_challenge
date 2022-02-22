#include "first_challenge/first_challenge.h"
#define M_PI 3.14159

FirstChallenge::FirstChallenge():private_nh_("~")
{
    private_nh_.param("hz_", hz_, {10});
    sub_odom_ = nh_.subscribe("/roomba/odometry", 100, &FirstChallenge::odometry_callback, this);
    sub_laser_ = nh_.subscribe("/scan", 100, &FirstChallenge::laser_callback, this);
    pub_cmd_vel_ = nh_.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control", 1);
}

void FirstChallenge::odometry_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odometry_ = *msg;
}

void FirstChallenge::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laser_ = *msg;
}

void FirstChallenge::run()
{
    cmd_vel_.mode = 11;
    cmd_vel_.cntl.linear.x = 0.5;
    ros::Duration(2.0).sleep();
    cmd_vel_.cntl.angular.z = M_PI;
    ros::Duration(2.0).sleep();

    pub_cmd_vel_.publish(cmd_vel_);//nodeに情報送る(roomba)
}

void FirstChallenge::show_odom()//ルンバの速度と位置がわかる
{
    // ROS_INFO_STREAM("odom: x: %f, y: %f, z: %f", odometry_.pose.pose.position.x, odometry_.pose.pose.position.y, odometry_.pose.pose.position.z);
    std::cout << "odom" << ": x:" << odometry_.pose.pose.position.x << " y:" <<  odometry_.pose.pose.position.y << " z:" <<  odometry_.pose.pose.position.z << std::endl;
}

void FirstChallenge::show_scan()//
{
    float range_min = 1e6;      //下から1番目　1e6=1000000
    float range_min_2 = 1e6;    //下から2番目
    for (int i = 0; i < laser_.ranges.size(); i++) {    //.sizeは配列の個数
        if (laser_.ranges[i] < range_min) {
            range_min_2 = range_min;
            range_min = laser_.ranges[i];
        } else if (laser_.ranges[i] < range_min_2) {
            range_min_2 = laser_.ranges[i];
        }
    }
    // ROS_INFO_STREAM("scan: min: %f", range_min);
    std::cout << "scan: min:" << range_min << std::endl; //std::coutターミナルに表示
}

void FirstChallenge::process()
{
    ros::Rate loop_rate(hz_);
    while(ros::ok())
    {
        run();
        show_odom();
        show_scan();

        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "first_challenge");
    FirstChallenge first_challenge;
    first_challenge.process();
    ros::spin();
    return 0;
}
