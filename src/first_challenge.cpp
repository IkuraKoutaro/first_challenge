#include "first_challenge/first_challenge.h"
#include <math.h>

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

void FirstChallenge::run()          //直進
{
    cmd_vel_.mode = 11;
    //std::cout<<odometry_.pose.pose.position.x<<","<<odometry_.pose.pose.position.y<<std::endl;
    cmd_vel_.cntl.linear.x = 0.05;
    pub_cmd_vel_.publish(cmd_vel_);     //nodeに情報送る(roomba)
}

void FirstChallenge::run_2(double* theta)    //回転
{
    cmd_vel_.mode = 11;
    cmd_vel_.cntl.angular.z = M_PI/4;
    pub_cmd_vel_.publish(cmd_vel_);

    double q0q3 = odometry_.pose.pose.orientation.x * odometry_.pose.pose.orientation.w;
    double q1q2 = odometry_.pose.pose.orientation.y * odometry_.pose.pose.orientation.z;
    double q0q0 = odometry_.pose.pose.orientation.x * odometry_.pose.pose.orientation.x;
    double q1q1 = odometry_.pose.pose.orientation.y * odometry_.pose.pose.orientation.y;
    double q2q2 = odometry_.pose.pose.orientation.z * odometry_.pose.pose.orientation.z;
    double q3q3 = odometry_.pose.pose.orientation.w * odometry_.pose.pose.orientation.w;

    *theta = atan((2*(q0q3 + q1q2))/(q0q0 + q1q1 - q2q2 - q3q3));

    std::cout<<theta<<std::endl;

    //theta += atan((odometry_.pose.pose.position.x - odometry_x)/(odometry_.pose.pose.position.y - odometry_y));
    //odometry_x = odometry_.pose.pose.position.x;
    //odometry_y = odometry_.pose.pose.position.y;*/

}

void FirstChallenge::show_odom()        //ルンバの速度と位置がわかる
{
    // ROS_INFO_STREAM("odom: x: %f, y: %f, z: %f", odometry_.pose.pose.position.x, odometry_.pose.pose.position.y, odometry_.pose.pose.position.z);
    //std::cout << "odom" << ": x:" << odometry_.pose.pose.position.x
        //<< " y:" <<  odometry_.pose.pose.position.y
        //<< " z:" <<  odometry_.pose.pose.position.z << std::endl;
}

void FirstChallenge::show_scan()
{
    range_min = 1e6;                              //下から1番目　1e6=1000000
    range_upper_limit = 0;                        //上限値
    range_size = laser_.ranges.size();
    double laser_ranges[range_size];            //
    for (int i = 0; i < laser_.ranges.size(); i++) {    //.sizeは配列の個数
        if (laser_ranges[i] < range_min) {
            range_min = laser_ranges[i];
        }
    }

    range_upper_limit = range_min + 0.1;

    range_min = 1e6;

    for (int j = 0; j < laser_.ranges.size(); j++){      //上限値以上の最小値を求める
        if( (laser_ranges[i] > range_upper_limit) && (laser_ranges[i] < range_min) ){
            range_min = laser_ranges[i];
        }
    }

    if(range_min <= 1.00){
        cmd_vel_.cntl.linear.x = 0.0;
    }
    // ROS_INFO_STREAM("scan: min: %f", range_min);
    std::cout << "scan: min:" << range_min << std::endl; //std::coutターミナルに表示
    //endl 改行
}

void FirstChallenge::process()
{
    ros::Rate loop_rate(hz_);   //ループ頻度の設定　最後にRate::sleep()を呼び出してからどれだけ経過したか常に管理し、正確な時間になるまでスリープする。
    while(ros::ok())        //while(ros::ok())無限ループ
        //ros::ok()はノードの終了指示が与えられたときにループを抜けて終了処理を行う。
    {
        ros::spinOnce();
        while(sqrt((odometry_.pose.pose.position.x * odometry_.pose.pose.position.x)+(odometry_.pose.pose.position.y * odometry_.pose.pose.position.y))<= 1.0){
            run();            //run
            ros::spinOnce();
        }

        std::cout << "a" << std::endl;
        cmd_vel_.cntl.linear.x = 0.0;
        pub_cmd_vel_.publish(cmd_vel_);     //nodeに情報送る(roomba)

        while(theta <= 2*M_PI){
            run_2(&theta);
            ros::spinOnce();
        }
        cmd_vel_.cntl.angular.z = 0.0;
        pub_cmd_vel_.publish(cmd_vel_);

        show_scan();        //laser min data入手
        //ros::spinOnce();    //spinOnce()
        loop_rate.sleep();  //ros::Rateオブジェクトをhz_の発信で行えるように残り時間をスリープするために使う。
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "first_challenge");   //ros::init ROSの初期化　また、ノードの名前を特定する箇所でもある。ノードの名前はシステム中で唯一無二でなければならない。
    FirstChallenge first_challenge;             //インスタンスの生成
    first_challenge.process();                  //メンバ関数(process())を呼び出す
    ros::spin();                                //spin?
    return 0;
}
