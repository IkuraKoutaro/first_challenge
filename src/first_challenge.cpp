#include "first_challenge/first_challenge.h"
#include <math.h>

FirstChallenge::FirstChallenge():private_nh_("~")
{
    private_nh_.param("hz_", hz_, {10});
    sub_odom_ = nh_.subscribe("/roomba/odometry", 1, &FirstChallenge::odometry_callback, this);
    sub_laser_ = nh_.subscribe("/scan", 1, &FirstChallenge::laser_callback, this);
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
    cmd_vel_.cntl.linear.x = 0.05;
    pub_cmd_vel_.publish(cmd_vel_);
}

void VetQuaternionMsg(double roll, double pitch, double yaw, geometry_msgs::Quaternion &q){
    tf::Quaternion quat=tf::createQuaternionFromRPY(roll,pitch,yaw);
    quaternionTFToMsg(quat,q);
}

void FirstChallenge::run_2()    //回転
{
    cmd_vel_.mode = 11;
    cmd_vel_.cntl.angular.z = M_PI/8;
    pub_cmd_vel_.publish(cmd_vel_);
}

void FirstChallenge::show_odom()        //ルンバの速度と位置がわかる
{
    // ROS_INFO_STREAM("odom: x: %f, y: %f, z: %f", odometry_.pose.pose.position.x, odometry_.pose.pose.position.y, odometry_.pose.pose.position.z);
    //std::cout << "odom" << ": x:" << odometry_.pose.pose.position.x
        //<< " y:" <<  odometry_.pose.pose.position.y
        //<< " z:" <<  odometry_.pose.pose.position.z << std::endl;
}

bool FirstChallenge::show_scan()
{
    double sum = 0;
    int num = 10;
    double ave = 0;

    range_size = laser_.ranges.size();

    for(int i=range_size/2 - num; i<range_size/2 + num; i++){
        sum += laser_.ranges[i];
    }

    ave = sum / (num*2);

    if(ave<0.5){
        return false;
    }


    // range_min = 1e6;                              //下から1番目　1e6=1000000
    // range_upper_limit = 0;                        //上限値
    // range_size = laser_.ranges.size();
    //
    // std::cout<< "range_size = " << range_size << std::endl;
    //
    // for (int i = 0; i < range_size; i++) {    //.sizeは配列の個数
    //     if (laser_.ranges[i] < range_min) {
    //         range_min = laser_.ranges[i];
    //     }
    // }
    //
    // std::cout << "a" << std::endl;
    //
    // range_upper_limit = range_min + 0.35;
    //
    // std::cout << "upper_limit = " << range_upper_limit << std::endl;
    // range_min = 1e6;
    // double min = 1e6;
    //
    // for (int j = 0; j < range_size; j++){      //上限値以上の最小値を求める
    //     if( (laser_.ranges[j] > range_upper_limit) && (laser_.ranges[j] < range_min) ){
    //         //range_min = laser_.ranges[j];
    //         if(laser_.ranges[j]<min){
    //             min = laser_.ranges[j];
    //         }
    //     }
    // }
    //
    // std::cout << "b" << std::endl;
    // std::cout << "scan: min:" << min << std::endl; //std::coutターミナルに表示
    //
    // if(min <= 0.50){
    //     cmd_vel_.cntl.linear.x = 0.0;
    //     return false;
    // }
    // // ROS_INFO_STREAM("scan: min: %f", range_min);
    return true;
}

void FirstChallenge::process()
{
    ros::Rate loop_rate(hz_);   //ループ頻度の設定　最後にRate::sleep()を呼び出してからどれだけ経過したか常に管理し、正確な時間になるまでスリープする。

    //while(ros::ok())無限ループ
    //ros::ok()はノードの終了指示が与えられたときにループを抜けて終了処理を行う。
    ros::spinOnce();

    while(sqrt((odometry_.pose.pose.position.x * odometry_.pose.pose.position.x)+(odometry_.pose.pose.position.y * odometry_.pose.pose.position.y))<= 1.0){
        run();
        ros::spinOnce();
    }
    cmd_vel_.cntl.linear.x = 0.0;
    pub_cmd_vel_.publish(cmd_vel_);     //nodeに情報送る(roomba)

    count = 0;

    while(!(count>0 && y>=0)){
        tf::Quaternion quat(odometry_.pose.pose.orientation.x, odometry_.pose.pose.orientation.y, odometry_.pose.pose.orientation.z, odometry_.pose.pose.orientation.w);
        tf::Matrix3x3(quat).getRPY(r, p, y);

        run_2();
        ros::spinOnce();
        if(y<(-M_PI/2)){
            count++;
        }
        loop_rate.sleep();  //ros::Rateオブジェクトをhz_の発信で行えるように残り時間をスリープするために使う。
        std::cout<<"yaw="<<y<<std::endl;
    }
    cmd_vel_.cntl.angular.z = 0.0;
    cmd_vel_.cntl.linear.x = 0.05;
    pub_cmd_vel_.publish(cmd_vel_);

    std::cout << "ikura" << std::endl;

    bool flag = true;

    while(flag){
        flag = show_scan();      //laser min data入手
        ros::spinOnce();
        loop_rate.sleep();
    }
    cmd_vel_.cntl.linear.x = 0.0;
    cmd_vel_.cntl.angular.z = 0.0;
    pub_cmd_vel_.publish(cmd_vel_);
    std::cout << "finish" <<std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "first_challenge");   //ros::init ROSの初期化　また、ノードの名前を特定する箇所でもある。ノードの名前はシステム中で唯一無二でなければならない。
    FirstChallenge first_challenge;             //インスタンスの生成
    first_challenge.process();                  //メンバ関数(process())を呼び出す
    ros::spin();                                //spin?
    return 0;
}
