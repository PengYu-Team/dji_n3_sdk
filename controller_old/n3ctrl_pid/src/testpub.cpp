//头文件
#include <ros/ros.h>
#include <iostream>
#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>

using namespace std;

bool init_ok = false;
string object_name;
Eigen::Vector3d pos_now, pos_last, vel_now , att_rate;
geometry_msgs::Quaternion att_now;
Eigen::Vector3d vel_0, vel_1, vel_2, vel_3, vel_4, vel_filter;
nav_msgs::Odometry Drone_odom;

ros::Time time_now;
ros::Time time_pos_now, time_pos_last;

ros::Publisher odom_pub;


//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "testpub");
    ros::NodeHandle nh("~");

    nh.param<string>("object_name", object_name, "dji_n3");

    // 将上述信息整合为里程计信息发布，用于位置环控制
    odom_pub = nh.advertise<geometry_msgs::TwistStamped>("/vrpn_client_node/"+ object_name + "/twist", 10);

    // 频率
    ros::Rate rate(100.0);

    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout<<setprecision(2);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

int pub_msg = 0;
geometry_msgs::TwistStamped test;
    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Main Loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while (ros::ok())
    {
        //回调一次 更新传感器状态
        ros::spinOnce();
            
        odom_pub.publish(test);

        pub_msg++;

        cout << "\033[1;31m" << " ----> pub_msg:  " << pub_msg << " [m]. " << "\033[0m" << endl;

        rate.sleep();
    }

    return 0;
}