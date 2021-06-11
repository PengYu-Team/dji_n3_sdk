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

int get_msg = 0;

void mocap_vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    get_msg++;

    cout << "\033[1;32m" << " ----> get_msg:  " << get_msg << " [m]. " << "\033[0m" << endl;
}



//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "testsub");
    ros::NodeHandle nh("~");

    nh.param<string>("object_name", object_name, "dji_n3");

    // 通过vrpn_client_ros订阅来自动捕的消息（位置和姿态消息），并通过微分平滑滤波的方式估计线速度
    ros::Subscriber mocap_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/vrpn_client_node/"+ object_name + "/twist", 1, mocap_vel_cb);

    // 结论：
    // 假设发布是100Hz
    // 缓冲区如果设置为10，即使订阅是50Hz，也能收到100Hz的信息，即一次回调处理多次
    // 缓冲区如果设置为1，订阅是50hz，则只能收到50Hz

    // 频率
    ros::Rate rate(50.0);

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

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Main Loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while (ros::ok())
    {
        //回调一次 更新传感器状态
        ros::spinOnce();

        rate.sleep();
    }

    return 0;
}