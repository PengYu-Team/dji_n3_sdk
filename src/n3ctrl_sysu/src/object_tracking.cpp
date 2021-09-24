#include <ros/ros.h>
#include <Eigen/Eigen>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include "prometheus_msgs/MultiDetectionInfo.h"
#include "printf_utils.h"

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
string object_name;
bool hold_mode; // 悬停模式，用于测试检测精度
Eigen::Vector3d object_pos_body,object_pos_enu;
double dis_to_object;
//---------------------------------------Drone---------------------------------------------
Eigen::Vector3d uav_pos;
Eigen::Vector3d camera_offset;
double uav_yaw;
//---------------------------------------Track---------------------------------------------
quadrotor_msgs::PositionCommand cmd;
int num_count_vision_lost;
int num_count_vision_get;

// 五种状态机
enum EXEC_STATE
{
    WAITING_RESULT,
    TRACKING,
    LANDING,
};
EXEC_STATE exec_state;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void detection_cb_real(const prometheus_msgs::MultiDetectionInfoConstPtr &msg)
{
    if(num_count_vision_lost > 10)
    {
        cout << RED << "  [ lost target ]."  << TAIL <<endl;
    }

    if(msg->num_objs == 0)
    {
        num_count_vision_lost++;
        num_count_vision_get = 0;
        return;
    }

    if(msg->num_objs != 1)
    {
        cout << RED << "ERROR: detect more than one objects!"  << TAIL <<endl;
        return;
    }
    
    if(msg->detection_infos[0].detected)
    {
        num_count_vision_get++;
        num_count_vision_lost = 0;
    }

    //  方向定义： 识别算法发布的目标位置位于相机坐标系（从相机往前看，物体在相机右方x为正，下方y为正，前方z为正）
    // 计算目标在机体系下的位置

    // 相机坐标系 -> 机体系
    object_pos_body[0] = -(uav_pos[2] + camera_offset[2]) * tan(msg->detection_infos[0].sight_angle[1]) + camera_offset[0];
    object_pos_body[1] = -(uav_pos[2] + camera_offset[2])  * tan(msg->detection_infos[0].sight_angle[0]) + camera_offset[1];
    object_pos_body[2] = - uav_pos[2];
    
    // 乘以无人机的偏航角 机体系->以机体为原点，x轴正方向指向惯性系x轴的坐标系
    Eigen::Vector3d enu_frame;
    enu_frame[0] = object_pos_body[0] * cos(uav_yaw) - object_pos_body[1] * sin(uav_yaw);
    enu_frame[1] = object_pos_body[0] * sin(uav_yaw) + object_pos_body[1] * cos(uav_yaw);
    enu_frame[2] = object_pos_body[2];

    cout << GREEN << " Case2 Detection enu_frame   ["<< enu_frame[0] <<", "<<enu_frame[1]<<", "<<enu_frame[2]<<" ]."  << TAIL <<endl;

    // 最后转换到惯性系
    object_pos_enu = enu_frame + uav_pos;

    cout << GREEN << " Case2 Detection object_pos_enu   ["<< object_pos_enu[0] <<", "<<object_pos_enu[1]<<", "<<object_pos_enu[2]<<" ]."  << TAIL <<endl;

    dis_to_object = sqrt( object_pos_body[0] * object_pos_body[0] + object_pos_body[1] * object_pos_body[1] );
}

void mocap_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    uav_pos << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    uav_yaw = 0.0;
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_tracking");
    ros::NodeHandle nh("~");

    // 节点运行频率： 20hz 【视觉端解算频率大概为20HZ】
    ros::Rate rate(20.0);
    
    nh.param<string>("object_name", object_name, "dji_n3");
    // 悬停模式 - 仅用于观察检测结果
    nh.param("hold_mode", hold_mode, true);
    // 相机安装偏移,规定为:相机在机体系(质心原点)的位置
    nh.param("camera_offset_x", camera_offset[0], 0.0);
    nh.param("camera_offset_y", camera_offset[1], 0.0);
    nh.param("camera_offset_z", camera_offset[2], 0.0);
    cout << YELLOW << camera_offset[0]  << TAIL <<endl; 

    ros::Subscriber mocap_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/"+ object_name + "/pose", 1, mocap_pos_cb);

    ros::Subscriber detection_sub_real = nh.subscribe<prometheus_msgs::MultiDetectionInfo>("/prometheus/object_detection/yolo", 1, detection_cb_real);

    ros::Publisher command_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/dji_n3ctrl/point_cmd", 1);

    // 等待
    ros::Duration(3.0).sleep();

    exec_state = EXEC_STATE::WAITING_RESULT;

    while (ros::ok())
    {
        //回调
        ros::spinOnce();

        static int printf_num = 0;
        printf_num++;
        // 此处是为了控制打印频率
        if(printf_num > 20)
        {
            if(exec_state == TRACKING)
            {
                // 正常追踪
                cout << GREEN << " Tracking, distance to target :    "<< dis_to_object << " [m] .."<< TAIL <<endl; 
            }            
            printf_num = 0;
        }

        switch (exec_state)
        {
            // 初始状态，等待视觉检测结果
            case WAITING_RESULT:
                if(num_count_vision_get > 10)
                {
                    exec_state = TRACKING;
                    cout << GREEN << " Get target info, Sart to tracking.."<< TAIL <<endl; 
                    break;
                }
                cout << GREEN << " Waiting for the detection result.."<< TAIL <<endl; 
                ros::Duration(1.0).sleep();
                break;
            // 追踪状态
            case TRACKING:

                cmd.header.stamp = ros::Time::now();
                cmd.header.frame_id = "world";
                cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
                cmd.trajectory_id = cmd.trajectory_id + 1;

                cmd.position.x = object_pos_enu[0];
                cmd.position.y = object_pos_enu[1];
                cmd.position.z = 1.0;

                command_pub.publish(cmd);

                break;

            case LANDING:

                break;
            
        }
        rate.sleep();
    }

    return 0;

}