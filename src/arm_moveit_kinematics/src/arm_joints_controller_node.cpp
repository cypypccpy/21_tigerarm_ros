#include <moveit/move_group_interface/move_group_interface.h>
#include <sensor_msgs/JointState.h>
#include <arm_moveit_kinematics/arm_moveit_kinermatics_node.h>

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/Int32.h>
#include "arm_moveit_kinematics/keyboards.h"

using namespace arm_keys;

static ros::Publisher* g_joints_puber_ptr = nullptr;
static serial::Serial* g_serial_port_ptr = nullptr;

static uint8_t g_serial_buffer[100] = {'a','b','c'};
static uint8_t g_serial_buffer2[2] = {0, 0};

static uint8_t g_test_recvbuff[4];
static float joint_position_buffer[6];
static int arm_states_buffer[2];
static int joint_direction[6] = {1,1,-1,1,1,1};

static bool arm_states_recv;
static int in;

void convert(void* a, int cnt)
{
    uint8_t temp;
    for(int i = 0; i < cnt; i++){
        temp = ((uint8_t*)a)[cnt - i - 1];
        ((uint8_t*)a)[cnt - i - 1] = ((uint8_t*)a)[i];
        ((uint8_t*)a)[i] = temp;
    }
}
/*
void interpolation(const sensor_msgs::JointStateConstPtr& msg1, const sensor_msgs::JointStateConstPtr& msg2) {
    for(size_t i = 0; i < 6; i++){
        msg1->position[i] * joint_direction[i] + msg1->position[i] * joint_direction[i];
    }
}
*/

int recv_keyboard() {
    memcpy(&in, &g_test_recvbuff, 4); //float32-->4Bytes

    return in;
}

void arm_states_recv_callback(const arm_moveit_kinematics::arm_statesConstPtr& states) {
    arm_states_buffer[0] = states->air_pump_close;
    arm_states_buffer[1] = states->execute_finished;

    arm_states_recv = true;
}

void joint_states_recv_callback(const sensor_msgs::JointStateConstPtr& msg)
{
    arm_moveit_kinematics::joint_plots send_msg;
    send_msg.p_joint0 = int32_t(msg->position[0] * 10000);
    send_msg.p_joint1 = int32_t(msg->position[1] * 10000);
    send_msg.p_joint2 = int32_t(msg->position[2] * 10000);
    send_msg.p_joint3 = int32_t(msg->position[3] * 10000);
    send_msg.p_joint4 = int32_t(msg->position[4] * 10000);
    send_msg.p_joint5 = int32_t(msg->position[5] * 10000);

    /* 准备替换成ros::serialization */
    //ros::serialization::serialize()
    if (arm_states_recv) {
        for(size_t i = 0; i < 6; i++){
            joint_position_buffer[i] = msg->position[i] * joint_direction[i];
            memcpy(&g_serial_buffer[i*4], &joint_position_buffer[i], 4); //float32-->4Bytes
        }
        for(size_t i = 0; i < 2; i++){
            memcpy(&g_serial_buffer[1*i + 24], &arm_states_buffer[i], 1); //int8-->1Bytes
        }
        std::cout << arm_states_recv  << std::endl;
        if(g_serial_port_ptr){
            g_serial_port_ptr->write(g_serial_buffer, 1*2 + 6*4);
        }
        arm_states_recv = false;
    }

    else {
        for(size_t i = 0; i < 6; i++){
            joint_position_buffer[i] = msg->position[i] * joint_direction[i];
            memcpy(&g_serial_buffer[i*4], &joint_position_buffer[i], 4); //float32-->4Bytes
        }

        if(g_joints_puber_ptr){
            g_joints_puber_ptr->publish(send_msg);
        }
        if(g_serial_port_ptr){
            g_serial_port_ptr->write(g_serial_buffer, 6*4);
        }
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "arm_joints_controller");
    ros::NodeHandle node_handle;

    ros::Subscriber joints_suber = node_handle.subscribe("joint_states", 100, joint_states_recv_callback);
    ros::Subscriber arm_suber = node_handle.subscribe("arm_states", 100, arm_states_recv_callback);
    ros::Publisher joints_puber = node_handle.advertise<arm_moveit_kinematics::joint_plots>("joint_plots", 100);
    ros::Publisher key_msg_puber = node_handle.advertise<std_msgs::Int32>("arm_keys", 100);
    ros::AsyncSpinner spinner(2); // two thread

    serial::Serial serial_port;
    serial::Timeout serial_timeout = serial::Timeout::simpleTimeout(100); //丢包时间100ms
    std::string port_name = "/dev/ttyUSB1";
    serial_port.setPort(port_name);
    serial_port.setBaudrate(115200);
    serial_port.setTimeout(serial_timeout);

    try{
        serial_port.open();
    }catch(serial::IOException& e)
    {
        ROS_ERROR_NAMED("arm_log", "Unable to open serial port %s", port_name.c_str());
    }

    if(serial_port.isOpen())
        ROS_INFO_NAMED("arm_log", "Successfully open serial port %s", port_name.c_str());

    g_joints_puber_ptr = &joints_puber;
    g_serial_port_ptr = &serial_port;
    
    //spinner.start();

    ROS_INFO_NAMED("arm_log", "Joints plot data publisher start");
    
    ros::Rate loop_rate(500);   //500Hz
    while(ros::ok())
    {
        size_t recv_size = serial_port.available();
        if(recv_size > 0){
            recv_size = serial_port.read(g_test_recvbuff, recv_size);
            std_msgs::Int32 res;

            res.data = recv_keyboard();

            key_msg_puber.publish(res);
        }
        ros::spinOnce();
    }
    serial_port.close();
    ros::waitForShutdown();
}