#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <termio.h>

int scanKeyboard()
{
    int in;
    struct termios new_settings;
    struct termios stored_settings;
    tcgetattr(STDIN_FILENO,&stored_settings); //获得stdin 输入
    new_settings = stored_settings;           //
    new_settings.c_lflag &= (~ICANON);        //
    new_settings.c_cc[VTIME] = 0;
    tcgetattr(STDIN_FILENO,&stored_settings); //获得stdin 输入
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(STDIN_FILENO,TCSANOW,&new_settings); //

    in = getchar();

    tcsetattr(STDIN_FILENO,TCSANOW,&stored_settings);
    return in;
}


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "keyboard");
    ros::NodeHandle node_handle;

    /* Msg Publisher */
    ros::Publisher key_msg_puber = node_handle.advertise<std_msgs::Int32>("arm_keys", 100);

    /* Multi threading */
    ros::AsyncSpinner spinner(1); // single thread
    spinner.start();

    ROS_INFO_NAMED("arm_log", "keyboard", "start binding keyboard\r\n");

    while(ros::ok())
    {
        std_msgs::Int32 res;
        res.data = scanKeyboard();
        printf(":%d\r\n",res);
        
        key_msg_puber.publish(res);
        ros::spinOnce();
    }

    ros::waitForShutdown();
}