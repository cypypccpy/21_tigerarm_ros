#pragma once
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

const double DP = 0.01;//d position
const double DA = 0.1; //d angle

struct arm_pose_delta
{
    double dx;
    double dy;
    double dz;
    double droll;
    double dpitch;
    double dyaw;
    geometry_msgs::Pose add(geometry_msgs::Pose current)
    {
        geometry_msgs::Pose target = current;
        tf::Quaternion target_q, current_q;
        tfScalar r, p, y;
        tf::quaternionMsgToTF(current.orientation, current_q);
        tf::Matrix3x3(current_q).getRPY(r, p, y);
        r += droll;
        p += dpitch;
        y += dyaw;
        target.position.x += dx;
        target.position.y += dy;
        target.position.z += dz;
        target.orientation = tf::createQuaternionMsgFromRollPitchYaw(r, p, y);
        return target;
    }
    geometry_msgs::Pose to_pose_msg()
    {
        geometry_msgs::Pose res;
        res.position.x = dx;
        res.position.y = dy;
        res.position.z = dz;
        res.orientation = tf::createQuaternionMsgFromRollPitchYaw(droll, dpitch, dyaw);
        return res;
    }
    void from_pose_msg(geometry_msgs::Pose pose_delta)
    {
        dx = pose_delta.position.x;
        dy = pose_delta.position.y;
        dz = pose_delta.position.z;
        tf::Quaternion dq;
        tf::quaternionMsgToTF(pose_delta.orientation, dq);
        tf::Matrix3x3(dq).getRPY(droll, dpitch, dyaw);
    }
    arm_pose_delta()
    {
        clear();
    }
    void clear()
    {
        dx = 0;
        dy = 0;
        dz = 0;
        droll = 0;
        dpitch = 0;
        dyaw = 0;
    }
};