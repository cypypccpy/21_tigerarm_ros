#pragma once
#include <yaml-cpp/yaml.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <unordered_map>

#define _constrain(a,b,x) x>=b?b:(x<=a?a:x)

enum action_type{
    NONE = 0,
    PICKUP = 1,
    PUTDOWN = 2,
};

struct point_struct{
    int id;
    geometry_msgs::Pose pose;
    action_type action_t;
    point_struct(){}
    point_struct(const point_struct& p){
        id = p.id;
        pose = p.pose;
        action_t = p.action_t;
    }
    point_struct(YAML::Node node){
        from_yaml(node);
    }
    void from_yaml(YAML::Node node)
    {
        id = node["id"].as<int>();
        pose.position.x = node["xyz"][0].as<double>();
        pose.position.y = node["xyz"][1].as<double>();
        pose.position.z = node["xyz"][2].as<double>();
        double r,p,y;
        r = node["rpy"][0].as<double>();
        p = node["rpy"][1].as<double>();
        y = node["rpy"][2].as<double>();
        pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(r,p,y);
        action_t = action_type(node["action"].as<int>());
    }
};

class pointsLoader
{
public:
    pointsLoader(){}
    void load(std::string path){
        if(!path.length())
            path = "/home/robotlab/project/21_tigerarm_ros/src/arm_moveit_kinematics/config/poses.yaml";
        this->config_node = YAML::LoadFile(path);
        this->total_points = this->config_node["Total"].as<int>();

        for(size_t i = 0; i < total_points; i++){
            this->position_vector.push_back(point_struct(this->config_node["Points"][i]));
            tag_dict[this->config_node["Points"][i]["tag"].as<std::string>()] = i;
        }

        ROS_INFO_NAMED("arm_log", "Loaded %d points", total_points);
    }

    point_struct next(){
        int idx = current_p >= total_points - 1 ? total_points - 1 : ++current_p;
        return position_vector[idx];
    }

    point_struct last(){
        int idx = current_p <= 0 ? 0 : --current_p;
        return position_vector[idx];
    }

    point_struct end(){
        if(this->total_points >= 1){
            this->current_p = this->total_points - 1;
            return position_vector[this->current_p];
        }
        else
            return point_struct();
    }

    point_struct first(){
        this->current_p = 0;
        return position_vector[this->current_p];
    }

    point_struct operator [](int idx){
        idx = _constrain(0, this->total_points, idx);
        return position_vector[idx];
    }

    point_struct operator [](std::string tag){
        int idx = tag_dict[tag];
        idx = _constrain(0, this->total_points, idx);
        return position_vector[idx];
    }

private:
    int total_points = 0;
    int current_p = -1;
    YAML::Node config_node;
    std::vector<point_struct> position_vector;
    std::unordered_map<std::string, int> tag_dict;
};