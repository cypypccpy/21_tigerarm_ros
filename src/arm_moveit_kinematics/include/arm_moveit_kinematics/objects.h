#pragma once
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <ros/ros.h>

const std::string Object_Name[4] = {"mineral", "barrire", "island_big", "island_little"};

enum object_type{
    MINERAL = 0,    //矿石
    BARRIER = 1,    //障碍块
    ISLAND_BIG = 2,
    ISLAND_LITTLE = 3
};

class objectBase
{
public:
    objectBase(){}
    void setup(
        moveit::planning_interface::PlanningSceneInterface* _psi_ptr,
        moveit::planning_interface::MoveGroupInterface* _mgi_ptr
    )
    {
        this->psi_ptr = _psi_ptr;
        this->mgi_ptr = _mgi_ptr;
    }
    void add();
    void remove();
    void addObj(std::string base_frame)
    {
        this->collision_object.header.frame_id = base_frame;
        this->collision_object.id = this->id;

        this->collision_object.primitives.clear();
        this->collision_object.primitive_poses.clear();
        this->collision_object.primitives.push_back(this->primitive);
        this->collision_object.primitive_poses.push_back(this->init_pose);
        this->collision_object.operation = this->collision_object.ADD;

        std::vector<moveit_msgs::CollisionObject> collision_objects;
        collision_objects.push_back(this->collision_object);

        this->psi_ptr->addCollisionObjects(collision_objects);
        ROS_INFO_NAMED("arm_log", "Add an object %s into the world", collision_object.id.c_str());
    }
    void addMeshObj(std::string base_frame, shape_msgs::Mesh mesh, geometry_msgs::Pose mesh_pose)
    {
        this->collision_object.header.frame_id = base_frame;
        this->collision_object.id = this->id;

        this->collision_object.meshes.clear();
        this->collision_object.mesh_poses.clear();
        this->collision_object.meshes.push_back(mesh);
        this->collision_object.mesh_poses.push_back(mesh_pose);
        this->collision_object.operation = this->collision_object.ADD;

        std::vector<moveit_msgs::CollisionObject> collision_objects;
        collision_objects.push_back(this->collision_object);

        this->psi_ptr->addCollisionObjects(collision_objects);
        ROS_INFO_NAMED("arm_log", "Add an object %s into the world", collision_object.id.c_str());
    }
    void attachObj(geometry_msgs::Pose grab_pose)
    {
        std::string end_effector_link = this->mgi_ptr->getEndEffectorLink();
        this->collision_object.header.frame_id = end_effector_link;

        this->collision_object.primitive_poses.clear();
        this->collision_object.primitive_poses.push_back(grab_pose);
        this->collision_object.operation = this->collision_object.MOVE;
        this->psi_ptr->applyCollisionObject(this->collision_object);
        
        ROS_INFO_NAMED("arm_log", "Attach the object %s to the robot's link %s", id.c_str(), end_effector_link.c_str());
        this->mgi_ptr->attachObject(this->collision_object.id, end_effector_link);
    }
    void detachObj()
    {
        std::string end_effector_link = this->mgi_ptr->getEndEffectorLink();

        moveit_msgs::AttachedCollisionObject attached_object;
        attached_object.link_name = end_effector_link;
        attached_object.object.header.frame_id = end_effector_link;
        attached_object.object.id = this->id;

        //detach the object
        attached_object.object.operation = attached_object.object.REMOVE;
        this->psi_ptr->applyAttachedCollisionObject(attached_object);
    }
    void removeObj()
    {
        //remove the object
        this->collision_object.operation = this->collision_object.REMOVE;
        this->psi_ptr->applyCollisionObject(this->collision_object);

        //remove objects' shape and pose
        this->collision_object.primitives.clear();
        this->collision_object.primitive_poses.clear();
    }
protected:
    moveit::planning_interface::PlanningSceneInterface* psi_ptr = nullptr;
    moveit::planning_interface::MoveGroupInterface* mgi_ptr = nullptr;
    moveit_msgs::CollisionObject collision_object;  //实体
    shape_msgs::SolidPrimitive primitive; //几何形状
    geometry_msgs::Pose init_pose;  //初始位姿
    std::string id;
};

/*
 * 因为机械臂一次只能夹取一个物体，仿真场景中只添加一个关注物体即可
 */
class mineObject : public objectBase
{
public:
    mineObject(){}
    void pickup()
    {
        geometry_msgs::Pose grab_pose;
        grab_pose.orientation.w = 1.0;
        grab_pose.position.y = -0.20;
        
        this->attachObj(grab_pose);
    }
    void putdown()
    {
        this->remove();
    }
    void add(
        std::string base_frame,
        object_type type
    )
    {
        this->id = Object_Name[type];
        this->primitive = get_box_primitive();
        this->init_pose = get_box_random_pose();
        this->addObj(base_frame);
    }
    void remove()
    {
        this->detachObj();
        this->removeObj();        

        ROS_INFO_NAMED("arm_log", "Remove an object %s from the world", collision_object.id.c_str());
    }
private:
    shape_msgs::SolidPrimitive get_box_primitive()
    {
        // Define a box to add to the world.
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 0.2;
        primitive.dimensions[primitive.BOX_Y] = 0.2;
        primitive.dimensions[primitive.BOX_Z] = 0.2;

        return primitive;
    }
    geometry_msgs::Pose get_box_random_pose()
    {
        // Define a pose for the box (specified relative to frame_id)
        geometry_msgs::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = 0.7;
        box_pose.position.y = 0;
        box_pose.position.z = 0.51;

        return box_pose;
    }
    geometry_msgs::Pose grab_pose;  //被夹取后的位姿，假设不变
};

class islandObject : public objectBase
{
public:
    islandObject(){}
    void setup(
        moveit::planning_interface::PlanningSceneInterface* _psi_ptr,
        moveit::planning_interface::MoveGroupInterface* _mgi_ptr,
        std::string _resource_path
    )
    {
        this->psi_ptr = _psi_ptr;
        this->mgi_ptr = _mgi_ptr;
        if(!_resource_path.length())
            this->resource_path = "package://arm_moveit_kinematics/resource/";
        this->resource_path = _resource_path;
    }
    void add(
        std::string base_frame,
        object_type type
    )
    {
        this->id = Object_Name[type];
        this->mesh_shape = get_mesh_primitive(type);
        this->init_pose = get_mesh_random_pose();
        this->addMeshObj(base_frame, this->mesh_shape, this->init_pose);
    }
    void remove()
    {
        this->detachObj();
        this->removeObj();        

        ROS_INFO_NAMED("arm_log", "Remove an object %s from the world", collision_object.id.c_str());
    }
private:
    std::string resource_path;
    shape_msgs::Mesh mesh_shape;
    shape_msgs::Mesh get_mesh_primitive(object_type type)
    {
        std::string path = resource_path + Object_Name[type] + ".stl";
        
        /* 进制转换 mm --> m */
        Eigen::Vector3d vectorScale(0.001, 0.001, 0.001);

        ROS_INFO_NAMED("arm_log", "load stl file %s", path.c_str());
        // Define a box to add to the world.
        shapes::Mesh* m = shapes::createMeshFromResource(path, vectorScale);

        shape_msgs::Mesh mesh;
        shapes::ShapeMsg mesh_msg;  
        shapes::constructMsgFromShape(m, mesh_msg);
        mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

        ROS_INFO_NAMED("arm_log", "load mesh shape");
        return mesh;
    }
    geometry_msgs::Pose get_mesh_random_pose()
    {
        // Define a pose for the box (specified relative to frame_id)
        geometry_msgs::Pose mesh_pose;
        mesh_pose.orientation.w = 1.0;
        mesh_pose.position.x = 0.3;
        mesh_pose.position.y = -1.445;
        mesh_pose.position.z = 0;

        return mesh_pose;
    }
};