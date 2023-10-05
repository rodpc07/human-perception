#include <ros/ros.h>
#include <signal.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>

#include <moveit_msgs/CollisionObject.h>

#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>

#include <tf2/LinearMath/Transform.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <hri/hri.h>

#include <iostream>

using namespace std;
using namespace hri;

const static std::string HEAD_TF_PREFIX("head_");

const static std::string TORSO_TF_PREFIX("torso_");
const static std::string WAIST_TF_PREFIX("waist_");
const static std::string RIGHT_HIP_TF_PREFIX("r_hip_");
const static std::string LEFT_HIP_TF_PREFIX("l_hip_");

const static std::string RIGHT_SHOULDER_TF_PREFIX("r_shoulder_");
const static std::string LEFT_SHOULDER_TF_PREFIX("l_shoulder_");
const static std::string RIGHT_ELBOW_TF_PREFIX("r_elbow_");
const static std::string LEFT_ELBOW_TF_PREFIX("l_elbow_");
const static std::string RIGHT_WRIST_TF_PREFIX("r_wrist_");
const static std::string LEFT_WRIST_TF_PREFIX("l_wrist_");

const static std::string RIGHT_KNEE_TF_PREFIX("r_knee_");
const static std::string LEFT_KNEE_TF_PREFIX("l_knee_");
const static std::string RIGHT_ANKLE_TF_PREFIX("r_ankle_");
const static std::string LEFT_ANKLE_TF_PREFIX("l_ankle_");

const static std::vector<std::string> tf_prefixes{
    HEAD_TF_PREFIX,
    TORSO_TF_PREFIX,
    WAIST_TF_PREFIX,
    RIGHT_HIP_TF_PREFIX,
    LEFT_HIP_TF_PREFIX,
    RIGHT_SHOULDER_TF_PREFIX,
    LEFT_SHOULDER_TF_PREFIX,
    RIGHT_ELBOW_TF_PREFIX,
    LEFT_ELBOW_TF_PREFIX,
    RIGHT_WRIST_TF_PREFIX,
    LEFT_WRIST_TF_PREFIX,
    RIGHT_KNEE_TF_PREFIX,
    LEFT_KNEE_TF_PREFIX,
    RIGHT_ANKLE_TF_PREFIX,
    LEFT_ANKLE_TF_PREFIX};

const static ros::Duration BODY_TF_TIMEOUT(0.01);

const static double BOX_COLLISION_OBJECT_TOLERANCE(0.25);
const static double FULLBODY_COLLISION_OBJECT_LENGTH_TOLERANCE(0.1);
const static double FULLBODY_COLLISION_OBJECT_RADIUS_TOLERANCE(0.0);

class HumanPerception
{
public:
    HumanPerception(ros::NodeHandle n) : n_(n)
    {
        n_.param("/human_perception/collision_mode", mode, 0);

        if (mode != 0 && mode != 1)
        {
            ROS_WARN("Collision Mode value not valid! Must be 0 or 1. \n Setting value to default value 0.");
            mode = 0;
        }

        hri_listener = std::make_shared<HRIListener>();
        hri_listener->onBodyLost(std::bind(&HumanPerception::removeBody, this, std::placeholders::_1));

        planning_scene_interface = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

        addHumanCollisions();
    }

    void addHumanCollisions()
    {
        ros::Rate loop_rate(10);

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);

        while (ros::ok())
        {
            auto bodies = hri_listener->getBodies();

            for (auto &b : bodies)
            {
                auto bodie_id = b.first;

                std::map<std::string, geometry_msgs::TransformStamped> tf_map;

                for (auto &prefix : tf_prefixes)
                {
                    try
                    {
                        tf_map[prefix] = tfBuffer.lookupTransform("world", prefix + bodie_id, ros::Time(0));
                    }
                    catch (tf2::TransformException &ex)
                    {
                        ROS_WARN("%s", ex.what());
                        ros::Duration(1.0).sleep();
                        continue;
                    }
                }

                switch (mode)
                {
                case 0:
                    BoxCollision(bodie_id, tf_map);
                    break;
                case 1:
                    FullBodyCollision(bodie_id, tf_map);
                    break;
                }
            }

            loop_rate.sleep();
            ros::spinOnce();
        }
    }

    void removeBody(hri::ID bodieID)
    {
        ros::V_string human{"human_" + bodieID};
        planning_scene_interface->removeCollisionObjects(human);
    }

    bool BoxCollision(std::string id, std::map<std::string, geometry_msgs::TransformStamped> tf_map)
    {
        moveit_msgs::CollisionObject collision_objects;
        moveit_msgs::ObjectColor object_colors;

        std_msgs::ColorRGBA collisionColor;
        collisionColor.r = 80 / 255.0;
        collisionColor.g = 200 / 255.0;
        collisionColor.b = 255 / 255.0;
        collisionColor.a = 0.4;

        double xMax = -INFINITY, xMin = INFINITY, yMax = -INFINITY, yMin = INFINITY, zMax = -INFINITY, zMin = INFINITY;

        for (auto &tf : tf_map)
        {
            if (tf.second.transform.translation.x > xMax)
                xMax = tf.second.transform.translation.x;
            else if (tf.second.transform.translation.x < xMin)
                xMin = tf.second.transform.translation.x;

            if (tf.second.transform.translation.y > yMax)
                yMax = tf.second.transform.translation.y;
            else if (tf.second.transform.translation.y < yMin)
                yMin = tf.second.transform.translation.y;

            if (tf.second.transform.translation.z > zMax)
                zMax = tf.second.transform.translation.z;
            else if (tf.second.transform.translation.z < zMin)
                zMin = tf.second.transform.translation.z;
        }

        collision_objects.id = "human_" + id;
        collision_objects.header.frame_id = "world";

        collision_objects.primitives.resize(1);
        collision_objects.primitives[0].type = collision_objects.primitives[0].BOX;
        collision_objects.primitives[0].dimensions.resize(3);
        collision_objects.primitives[0].dimensions[0] = xMax - xMin + BOX_COLLISION_OBJECT_TOLERANCE;
        collision_objects.primitives[0].dimensions[1] = yMax - yMin + BOX_COLLISION_OBJECT_TOLERANCE;
        collision_objects.primitives[0].dimensions[2] = zMax - zMin + BOX_COLLISION_OBJECT_TOLERANCE;

        collision_objects.primitive_poses.resize(1);
        collision_objects.primitive_poses[0].position.x = (xMax + xMin) / 2;
        collision_objects.primitive_poses[0].position.y = (yMax + yMin) / 2;
        collision_objects.primitive_poses[0].position.z = (zMax + zMin) / 2;
        collision_objects.primitive_poses[0].orientation.w = 1.0;

        collision_objects.operation = collision_objects.ADD;

        // if (planning_scene_interface->getKnownObjectNames())

        return planning_scene_interface->applyCollisionObject(collision_objects, collisionColor);
    }

    bool FullBodyCollision(std::string id, std::map<std::string, geometry_msgs::TransformStamped> tf_map)
    {
        moveit_msgs::CollisionObject collision_objects;
        moveit_msgs::ObjectColor object_colors;

        std_msgs::ColorRGBA collisionColor;
        collisionColor.r = 80 / 255.0;
        collisionColor.g = 200 / 255.0;
        collisionColor.b = 255 / 255.0;
        collisionColor.a = 0.4;

        collision_objects.id = "human_" + id;
        collision_objects.header.frame_id = "world";

        // BODY POSE (to avoid warnings)
        collision_objects.pose.position.z = 0.0;
        collision_objects.pose.position.z = 0.0;
        collision_objects.pose.position.z = 0.0;
        collision_objects.pose.orientation.w = 1.0;

        collision_objects.primitives.resize(10);
        collision_objects.primitive_poses.resize(10);

        // HEAD

        collision_objects.primitives[0].type = collision_objects.primitives[0].SPHERE;
        collision_objects.primitives[0].dimensions.resize(1);
        collision_objects.primitives[0].dimensions[0] = 0.15 + FULLBODY_COLLISION_OBJECT_RADIUS_TOLERANCE;

        collision_objects.primitive_poses[0].position.x = tf_map[HEAD_TF_PREFIX].transform.translation.x;
        collision_objects.primitive_poses[0].position.y = tf_map[HEAD_TF_PREFIX].transform.translation.y;
        collision_objects.primitive_poses[0].position.z = tf_map[HEAD_TF_PREFIX].transform.translation.z;
        collision_objects.primitive_poses[0].orientation.w = 1.0;

        // TORSO

        tf2::Stamped<tf2::Transform> torso, waist, right_shoulder, left_shoulder;

        tf2::fromMsg(tf_map[TORSO_TF_PREFIX], torso);
        tf2::fromMsg(tf_map[WAIST_TF_PREFIX], waist);
        tf2::fromMsg(tf_map[RIGHT_SHOULDER_TF_PREFIX], right_shoulder);
        tf2::fromMsg(tf_map[LEFT_SHOULDER_TF_PREFIX], left_shoulder);

        collision_objects.primitives[1].type = collision_objects.primitives[1].CYLINDER;
        collision_objects.primitives[1].dimensions.resize(2);
        collision_objects.primitives[1].dimensions[0] = tf2::tf2Distance(torso.getOrigin(), waist.getOrigin()) + FULLBODY_COLLISION_OBJECT_LENGTH_TOLERANCE;
        collision_objects.primitives[1].dimensions[1] = tf2::tf2Distance(right_shoulder.getOrigin(), left_shoulder.getOrigin()) / 2 + FULLBODY_COLLISION_OBJECT_RADIUS_TOLERANCE;

        tf2::toMsg((torso.getOrigin() + waist.getOrigin()) / 2.0, collision_objects.primitive_poses[1].position); // POSITION

        tf2::Vector3 upperbody = waist.getOrigin() - torso.getOrigin();
        upperbody.normalize();

        double yaw = atan2(upperbody.y(), upperbody.x());
        double pitch = M_PI_2 - atan2(upperbody.z(), sqrt(pow(upperbody.x(), 2) + pow(upperbody.y(), 2)));

        tf2::Quaternion q1(tf2::Vector3(0, 0, 1), yaw);
        tf2::Quaternion q2(tf2::Vector3(0, 1, 0), pitch);
        tf2::Quaternion upperbody_quaternion = q1 * q2;
        upperbody_quaternion.normalize();

        tf2::convert(upperbody_quaternion, collision_objects.primitive_poses[1].orientation); // ORIENTATION

        // LEFT UPPER ARM
        membersCollision(tf_map, &collision_objects, 2, LEFT_SHOULDER_TF_PREFIX, LEFT_ELBOW_TF_PREFIX, 0.06);

        // LEFT FOREARM
        membersCollision(tf_map, &collision_objects, 3, LEFT_ELBOW_TF_PREFIX, LEFT_WRIST_TF_PREFIX, 0.05);

        // RIGHT UPPER ARM
        membersCollision(tf_map, &collision_objects, 4, RIGHT_SHOULDER_TF_PREFIX, RIGHT_ELBOW_TF_PREFIX, 0.06);

        // RIGHT FOREARM
        membersCollision(tf_map, &collision_objects, 5, RIGHT_ELBOW_TF_PREFIX, RIGHT_WRIST_TF_PREFIX, 0.05);

        // LEFT UPPER LEG
        membersCollision(tf_map, &collision_objects, 6, LEFT_HIP_TF_PREFIX, LEFT_KNEE_TF_PREFIX, 0.1);

        // LEFT LOWER LEG
        membersCollision(tf_map, &collision_objects, 7, LEFT_KNEE_TF_PREFIX, LEFT_ANKLE_TF_PREFIX, 0.075);

        // RIGHT UPPER LEG
        membersCollision(tf_map, &collision_objects, 8, RIGHT_HIP_TF_PREFIX, RIGHT_KNEE_TF_PREFIX, 0.1);

        // LERIGHTFT LOWER LEG
        membersCollision(tf_map, &collision_objects, 9, RIGHT_KNEE_TF_PREFIX, RIGHT_ANKLE_TF_PREFIX, 0.075);

        collision_objects.operation = collision_objects.ADD;

        if (planning_scene_interface->applyCollisionObject(collision_objects, collisionColor))
            return true;

        // planning_scene_interface->removeCollisionObjects(collision_objects.id);

        return false;
    }

    void membersCollision(std::map<std::string, geometry_msgs::TransformStamped> tf_map, moveit_msgs::CollisionObject *object, int index, std::string point1, std::string point2, double radius)
    {
        if (object->primitives.size() <= index)
        {
            object->primitives.resize(index + 1);
            ROS_INFO("ADD CAPACITY");
        }

        if (object->primitive_poses.size() <= index)
            object->primitive_poses.resize(index + 1);

        tf2::Stamped<tf2::Transform> point1_tf, point2_tf;

        tf2::fromMsg(tf_map[point1], point1_tf);
        tf2::fromMsg(tf_map[point2], point2_tf);

        object->primitives[index].type = object->primitives[index].CYLINDER;
        object->primitives[index].dimensions.resize(2);
        object->primitives[index].dimensions[0] = tf2::tf2Distance(point1_tf.getOrigin(), point2_tf.getOrigin()) + FULLBODY_COLLISION_OBJECT_LENGTH_TOLERANCE;
        object->primitives[index].dimensions[1] = radius + FULLBODY_COLLISION_OBJECT_RADIUS_TOLERANCE;

        tf2::toMsg((point1_tf.getOrigin() + point2_tf.getOrigin()) / 2.0, object->primitive_poses[index].position); // POSITION

        tf2::Vector3 member = point1_tf.getOrigin() - point2_tf.getOrigin();
        member.normalize();

        double yaw = atan2(member.y(), member.x());
        double pitch = M_PI_2 - atan2(member.z(), sqrt(pow(member.x(), 2) + pow(member.y(), 2)));

        tf2::Quaternion q1(tf2::Vector3(0, 0, 1), yaw);
        tf2::Quaternion q2(tf2::Vector3(0, 1, 0), pitch);
        tf2::Quaternion member_quaternion = q1 * q2;
        member_quaternion.normalize();

        tf2::convert(member_quaternion, object->primitive_poses[index].orientation); // ORIENTATION
    }

private:
    ros::NodeHandle n_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface;

    std::shared_ptr<HRIListener> hri_listener;

    int mode;
};

void removeHumanCollisions(int sig)
{
    ROS_INFO("Removing Human Collision Objects...");

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    ros::V_string knownObjects = planning_scene_interface.getKnownObjectNames();

    for (auto &object : knownObjects)
    {
        ROS_INFO(object.c_str());
    }

    ros::V_string humanObjects;
    std::copy_if(knownObjects.begin(), knownObjects.end(), std::back_inserter(humanObjects), [](const std::string &str)
                 { return str.find("human") != std::string::npos; });

    for (auto &object : humanObjects)
    {
        ROS_INFO(object.c_str());
    }

    planning_scene_interface.removeCollisionObjects(humanObjects);
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "human_perception", ros::init_options::NoSigintHandler);
    ros::NodeHandle node_handle;
    signal(SIGINT, removeHumanCollisions);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    HumanPerception hp = HumanPerception(node_handle);

    ros::shutdown();
    return 0;
}