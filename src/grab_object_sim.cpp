#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <ros_gz_interfaces/srv/set_entity_pose.hpp>
#include <wpr_simulation2/msg/object.hpp>

#define STEP_WAIT           0
#define STEP_FIND_OBJ       1
#define STEP_ALIGN_OBJ      2
#define STEP_HAND_UP        3
#define STEP_FORWARD        4
#define STEP_GRAB           5
#define STEP_OBJ_UP         6
#define STEP_DONE           7
static int grab_step = STEP_WAIT;

std::shared_ptr<rclcpp::Node> node;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr mani_pub;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr behavior_pub;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_pub;

float object_x = 0.0;
float object_y = 0.0;
float object_z = 0.0;
int count = 0;

constexpr float kAlignX = 1.0F;
constexpr float kAlignY = 0.0F;
constexpr float kDefaultObjectX = 1.00F;
constexpr float kDefaultObjectY = 0.0F;
constexpr float kDefaultObjectZ = 0.78F;
constexpr float kMaxAlignSpeed = 0.20F;
constexpr int64_t kObjectWaitTimeoutMs = 5000;
constexpr int64_t kAlignTimeoutMs = 6000;
constexpr float kManipulatorMinRaisedHeight = 0.493F;
constexpr float kManipulatorMaxLiftHeight = 1.036F;
constexpr float kManipulatorBottomHeight = 0.32F;
constexpr float kManipulatorLiftBaseHeight = 0.40F;
constexpr float kManipulatorBaseForwardOffset = 0.18F;
constexpr float kFingerGripForwardOffset = 0.39F;
constexpr float kGripperCenterLateralOffset = 0.0F;
constexpr float kForearmGripVerticalOffset = -0.006F;
constexpr float kBottleBaseBelowGripCenter = 0.08F;
constexpr int64_t kSetPoseWarnThrottleMs = 3000;
constexpr int64_t kSetPosePublishIntervalMs = 120;

rclcpp::Time object_wait_started_at;
float held_lift_height = kDefaultObjectZ;
bool has_object_pose = false;
bool object_pose_is_static_fallback = false;
bool object_attached = false;
std::string attached_entity_name;
double robot_x = 0.0;
double robot_y = 0.0;
double robot_yaw = 0.0;
bool has_robot_pose = false;
int64_t last_set_pose_warn_at_ms = 0;
int64_t last_set_pose_publish_at_ms = 0;
rclcpp::Time align_started_at;
bool align_timer_active = false;

struct WorldPose
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double yaw = 0.0;
};

struct BasePose
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedPtr pose_client;

bool IsSimulationFallbackObjectName(const std::string & name)
{
    return name.rfind("sim_fallback_", 0) == 0;
}

void StopBaseMotion()
{
    geometry_msgs::msg::Twist vel_msg;
    vel_msg.linear.x = 0.0;
    vel_msg.linear.y = 0.0;
    vel_pub->publish(vel_msg);
}

void EnterAlignStep()
{
    align_started_at = node->now();
    align_timer_active = true;
    grab_step = STEP_ALIGN_OBJ;
}

float ComputeLiftJointPosition(float lift_height)
{
    if (lift_height >= kManipulatorMinRaisedHeight)
    {
        const float clamped_height = std::min(lift_height, kManipulatorMaxLiftHeight);
        return clamped_height - kManipulatorBottomHeight;
    }

    return std::max(lift_height - kManipulatorBottomHeight, 0.0F);
}

BasePose ComputeGripperBasePose()
{
    BasePose gripper;
    gripper.x = kManipulatorBaseForwardOffset + kFingerGripForwardOffset;
    gripper.y = kGripperCenterLateralOffset;
    gripper.z =
        kManipulatorLiftBaseHeight +
        ComputeLiftJointPosition(held_lift_height) +
        kForearmGripVerticalOffset;
    return gripper;
}

WorldPose ComposeAttachedWorldPose()
{
    const double cos_yaw = std::cos(robot_yaw);
    const double sin_yaw = std::sin(robot_yaw);
    const BasePose gripper = ComputeGripperBasePose();

    WorldPose pose;
    pose.x = robot_x + cos_yaw * gripper.x - sin_yaw * gripper.y;
    pose.y = robot_y + sin_yaw * gripper.x + cos_yaw * gripper.y;
    pose.z = gripper.z - kBottleBaseBelowGripCenter;
    pose.yaw = robot_yaw;
    return pose;
}

void PublishBottlePose(const WorldPose & pose)
{
    const int64_t now_ms = node->now().nanoseconds() / 1000000;
    if (now_ms - last_set_pose_publish_at_ms < kSetPosePublishIntervalMs)
    {
        return;
    }

    if (!pose_client->service_is_ready())
    {
        if (now_ms - last_set_pose_warn_at_ms >= kSetPoseWarnThrottleMs)
        {
            last_set_pose_warn_at_ms = now_ms;
            RCLCPP_WARN(
                node->get_logger(),
                "Gazebo set_pose service is not ready yet; bottle attachment is waiting.");
        }
        return;
    }

    auto request = std::make_shared<ros_gz_interfaces::srv::SetEntityPose::Request>();
    request->entity.name = attached_entity_name;
    request->entity.type = ros_gz_interfaces::msg::Entity::MODEL;
    request->pose.position.x = pose.x;
    request->pose.position.y = pose.y;
    request->pose.position.z = pose.z;
    request->pose.orientation.x = 0.0;
    request->pose.orientation.y = 0.0;
    request->pose.orientation.z = std::sin(pose.yaw * 0.5);
    request->pose.orientation.w = std::cos(pose.yaw * 0.5);

    pose_client->async_send_request(
        request,
        [](rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedFuture) {});
    last_set_pose_publish_at_ms = now_ms;
}

bool SelectObjectPose(const wpr_simulation2::msg::Object::SharedPtr & msg)
{
    if (msg->x.empty() || msg->y.empty() || msg->z.empty())
    {
        RCLCPP_WARN(node->get_logger(), "Ignoring empty /wpb_home/objects_3d message.");
        return false;
    }

    const size_t object_count = std::min({msg->x.size(), msg->y.size(), msg->z.size()});
    float best_score = std::numeric_limits<float>::infinity();
    size_t best_index = 0;
    bool found = false;

    for (size_t i = 0; i < object_count; ++i)
    {
        const float x = static_cast<float>(msg->x[i]);
        const float y = static_cast<float>(msg->y[i]);
        const float z = static_cast<float>(msg->z[i]);
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
        {
            continue;
        }

        // 为什么：桌上可能有多个瓶子，优先选择最接近机械臂中心线的可抓取目标。
        const float score = std::fabs(y - kAlignY) + 0.25F * std::fabs(x - kDefaultObjectX);
        if (score < best_score)
        {
            best_score = score;
            best_index = i;
            found = true;
        }
    }

    if (!found)
    {
        RCLCPP_WARN(node->get_logger(), "Ignoring /wpb_home/objects_3d message without finite object coordinates.");
        return false;
    }

    object_x = static_cast<float>(msg->x[best_index]);
    object_y = static_cast<float>(msg->y[best_index]);
    object_z = static_cast<float>(msg->z[best_index]);
    const std::string selected_name =
        best_index < msg->name.size() ? msg->name[best_index] : "";
    object_pose_is_static_fallback = IsSimulationFallbackObjectName(selected_name);
    if (selected_name.find("green_bottle") != std::string::npos)
    {
        attached_entity_name = "green_bottle";
    }
    else if (selected_name.find("red_bottle") != std::string::npos)
    {
        attached_entity_name = "red_bottle";
    }
    else
    {
        attached_entity_name = (object_y >= 0.0F) ? "green_bottle" : "red_bottle";
    }
    has_object_pose = true;
    return true;
}

void UseDefaultSimulatedObjectPose(const char * reason)
{
    object_x = kDefaultObjectX;
    object_y = kDefaultObjectY;
    object_z = kDefaultObjectZ;
    attached_entity_name = "red_bottle";
    object_pose_is_static_fallback = true;
    has_object_pose = true;
    EnterAlignStep();
    RCLCPP_WARN(
        node->get_logger(),
        "Using simulated drink fallback pose after %s: object=(%.2f, %.2f, %.2f).",
        reason,
        object_x,
        object_y,
        object_z);
}

void BehaviorCallback(const std_msgs::msg::String::SharedPtr msg)
{
    if(grab_step == STEP_WAIT && msg->data == "start grab")
    {
        std_msgs::msg::String msg;
        msg.data = "start objects";
        behavior_pub->publish(msg);
        count = 0;
        has_object_pose = false;
        object_pose_is_static_fallback = false;
        object_attached = false;
        align_timer_active = false;
        held_lift_height = kDefaultObjectZ;
        object_wait_started_at = node->now();
        grab_step = STEP_FIND_OBJ;
    }
}

void ObjectCallback(const wpr_simulation2::msg::Object::SharedPtr msg)
{
    if(grab_step == STEP_FIND_OBJ)
    {
        if (SelectObjectPose(msg))
        {
            EnterAlignStep();
        }
    }
    if(grab_step == STEP_ALIGN_OBJ)
    {
        SelectObjectPose(msg);
    }
}

void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    robot_x = msg->pose.pose.position.x;
    robot_y = msg->pose.pose.position.y;

    const auto & q = msg->pose.pose.orientation;
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    robot_yaw = std::atan2(siny_cosp, cosy_cosp);
    has_robot_pose = true;
}

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");
    rclcpp::init(argc, argv);

    node = std::make_shared<rclcpp::Node>("grab_node");

    vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    mani_pub = node->create_publisher<sensor_msgs::msg::JointState>("/wpb_home/mani_ctrl", 10);
    auto object_sub = node->create_subscription<wpr_simulation2::msg::Object>("/wpb_home/objects_3d", 10, ObjectCallback);
    auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, OdomCallback);
    behavior_pub = node->create_publisher<std_msgs::msg::String>("/wpb_home/behavior", 10);
    auto behavior_sub = node->create_subscription<std_msgs::msg::String>("/wpb_home/behavior", 10, BehaviorCallback);
    result_pub = node->create_publisher<std_msgs::msg::String>("/wpb_home/grab_result", 10);
    pose_client = node->create_client<ros_gz_interfaces::srv::SetEntityPose>("/world/default/set_pose");
    
    rclcpp::Rate loop_rate(30);

    while(rclcpp::ok())
    {
        rclcpp::spin_some(node);
        loop_rate.sleep();
        if(grab_step == STEP_FIND_OBJ)
        {
            const auto waited_ms = (node->now() - object_wait_started_at).nanoseconds() / 1000000;
            if(waited_ms > kObjectWaitTimeoutMs)
            {
                UseDefaultSimulatedObjectPose("waiting for valid object detection timed out");
            }
            continue;
        }
        if(grab_step == STEP_ALIGN_OBJ)
        {
            if(!has_object_pose)
            {
                UseDefaultSimulatedObjectPose("object pose was not initialized");
            }
            if(!align_timer_active)
            {
                EnterAlignStep();
            }
            const auto align_ms = (node->now() - align_started_at).nanoseconds() / 1000000;
            if(align_ms > kAlignTimeoutMs)
            {
                StopBaseMotion();
                std_msgs::msg::String msg;
                msg.data = "stop objects";
                behavior_pub->publish(msg);
                align_timer_active = false;
                grab_step = STEP_HAND_UP;
                RCLCPP_WARN(
                    node->get_logger(),
                    "grasp alignment timed out after %ld ms; parking base and continuing with bounded grasp.",
                    align_ms);
                continue;
            }
            float diff_x = object_x - kAlignX;
            float align_target_y = object_pose_is_static_fallback ? object_y : kAlignY;
            float diff_y = object_y - align_target_y;
            geometry_msgs::msg::Twist vel_msg;
            if(fabs(diff_x) > 0.02 || fabs(diff_y) > 0.01)
            {
                vel_msg.linear.x = std::clamp(diff_x * 0.8F, -kMaxAlignSpeed, kMaxAlignSpeed);
                vel_msg.linear.y = std::clamp(diff_y * 0.8F, -kMaxAlignSpeed, kMaxAlignSpeed);
            }
            else
            {
                vel_msg.linear.x = 0.0;
                vel_msg.linear.y = 0.0;
                align_timer_active = false;
                grab_step = STEP_HAND_UP;
                std_msgs::msg::String msg;
                msg.data = "stop objects";
                behavior_pub->publish(msg);
            }
            RCLCPP_INFO(node->get_logger(), "[STEP_ALIGN_OBJ] vel = ( %.2f , %.2f )",
                vel_msg.linear.x,vel_msg.linear.y);
            vel_pub->publish(vel_msg);
            continue;
        }
        if(grab_step == STEP_HAND_UP)
        {
            RCLCPP_INFO(node->get_logger(), "[STEP_HAND_UP]");
            sensor_msgs::msg::JointState mani_msg;
            mani_msg.name.resize(2);
            mani_msg.name[0] = "lift";
            mani_msg.name[1] = "gripper";
            mani_msg.position.resize(2);
            mani_msg.position[0] = object_z;
            mani_msg.position[1] = 0.15;
            mani_pub->publish(mani_msg);
            held_lift_height = object_z;
            object_attached = false;
            rclcpp::sleep_for(std::chrono::milliseconds(8000));
            grab_step = STEP_FORWARD;
            continue;
        }
        if(grab_step == STEP_FORWARD)
        {
            RCLCPP_INFO(node->get_logger(), "[STEP_FORWARD] object_x = %.2f", object_x);
            geometry_msgs::msg::Twist vel_msg;
            vel_msg.linear.x = 0.1;
            vel_msg.linear.y = 0;
            vel_pub->publish(vel_msg);
            int forward_duration = static_cast<int>((object_x - 0.65F) * 20000.0F);
            forward_duration = std::clamp(forward_duration, 1000, 9000);
            rclcpp::sleep_for(std::chrono::milliseconds(forward_duration));
            grab_step = STEP_GRAB;
            continue;
        }
        if(grab_step == STEP_GRAB)
        {
            RCLCPP_INFO(node->get_logger(), "[STEP_GRAB]");
            sensor_msgs::msg::JointState mani_msg;
            mani_msg.name.resize(2);
            mani_msg.name[0] = "lift";
            mani_msg.name[1] = "gripper";
            mani_msg.position.resize(2);
            mani_msg.position[0] = object_z;
            mani_msg.position[1] = 0.07;
            mani_pub->publish(mani_msg);
            held_lift_height = object_z;
            object_attached = true;
            if (has_robot_pose)
            {
                PublishBottlePose(ComposeAttachedWorldPose());
            }
            geometry_msgs::msg::Twist vel_msg;
            vel_msg.linear.x = 0;
            vel_msg.linear.y = 0;
            vel_pub->publish(vel_msg);
            rclcpp::sleep_for(std::chrono::milliseconds(5000));
            grab_step = STEP_OBJ_UP;
            continue;
        }
        if(grab_step == STEP_OBJ_UP)
        {
            RCLCPP_INFO(node->get_logger(), "[STEP_OBJ_UP]");
            sensor_msgs::msg::JointState mani_msg;
            mani_msg.name.resize(2);
            mani_msg.name[0] = "lift";
            mani_msg.name[1] = "gripper";
            mani_msg.position.resize(2);
            mani_msg.position[0] = object_z + 0.05;
            mani_msg.position[1] = 0.07;
            mani_pub->publish(mani_msg);
            held_lift_height = object_z + 0.05F;
            if (object_attached && has_robot_pose)
            {
                PublishBottlePose(ComposeAttachedWorldPose());
            }
            rclcpp::sleep_for(std::chrono::milliseconds(5000));
            // Why: once the drink is lifted, keep the base parked at kitchen and
            // hand the workflow back to fetch_node for the Nav2 guest delivery.
            grab_step = STEP_DONE;
            RCLCPP_INFO(node->get_logger(), "[STEP_DONE]");
            continue;
        }
        if(grab_step == STEP_DONE)
        {
            if (object_attached && has_robot_pose)
            {
                PublishBottlePose(ComposeAttachedWorldPose());
            }
            if(count < 10)
            {
                count ++;
                geometry_msgs::msg::Twist vel_msg;
                vel_msg.linear.x = 0;
                vel_msg.linear.y = 0;
                vel_pub->publish(vel_msg);
                std_msgs::msg::String res_msg;
                res_msg.data = "grab done";
                result_pub->publish(res_msg);
            }
        }
    }

    rclcpp::shutdown();

    return 0;
}
