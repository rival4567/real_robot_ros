#include <ros/ros.h>
#include <std_msgs/String.h>

// MoveIt!
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/MoveGroupActionGoal.h>
#include <moveit_msgs/MoveGroupGoal.h>
#include <moveit_msgs/RobotTrajectory.h>

#include <moveit_msgs/RobotState.h>
#include <sensor_msgs/JointState.h>
#include <sstream>

#include <moveit_visual_tools/moveit_visual_tools.h>


class KR1410
{
public:
    moveit::planning_interface::MoveGroupInterface move_group_interface{"manipulator"};
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // For visualizing things in rviz
    moveit_visual_tools::MoveItVisualTools visual_tools{"base_link_1"};

    KR1410()
    {
        sub_ = n.subscribe("/move_group/goal", 1000, &KR1410::subCallback, this);
        trajectory_sub_ = n.subscribe("/move_group/display_planned_path", 100, &KR1410::trajectorySubCallback, this);
        pub_ = n.advertise<sensor_msgs::JointState>("/kr1410/query_start/joint_states", 1000);   

        /* Otherwise robot with zeros joint_states */
        ros::Duration(1.0).sleep();

        const moveit::core::JointModelGroup* joint_model_group =
            KR1410::move_group_interface.getCurrentState()->getJointModelGroup("manipulator");
        
        ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
        ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());
    }

    void trajectorySubCallback(const moveit_msgs::DisplayTrajectory::ConstPtr& msg)
    {
        // Get Planned trajectory path and create a visual trajectory line
        ROS_INFO("Listening to /move_group/display_planned_path ...");
        // visual_tools.publishTrajectoryLine(msg->trajectory, msg->t)

        // visual_tools.deleteAllMarkers();
        visual_tools.loadRemoteControl();

        Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
        text_pose.translation().z() = 1.0;
        
        visual_tools.setPlanningSceneTopic("/planning_scene");
        for(int i=0; i<10000; i++)
        {
            visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
            visual_tools.trigger();
        }
    }

    void subCallback(const moveit_msgs::MoveGroupActionGoal::ConstPtr& msg)
    {
        // Get MoveGroupActionGoal Message and publish joint states based on it
        ROS_INFO("Listening to goals on /move_group/goal ...");
        moveit_msgs::MoveGroupGoal goal;
        goal = msg->goal;

        moveit_msgs::RobotState start_state;
        start_state = goal.request.start_state;
        ros::Time stamp = ros::Time::now();
        start_state.joint_state.header.stamp = stamp;
        
        // Publishing joint
        ROS_INFO("Publising joint states ...");
        pub_.publish(start_state.joint_state);
    }

private:
    ros::NodeHandle n;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    ros::Subscriber trajectory_sub_;
};


int main(int argc, char **argv)
{
    // Initiate ROS
    ros::init(argc, argv, "kr1410_state_publisher");

    // Create an object of class KR1410StatePublisher
    KR1410 kr1410;

    ros::spin();

    return 0;
}