#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <fstream>
#include <sstream>
#include <vector>

using Client = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

struct Waypoint {
  double x{0}, y{0}, yaw{0};
};

std::vector<Waypoint> loadCSV(const std::string& file)
{
  std::vector<Waypoint> goals;
  std::ifstream in(file);
  if (!in.is_open()) {
    ROS_ERROR_STREAM("Cannot open CSV file " << file);
    return goals;
  }
  std::string line;
  while (std::getline(in, line)) {
    std::stringstream ss(line);
    std::string field;
    Waypoint w;
    if (std::getline(ss, field, ',')) w.x = std::stod(field);
    if (std::getline(ss, field, ',')) w.y = std::stod(field);
    if (std::getline(ss, field, ',')) w.yaw = std::stod(field)* (M_PI / 180.0);
    goals.push_back(w);
  }
  ROS_INFO("Loaded %zu waypoints", goals.size());
  return goals;
}

move_base_msgs::MoveBaseGoal toGoal(const Waypoint& w)
{
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";   
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = w.x;
  goal.target_pose.pose.position.y = w.y;

  tf2::Quaternion q;  q.setRPY(0, 0, w.yaw);
  goal.target_pose.pose.orientation.x = q.x();
  goal.target_pose.pose.orientation.y = q.y();
  goal.target_pose.pose.orientation.z = q.z();
  goal.target_pose.pose.orientation.w = q.w();
  return goal;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "goal_controller");
  ros::NodeHandle nh("~");

  /* ---- parametri ------------------------------------------------------- */
 //std::string pkg_path = ros::package::getPath("second_project");   // …/catkin_ws/src/second_project
  std::string csv_path;
  nh.getParam("csv_file", csv_path);
  double goal_timeout = 120.0;   // [s] per ogni goal

  auto goals = loadCSV(csv_path);
  if (goals.empty())  { ROS_ERROR("No goals loaded.");  return 1; }

  /* ---- action client --------------------------------------------------- */
  Client client("move_base", true);
  ROS_INFO("Waiting for move_base action …");
  client.waitForServer();
  ROS_INFO("move_base ready; sending goals");

  for (size_t i = 0; ros::ok() && i < goals.size(); ++i)
  {
    move_base_msgs::MoveBaseGoal goal = toGoal(goals[i]);
    ROS_INFO("Sending goal %zu : (%.2f, %.2f, %.1f deg)",
             i, goals[i].x, goals[i].y,goals[i].yaw*180/M_PI);
    client.sendGoal(goal);

    bool ok = client.waitForResult(ros::Duration(goal_timeout));
    if (!ok || client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_WARN("Goal %zu not achieved, skipping to next.", i);
      client.cancelGoal();
    }
  }
  ROS_INFO("All goals processed - node exit.");
  return 0;
}
