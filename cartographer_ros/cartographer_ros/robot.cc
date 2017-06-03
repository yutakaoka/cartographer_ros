#include <string>
#include <vector>

#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/ros_log_sink.h"
#include "cartographer_ros_msgs/StartTrajectory.h"
#include "gflags/gflags.h"
#include "tf2_ros/transform_listener.h"

DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");

namespace cartographer_ros {
namespace {

const TrajectoryOptions LoadTrajectoryOptions() {
  auto file_resolver = cartographer::common::make_unique<
      cartographer::common::ConfigurationFileResolver>(
      std::vector<string>{FLAGS_configuration_directory});
  const string code =
      file_resolver->GetFileContentOrDie(FLAGS_configuration_basename);
  cartographer::common::LuaParameterDictionary lua_parameter_dictionary(
      code, std::move(file_resolver));
  return CreateTrajectoryOptions(&lua_parameter_dictionary);
}

void Run() {
  ::ros::NodeHandle node;
  ::ros::ServiceClient client = node.serviceClient<
    cartographer_ros_msgs::StartTrajectory>("start_trajectory");
  cartographer_ros_msgs::StartTrajectory srv;
  auto trajectory_options = LoadTrajectoryOptions();
  if (!trajectory_options.trajectory_builder_options.SerializeToString(
          &srv.request.options.trajectory_builder_options_proto)) {
    ROS_ERROR("Failed to serialize");
  }
  srv.request.options.provide_odom_frame =
      trajectory_options.provide_odom_frame;
  srv.request.options.use_laser_scan = trajectory_options.use_laser_scan;
  srv.request.options.use_odometry = trajectory_options.use_odometry;
  srv.request.options.use_multi_echo_laser_scan =
      trajectory_options.use_multi_echo_laser_scan;
  srv.request.options.num_point_clouds = trajectory_options.num_point_clouds;

  ::ros::param::param<string>("~tracking_frame",
                              srv.request.options.tracking_frame,
                              "");
  ROS_INFO_STREAM("tracking_frame is " << srv.request.options.tracking_frame);
  ::ros::param::param<string>("~published_frame",
                              srv.request.options.published_frame,
                              "");
  ROS_INFO_STREAM("published_frame is " << srv.request.options.published_frame);
  ::ros::param::param<string>("~odom_frame",
                              srv.request.options.odom_frame,
                              "");
  ROS_INFO_STREAM("odom_frame is " << srv.request.options.odom_frame);
  ::ros::param::param<string>("~laser_scan_topic",
                              srv.request.topics.laser_scan_topic,
                              "");
  ROS_INFO_STREAM(
      "laser_scan_topic is " << srv.request.topics.laser_scan_topic);
  ::ros::param::param<string>("~multi_echo_laser_scan_topic",
                              srv.request.topics.multi_echo_laser_scan_topic,
                              "");
  ::ros::param::param<string>("~point_cloud2_topic",
                              srv.request.topics.point_cloud2_topic,
                              "");
  ::ros::param::param<string>("~imu_topic",
                              srv.request.topics.imu_topic,
                              "");
  ::ros::param::param<string>("~odometry_topic",
                              srv.request.topics.odometry_topic,
                              "");

  if (!::ros::service::waitForService(
          "start_trajectory", ::ros::Duration(10.0))) {
    ROS_ERROR("start_trajectory service is not available");
    exit(0);
  }
    
  if (client.call(srv)) {
    ROS_INFO_STREAM("trajectory_id: " << srv.response.trajectory_id);
  } else {
    ROS_ERROR("Failed to call start_trajectory service");
  }
}
}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";

  ::ros::init(argc, argv, "robot_node");
  ::ros::start();

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  cartographer_ros::Run();
  ::ros::shutdown();
}
