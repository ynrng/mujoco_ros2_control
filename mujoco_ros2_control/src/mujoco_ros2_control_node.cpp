
#include "rclcpp/rclcpp.hpp"
#include "mujoco/mujoco.h"

#include "mujoco_ros2_control/mujoco_ros2_control.hpp"
#include "mujoco_ros2_control/mujoco_rendering.hpp"

#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/image.hpp"


// MuJoCo data structures
mjModel* mujoco_model = nullptr;
mjData* mujoco_data = nullptr;
rclcpp::Node::SharedPtr node = nullptr;

// main function
int main(int argc, const char** argv) {

  rclcpp::init(argc, argv);
  // std::shared_ptr<rclcpp::Node>
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("mujoco_ros2_control_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  RCLCPP_INFO_STREAM(node->get_logger(), "Initializing mujoco_ros2_control node...");
  auto model_path = node->get_parameter("mujoco_model_path").as_string();

  // load and compile model
  char error[1000] = "Could not load binary model";
  if (std::strlen(model_path.c_str())>4 && !std::strcmp(model_path.c_str()+std::strlen(model_path.c_str())-4, ".mjb")) {
    mujoco_model = mj_loadModel(model_path.c_str(), 0);
  } else {
    mujoco_model = mj_loadXML(model_path.c_str(), 0, error, 1000);
  }
  if (!mujoco_model) {
    mju_error("Load model error: %s", error);
  }

  RCLCPP_INFO_STREAM(node->get_logger(), "Mujoco model has been successfully loaded !");
  // make data
  mujoco_data = mj_makeData(mujoco_model);

  // initialize mujoco control
  auto control = mujoco_ros2_control::MujocoRos2Control(node, mujoco_model, mujoco_data);
  control.init();
  RCLCPP_INFO_STREAM(node->get_logger(), "Mujoco ros2 controller has been successfully initialized !");

  // initialize mujoco redering
  auto rendering = mujoco_ros2_control::MujocoRendering::get_instance();
  rendering->init(node, mujoco_model, mujoco_data);
  RCLCPP_INFO_STREAM(node->get_logger(), "Mujoco rendering has been successfully initialized !");

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr xpos_publisher_;
  xpos_publisher_ = node->create_publisher<geometry_msgs::msg::PoseArray>("/mjc/poses", 10);
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_pub;
  camera_pub = node->create_publisher<sensor_msgs::msg::Image>("mjc/camera", 10);
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub;
  depth_pub = node->create_publisher<sensor_msgs::msg::Image>("mjc/depth", 10);

  unsigned char* rgb;
  float* depth;


  // run main loop, target real-time simulation and 60 fps rendering
  while (rclcpp::ok() && !rendering->is_close_flag_raised()) {
    // advance interactive simulation for 1/60 sec
    //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
    //  this loop will finish on time for the next frame to be rendered at 60 fps.
    //  Otherwise add a cpu timer and exit this loop when it is time to render.
    mjtNum simstart = mujoco_data->time;
    while (mujoco_data->time - simstart < 1.0/60.0) {
      control.update();
    }
    rendering->update();

    // debug
    // mj_printData(mujoco_model, mujoco_data, "/home/yan/code/assembly_ws/mjdata");
    // mj_printModel(mujoco_model, "/home/yan/code/assembly_ws/mjmodel");

    // camera data
    {

        // unsigned char rgb[640 * 480 * 3];
        // float depth[640 * 480];

        rendering->get_camera_buffer(&rgb, &depth);

        // Depth image message
        auto depth_msg = sensor_msgs::msg::Image();
        depth_msg.header.stamp = node->now();
        depth_msg.header.frame_id = "camera";
        depth_msg.height = 800;
        depth_msg.width = 800;
        depth_msg.encoding = "32FC1";
        depth_msg.is_bigendian = false;
        depth_msg.step = 800 * sizeof(float);
        // Copy depth buffer to msg.data
        depth_msg.data.resize(800 * 800 * sizeof(float));
        // Flip depth vertically
        for (int y = 0; y < 800; ++y) {
            std::memcpy(
                depth_msg.data.data() + y * 800 * sizeof(float),
                depth + (799 - y) * 800,
                800 * sizeof(float)
            );
        }

        depth_pub->publish(depth_msg);



        // Depth image message
        auto camera_msg = sensor_msgs::msg::Image();
        camera_msg.header.stamp = node->now();
        camera_msg.header.frame_id = "camera";
        camera_msg.height = 800;
        camera_msg.width = 800;
        camera_msg.encoding = "rgb8";
        camera_msg.step = 800 * 3;
        // Copy depth buffer to msg.data
        camera_msg.data.resize(800 * 800 * 3);
        for (int y = 0; y < 800; ++y) {
            std::memcpy(
                camera_msg.data.data() + y * 800 * 3,
                rgb + (799 - y) * 800 * 3,
                800 * 3
            );
        }

        camera_pub->publish(camera_msg);
    }

    // publish xpos + xquat
    {
      std::vector<std::string> list_items = { "peg-red","peg-green", "hole" };
      auto pose_array = geometry_msgs::msg::PoseArray();
      pose_array.header.stamp = node->now();
      pose_array.header.frame_id = "world";

      for (std::size_t i = 0; i < list_items.size(); i++) {
        auto pose = geometry_msgs::msg::Pose();
        int geom_id = mj_name2id(mujoco_model, mjOBJ_BODY, list_items[i].c_str());
        if (geom_id == -1) {
          RCLCPP_WARN(node->get_logger(), "Body '%s' not found!", list_items[i].c_str());
        }
        else {
          pose.position.x = mujoco_data->xpos[3 * geom_id];
          pose.position.y = mujoco_data->xpos[3 * geom_id + 1];
          pose.position.z = mujoco_data->xpos[3 * geom_id + 2];
          pose.orientation.w = mujoco_data->xquat[4 * geom_id];
          pose.orientation.x = mujoco_data->xquat[4 * geom_id + 1];
          pose.orientation.y = mujoco_data->xquat[4 * geom_id + 2];
          pose.orientation.z = mujoco_data->xquat[4 * geom_id + 3];

          pose_array.poses.push_back(pose);
        }
      }

      xpos_publisher_->publish(pose_array);
    }
  }

  rendering->close();

  // free MuJoCo model and data
  mj_deleteData(mujoco_data);
  mj_deleteModel(mujoco_model);

  return 1;
}
