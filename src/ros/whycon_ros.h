#include <ros/ros.h>
#include <whycon/localization_system.h>
#include <boost/shared_ptr.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <std_srvs/Empty.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <Eigen/Geometry>
#include <chrono>

namespace whycon {
  class WhyConROS {
    public:
      WhyConROS(ros::NodeHandle& n);

      void on_image(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
      bool reset(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

    private:
			void load_transforms(void);
      void publish_results(const std_msgs::Header& header, const cv_bridge::CvImageConstPtr& cv_ptr);
      
			whycon::DetectorParameters parameters;
      boost::shared_ptr<whycon::LocalizationSystem> system;
      std::string mav_name;
      bool is_tracking, should_reset;
      Eigen::Matrix<float,3,3> RCB_;
      Eigen::Vector3f tCB_;
      int max_attempts, max_refine;
      std::string world_frame_id, frame_id;
      int targets,copr_status_;
      double xscale, yscale,cam_center_x,cam_center_y;
      double cable_length_,focal_length_y_,focal_length_x_; 
      double predict_circle_y, predict_circle_x;
      double last_pub_time;
      std::chrono::time_point<std::chrono::system_clock> last_image_income_time;

      std::vector<double> projection;
      std::vector<double> camera_matrix;
      tf::Transform similarity;
      void copr_status_callback(const std_msgs::UInt8::ConstPtr &msg);
      void qvec_callback(const geometry_msgs::Vector3Stamped::ConstPtr &msg);

      image_transport::ImageTransport it;
      image_transport::CameraSubscriber cam_sub;
      ros::Subscriber copr_status_sub;
      ros::Subscriber estimated_tag_pos_sub;
      ros::ServiceServer reset_service;
      ros::ServiceClient emergency_land_client;

      ros::Publisher image_elapsed_time_pub, elapsed_time_pub, image_pub, poses_pub, context_pub, projection_pub, transformed_poses_pub, original_transformed_poses_pub, emergency_pub;
      boost::shared_ptr<tf::TransformBroadcaster>	transform_broadcaster;

      image_geometry::PinholeCameraModel camera_model;

      bool transformation_loaded;
  };
}
