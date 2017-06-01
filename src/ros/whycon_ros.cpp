#include <camera_info_manager/camera_info_manager.h>
#include <fstream>
#include <tf/tf.h>
#include <geometry_msgs/PoseArray.h>
#include <yaml-cpp/yaml.h>
#include <whycon/Projection.h>
#include "whycon_ros.h"

whycon::WhyConROS::WhyConROS(ros::NodeHandle &n) : is_tracking(false), should_reset(true), it(n) {
  transformation_loaded = false;
  similarity.setIdentity();

  if (!n.getParam("targets", targets)) throw std::runtime_error("Private parameter \"targets\" is missing");

  std::string vicon_quad_topic, vicon_payload_topic;
  n.param("vicon_quad_topic", vicon_quad_topic, std::string(""));
  n.param("vicon_payload_topic", vicon_payload_topic, std::string(""));
  n.param("name", frame_id, std::string("whycon"));
  n.param("world_frame", world_frame_id, std::string("world"));
  n.param("max_attempts", max_attempts, 1);
  n.param("max_refine", max_refine, 1);
  n.param("B_T_BC_", B_T_BC_yaml, {0, 0, 0});
  n.param("R_BC_", R_BC_yaml, {1, 0, 0, 0, 1, 0, 0, 0, 1});
  n.param("cable_length_", cable_length_, 1.0);
  n.param("distance_tag_CoG", distance_tag_CoG, 0.0);
  n.param("use_omni_model", use_omni_model, false);
  n.param("calib_file", calib_file, std::string(""));
  n.param("publish_tf", publish_tf, false);
  n.param("publish_vicon", publish_vicon, false);
  n.param("transform_to_world_frame", transform_to_world_frame, false);

  B_T_BC_ = cv::Vec3d(B_T_BC_yaml[0], B_T_BC_yaml[1], B_T_BC_yaml[2]);
  R_BC_ = cv::Matx33d(R_BC_yaml[0], R_BC_yaml[1], R_BC_yaml[2], \
                      R_BC_yaml[3], R_BC_yaml[4], R_BC_yaml[5], \
                      R_BC_yaml[6], R_BC_yaml[7], R_BC_yaml[8]);

  n.getParam("outer_diameter", parameters.outer_diameter);
  n.getParam("inner_diameter", parameters.inner_diameter);
  n.getParam("center_distance_tolerance_abs", parameters.center_distance_tolerance_abs);
  n.getParam("center_distance_tolerance_ratio", parameters.center_distance_tolerance_ratio);
  n.getParam("roundness_tolerance", parameters.roundness_tolerance);
  n.getParam("circularity_tolerance", parameters.circularity_tolerance);
  n.getParam("max_size", parameters.max_size);
  n.getParam("min_size", parameters.min_size);
  n.getParam("ratio_tolerance", parameters.ratio_tolerance);
  n.getParam("max_eccentricity", parameters.max_eccentricity);

  load_transforms();

  /* initialize ros */
  int input_queue_size = 1;
  n.param("input_queue_size", input_queue_size, input_queue_size);
  if (use_omni_model)
    cam_sub = it.subscribeCamera("/camera/image_raw", input_queue_size,
                                 boost::bind(&WhyConROS::on_image, this, _1, _2));
  else
    cam_sub = it.subscribeCamera("/camera/image_rect_color", input_queue_size,
                                 boost::bind(&WhyConROS::on_image, this, _1, _2));

  image_pub = n.advertise<sensor_msgs::Image>("image_out", 1);
  pointload_odom_pub = n.advertise<payload_msgs::PointloadOdom>("pointload_odom", 1);
  observ_dir_pub = n.advertise<geometry_msgs::Vector3Stamped>("observ_dir", 1);

  reset_service = n.advertiseService("reset", &WhyConROS::reset, this);

  //omni cam model
  get_ocam_model(&model, strdup(calib_file.c_str()));
}

void whycon::WhyConROS::on_image(const sensor_msgs::ImageConstPtr &image_msg,
                                 const sensor_msgs::CameraInfoConstPtr &info_msg) {
  bool publish_images = (image_pub.getNumSubscribers() != 0);

  camera_model.fromCameraInfo(info_msg);
  if (camera_model.fullResolution().width == 0) {
    ROS_ERROR_STREAM("camera is not calibrated!");
    return;
  }

  cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(image_msg, "rgb8");
  const cv::Mat &image = cv_ptr->image;

  if (!system)
    system = boost::make_shared<whycon::LocalizationSystem>(targets, image.size().width, image.size().height,
                                                            cv::Mat(camera_model.fullIntrinsicMatrix()),
                                                            cv::Mat(camera_model.distortionCoeffs()), parameters);

  is_tracking = system->localize(image, should_reset/*!is_tracking*/, max_attempts, max_refine);



  if (is_tracking) {

    cv::Mat output_image;
    if (publish_images)
      output_image = cv_ptr->image.clone();

    for (int id = 0; id < system->targets; id++) {
      double point3D[3];
      double point2D[2] = {system->get_circle(id).y, system->get_circle(id).x};
      cam2world(point3D, point2D, &model);

      // adapt coord sys of ocam to coord sys of whycon
      cv::Vec3d direction = {point3D[1],
                             point3D[0],
                            -point3D[2]};

      // publish direction
      geometry_msgs::Vector3Stamped p;
      p.vector.x = direction(0);
      p.vector.y = direction(1);
      p.vector.z = direction(2);
      p.header = image_msg->header;
      p.header.frame_id = std::to_string(0);
      observ_dir_pub.publish(p);

      // draw each target
      if (publish_images) {
        std::ostringstream ostr;
        ostr << std::fixed << std::setprecision(2);
        //ostr << coord << " " << id;
        if (system->targets > 1)
          ostr << " id: " << id;
        else
          ostr << " ";
        system->get_circle(id).draw(output_image, ostr.str(), cv::Vec3b(0, 255, 255));
        cv::circle(output_image,camera_model.project3dToPixel(cv::Point3d(coord)),1,cv::Scalar(255,0,255),1,CV_AA);
      }
    }

    if (publish_images) {
      cv_bridge::CvImage output_image_bridge = *cv_ptr;
      output_image_bridge.image = output_image;
      image_pub.publish(output_image_bridge);
    }

    should_reset = false;

  } else if (publish_images)
    image_pub.publish(cv_ptr);

}

void whycon::WhyConROS::calculate_3D_position(const nav_msgs::OdometryConstPtr& msg_quad,
                                              const geometry_msgs::Vector3StampedConstPtr& msg_observ_dir) {

  bool publish_odom_whycon = (pointload_odom_pub.getNumSubscribers() != 0);

  // calculate rotation and angular velocity from vicon data
  cv::Matx33d R_WB =
    {1 - 2 * msg_quad->pose.pose.orientation.y * msg_quad->pose.pose.orientation.y -
         2 * msg_quad->pose.pose.orientation.z * msg_quad->pose.pose.orientation.z,
         2 * msg_quad->pose.pose.orientation.x * msg_quad->pose.pose.orientation.y -
         2 * msg_quad->pose.pose.orientation.z * msg_quad->pose.pose.orientation.w,
         2 * msg_quad->pose.pose.orientation.x * msg_quad->pose.pose.orientation.z +
         2 * msg_quad->pose.pose.orientation.y * msg_quad->pose.pose.orientation.w,
         2 * msg_quad->pose.pose.orientation.x * msg_quad->pose.pose.orientation.y +
         2 * msg_quad->pose.pose.orientation.z * msg_quad->pose.pose.orientation.w,
     1 - 2 * msg_quad->pose.pose.orientation.x * msg_quad->pose.pose.orientation.x -
         2 * msg_quad->pose.pose.orientation.z * msg_quad->pose.pose.orientation.z,
         2 * msg_quad->pose.pose.orientation.y * msg_quad->pose.pose.orientation.z -
         2 * msg_quad->pose.pose.orientation.x * msg_quad->pose.pose.orientation.w,
         2 * msg_quad->pose.pose.orientation.x * msg_quad->pose.pose.orientation.z -
         2 * msg_quad->pose.pose.orientation.y * msg_quad->pose.pose.orientation.w,
         2 * msg_quad->pose.pose.orientation.y * msg_quad->pose.pose.orientation.z +
         2 * msg_quad->pose.pose.orientation.x * msg_quad->pose.pose.orientation.w,
     1 - 2 * msg_quad->pose.pose.orientation.x * msg_quad->pose.pose.orientation.x -
         2 * msg_quad->pose.pose.orientation.y * msg_quad->pose.pose.orientation.y};
  cv::Vec3d vicon_quad_angVel =
    {msg_quad->twist.twist.angular.x,
     msg_quad->twist.twist.angular.y,
     msg_quad->twist.twist.angular.z};

  // calculate time difference between Vicon and Camera frame
  double time_diff_vw  ( ros::Duration(msg_observ_dir->header.stamp - msg_quad->header.stamp).toSec() );
  double time_diff_io  ( ros::Duration(ros::Time::now() - msg_observ_dir->header.stamp).toSec() );
  double time_diff_last( ros::Duration(msg_observ_dir->header.stamp - time_old_whycon).toSec() );

  num_meas++;
  if (time_diff_vw > 1) {//0.5/150.0) {
    dropped_frames++;
    ROS_INFO("----- dropped frame ----- %f -----", dropped_frames/num_meas);
    return;
  }

  // calculate mean time difference Vicon/Camera
  avg = (avg * (num_meas - 1) + std::abs(time_diff_vw) )/num_meas;
  ROS_INFO("w-v: avg    %f", avg);
  ROS_INFO("t-diff vw   %f", time_diff_vw);
  ROS_INFO("duration IO %f", time_diff_io);
  ROS_INFO("since last  %f", time_diff_last);

  cv::Vec3d direction = {msg_observ_dir->vector.x,
                         msg_observ_dir->vector.y,
                         msg_observ_dir->vector.z};

  // rotate from camera frame to quad frame
  direction = R_BC_ * direction;

  // calculate distance camera-load
  double b_term = 2 * B_T_BC_.dot(direction);
  double c_term = B_T_BC_.dot(B_T_BC_) - cable_length_ * cable_length_;
  double dist = (-b_term + sqrt(b_term * b_term - 4 * c_term)) / 2;

  // extend vector from quad origin to center of gravity of the load
  cv::Vec3d whycon_realtive_position_bodyFrame = (B_T_BC_ + dist * direction) * //calculate direction in quad frame
      (cable_length_ + distance_tag_CoG) / cable_length_; //adjust length

  // calculate position for output
  cv::Vec3d whycon_position_outputFrame;
  if (!transform_to_world_frame) //quad frame
    whycon_position_outputFrame = whycon_realtive_position_bodyFrame;
  else //world frame
    whycon_position_outputFrame = R_WB * whycon_realtive_position_bodyFrame;

  // calculate angle for output
  cv::Vec3d whycon_angle_outputFrame = { atan2(whycon_position_outputFrame(1), -whycon_position_outputFrame(2)),
                                        -atan2(whycon_position_outputFrame(0), -whycon_position_outputFrame(2)),
                                         0};

  // calculate velocity for output
  cv::Vec3d whycon_velocity_outputFrame;
  if (!time_old_whycon.isZero()) {
    whycon_velocity_outputFrame = (whycon_position_outputFrame - whycon_position_outputFrame_old) /
                                  (time_diff_last);
  }

  // calculate angular velocity for output
  cv::Vec3d whycon_angVel_outputFrame;
  if (!time_old_whycon.isZero()) {
    if (!transform_to_world_frame) {
      whycon_angVel_outputFrame = (whycon_angle_outputFrame - whycon_angle_outputFrame_old) /
                                  (time_diff_last);
    } else {
      // Version 1
      //whycon_angVel_outputFrame = (whycon_angle_outputFrame - whycon_angle_outputFrame_old) /
      //                            (time_diff_last);
      // Version 2
      //whycon_angVel_outputFrame = whycon_position_outputFrame.cross(whycon_velocity_outputFrame) /
      //                            whycon_position_outputFrame.dot(whycon_position_outputFrame);
      // Version 3
      whycon_angVel_outputFrame = R_WB *
                                  whycon_realtive_position_bodyFrame.cross( (whycon_realtive_position_bodyFrame-whycon_position_bodyFrame_old)/time_diff_last )/
                                  whycon_realtive_position_bodyFrame.dot(whycon_realtive_position_bodyFrame) +
                                  vicon_quad_angVel;
      // Version 4
      //whycon_angVel_outputFrame = R_WB *
      //                            (R_WB.t() * whycon_angle_outputFrame - R_WBold.t() * whycon_angle_outputFrame_old) /
      //                            (time_diff_last) +
      //                            vicon_quad_angVel_;

      // FILTERING
      // first order low-pass filter
      /*if (filter_a == 0.0) {
        double filter_h = 0.02;
        double filter_T = 0.1;
        filter_a = filter_h/(filter_T-filter_h);
      }

      whycon_angVel_outputFrame = filter_a  * whycon_angVel_outputFrame +
                               (1-filter_a) * whycon_angVel_outputFrame_old;
      whycon_angVel_outputFrame_old = whycon_angVel_outputFrame;
      //whycon_angVel_outputFrame_old2 = whycon_angVel_outputFrame_old;*/

      // up to third order butterwroth low-pass filter
      // update input matrix u and output matrix y
      whycon_angVel_u = whycon_angVel_u * cv::Matx44d(0,1,0,0,  0,0,1,0,  0,0,0,1,  0,0,0,0);
      whycon_angVel_y = whycon_angVel_y * cv::Matx44d(0,1,0,0,  0,0,1,0,  0,0,0,1,  0,0,0,0);

      // fill in new data
      whycon_angVel_u(0,0) = whycon_angVel_outputFrame(0);
      whycon_angVel_u(1,0) = whycon_angVel_outputFrame(1);
      whycon_angVel_u(2,0) = whycon_angVel_outputFrame(2);

      // fill in new data
      whycon_angVel_outputFrame = whycon_angVel_u * filter_B - whycon_angVel_y.get_minor<3,3>(0,1) * filter_A;
      whycon_angVel_y(0,0) = whycon_angVel_outputFrame(0);
      whycon_angVel_y(1,0) = whycon_angVel_outputFrame(1);
      whycon_angVel_y(2,0) = whycon_angVel_outputFrame(2);
    }
  }
  else
    whycon_angVel_outputFrame = {0.0, 0.0, 0.0};

  whycon_position_outputFrame_old = whycon_position_outputFrame;
  whycon_position_bodyFrame_old = whycon_realtive_position_bodyFrame;
  time_old_whycon = msg_observ_dir->header.stamp;
  whycon_angle_outputFrame_old = whycon_angle_outputFrame;




  // publish results
  payload_msgs::PointloadOdom pointload_odom;
  if (publish_odom_whycon) {
    if (transform_to_world_frame) { //output absolute, not relative position
      pointload_odom.pose_pointload.pose.position.x = whycon_position_outputFrame(0)+msg_quad->pose.pose.position.x;
      pointload_odom.pose_pointload.pose.position.y = whycon_position_outputFrame(1)+msg_quad->pose.pose.position.y;
      pointload_odom.pose_pointload.pose.position.z = whycon_position_outputFrame(2)+msg_quad->pose.pose.position.z;
    } else {
      pointload_odom.pose_pointload.pose.position.x = whycon_position_outputFrame(0);
      pointload_odom.pose_pointload.pose.position.y = whycon_position_outputFrame(1);
      pointload_odom.pose_pointload.pose.position.z = whycon_position_outputFrame(2);
    }

    pointload_odom.twist_pointload.twist.linear.x = whycon_velocity_outputFrame(0);
    pointload_odom.twist_pointload.twist.linear.y = whycon_velocity_outputFrame(1);
    pointload_odom.twist_pointload.twist.linear.z = whycon_velocity_outputFrame(2);

    pointload_odom.pose_pointload.pose.orientation.x = whycon_angle_outputFrame(0);
    pointload_odom.pose_pointload.pose.orientation.y = whycon_angle_outputFrame(1);
    pointload_odom.pose_pointload.pose.orientation.z = whycon_angle_outputFrame(2);
    pointload_odom.pose_pointload.pose.orientation.w = 0;

    pointload_odom.twist_pointload.twist.angular.x = whycon_angVel_outputFrame(0);
    pointload_odom.twist_pointload.twist.angular.y = whycon_angVel_outputFrame(1);
    pointload_odom.twist_pointload.twist.angular.z = whycon_angVel_outputFrame(2);

    pointload_odom.pose_quad = msg_quad->pose;
    pointload_odom.twist_quad = msg_quad->twist;

    pointload_odom.header = msg_observ_dir->header;
    pointload_odom_pub.publish(pointload_odom);
  }
}

bool whycon::WhyConROS::reset(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response) {
  should_reset = true;
  return true;
}

void whycon::WhyConROS::load_transforms(void) {
  std::string filename = frame_id + "_transforms.yml";
  ROS_INFO_STREAM("Loading file " << filename);

  std::ifstream file(filename);
  if (!file) {
    ROS_WARN_STREAM("Could not load \"" << filename << "\"");
    return;
  }

  YAML::Node node = YAML::Load(file);

  projection.resize(9);
  for (int i = 0; i < 9; i++)
    projection[i] = node["projection"][i].as<double>();

  std::vector<double> similarity_origin(3);
  for (int i = 0; i < 3; i++) similarity_origin[i] = node["similarity"]["origin"][i].as<double>();

  std::vector<double> similarity_rotation(4);
  for (int i = 0; i < 4; i++) similarity_rotation[i] = node["similarity"]["rotation"][i].as<double>();

  tf::Vector3 origin(similarity_origin[0], similarity_origin[1], similarity_origin[2]);
  tf::Quaternion Q(similarity_rotation[0], similarity_rotation[1], similarity_rotation[2], similarity_rotation[3]);

  similarity.setOrigin(origin);
  similarity.setRotation(Q);

  transformation_loaded = true;

  ROS_INFO_STREAM("Loaded transformation from \"" << filename << "\"");
}
