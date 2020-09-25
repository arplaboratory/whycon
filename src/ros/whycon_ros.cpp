#include <camera_info_manager/camera_info_manager.h>
#include <fstream>
#include <sensor_msgs/CameraInfo.h>
#include <tf/tf.h>
#include <sstream>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <yaml-cpp/yaml.h>
#include <whycon/Projection.h>
#include <whycon/ElapseTime.h>
#include "whycon_ros.h"
#include <std_srvs/Trigger.h>
#include <std_msgs/Bool.h>
#include <chrono>

whycon::WhyConROS::WhyConROS(ros::NodeHandle& n) : is_tracking(false), should_reset(true), it(n)
{
	transformation_loaded = false;
	similarity.setIdentity();

        if (!n.getParam("targets", targets)) throw std::runtime_error("Private parameter \"targets\" is missing");
      
        n.param("name", frame_id, std::string("whycon"));
        n.param("world_frame", world_frame_id, std::string("world"));
        n.param("max_attempts", max_attempts, 1);
        n.param("max_refine", max_refine, 1);

	n.getParam("outer_diameter", parameters.outer_diameter);
	n.getParam("inner_diameter", parameters.inner_diameter);
	n.getParam("mav_name", mav_name);
	n.getParam("center_distance_tolerance_abs", parameters.center_distance_tolerance_abs);
	n.getParam("center_distance_tolerance_ratio", parameters.center_distance_tolerance_ratio);
	n.getParam("roundness_tolerance", parameters.roundness_tolerance);
	n.getParam("circularity_tolerance", parameters.circularity_tolerance);
	n.getParam("max_size", parameters.max_size);
	n.getParam("min_size", parameters.min_size);
	n.getParam("ratio_tolerance", parameters.ratio_tolerance);
	n.getParam("max_eccentricity", parameters.max_eccentricity);

        n.param("cable_length",  cable_length_, 1.0);

        n.param("Transformation/RCB/xx",  RCB_(0,0), 0.000601f);
        n.param("Transformation/RCB/xy", RCB_(0,1), 0.0f);
        n.param("Transformation/RCB/xz", RCB_(0,2), 0.0f);
        n.param("Transformation/RCB/yx", RCB_(1,0), 0.0f);
        n.param("Transformation/RCB/yy",  RCB_(1,1), 0.000589f);
        n.param("Transformation/RCB/yz", RCB_(1,2), 0.0f); 
	n.param("Transformation/RCB/zx", RCB_(2,0), 0.0f); 
	n.param("Transformation/RCB/zy", RCB_(2,1), 0.0f);
        n.param("Transformation/RCB/zz",  RCB_(2,2), 0.001076f);
      
        n.param("Transformation/tCB/x", tCB_(0), 0.0f);
        n.param("Transformation/tCB/y", tCB_(1), 0.0f);
        n.param("Transformation/tCB/z", tCB_(2), 0.001076f);

	ROS_INFO_STREAM("RCB is " << RCB_(0,0) << ", " << RCB_(0,1) <<  ", " << RCB_(0,2));
	ROS_INFO_STREAM("       " << RCB_(1,0) << ", " << RCB_(1,1) <<  ", " << RCB_(1,2));
	ROS_INFO_STREAM("       " << RCB_(2,0) << ", " << RCB_(2,1) <<  ", " << RCB_(2,2));
                                     
	ROS_INFO_STREAM("tCB is " << tCB_(0) << ", " << tCB_(1) <<  ", " << tCB_(2));
	std::vector<double> vec{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

        n.param("projection_matrix/data", camera_matrix, vec);
	cam_center_x = camera_matrix[2];
	cam_center_y = camera_matrix[6];
	focal_length_x_ = camera_matrix[0];
	focal_length_y_ = camera_matrix[5];
	ROS_INFO_STREAM("cable_length is " << cable_length_ << "cam_center_x is " << cam_center_x << "cam_center_y is , " << cam_center_y <<  "focal_length_x_, " << focal_length_x_ << "focal_length_y_, " << focal_length_y_);
	load_transforms();
	transform_broadcaster = boost::make_shared<tf::TransformBroadcaster>();

  /* initialize ros */
  int input_queue_size = 1;
  copr_status_ = 0;
  n.param("input_queue_size", input_queue_size, input_queue_size);
  //cam_sub = it.subscribeCamera("/camera/image_raw", input_queue_size, boost::bind(&WhyConROS::on_image, this, _1, _2));
  cam_sub = it.subscribeCamera("/"+mav_name+"/image_raw", input_queue_size, boost::bind(&WhyConROS::on_image, this, _1, _2));
  copr_status_sub = n.subscribe("copr_status", 10, &WhyConROS::copr_status_callback,this);
  estimated_tag_pos_sub = n.subscribe("/"+mav_name+"/tag_pos_body", 10, &WhyConROS::qvec_callback,this);
  //cam_sub = it.subscribeCamera("/"+mav_name+"/image_rect", input_queue_size, boost::bind(&WhyConROS::on_image, this, _1, _2));
  
  image_pub = n.advertise<sensor_msgs::Image>("image_out", 1);
  poses_pub = n.advertise<geometry_msgs::PoseArray>("poses", 1);
  elapsed_time_pub = n.advertise<whycon::ElapseTime>("detection_elapsed_time", 1);
  image_elapsed_time_pub = n.advertise<whycon::ElapseTime>("image_elapsed_time", 1);
  emergency_pub = n.advertise<std_msgs::Bool>("whycon_emergency", 1);
  transformed_poses_pub = n.advertise<geometry_msgs::Vector3Stamped>("transformed_poses", 1);
  original_transformed_poses_pub = n.advertise<geometry_msgs::Vector3Stamped>("original_transformed_poses", 1);
  //transformed_poses_pub = n.advertise<geometry_msgs::PoseArray>("transformed_poses", 1);
  context_pub = n.advertise<sensor_msgs::Image>("context", 1);
  projection_pub = n.advertise<whycon::Projection>("projection", 1);

  reset_service = n.advertiseService("reset", &WhyConROS::reset, this);
  //emergency_land_client = n.serviceClient<std_srvs::Trigger>("/payload/mav_services/Loadland");
}


void whycon::WhyConROS::copr_status_callback(const std_msgs::UInt8::ConstPtr &msg){

     copr_status_ = msg->data;
     if(copr_status_ == 1){
       ROS_INFO("whycon::Taking off");}
     else if(copr_status_ == 3){
       ROS_INFO("whycon::Enabling cooperative control");
     }else if(copr_status_ == 2){
       ROS_INFO("whycon::Landing");
     }

}

void whycon::WhyConROS::qvec_callback(const geometry_msgs::Vector3Stamped::ConstPtr &msg){
     
     Eigen::Vector3f tag_pos_bodyFrame(msg->vector.x,msg->vector.y,msg->vector.z);
     Eigen::Vector3f tag_pos_camFrame;
     tag_pos_camFrame =  RCB_.transpose() * (tag_pos_bodyFrame - tCB_) ;
     predict_circle_y = focal_length_y_ * tag_pos_camFrame(1)/tag_pos_camFrame(2)+cam_center_y; 
     predict_circle_x = focal_length_x_ * tag_pos_camFrame(0)/tag_pos_camFrame(2)+cam_center_x; 
     bool estimated_tag_pos_updated = true;
     system->detector.set_tag_pos(predict_circle_x, predict_circle_y);
     std::cout << "The predicted circle is " << predict_circle_x << ", " << predict_circle_y << std::endl;

}

void whycon::WhyConROS::on_image(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  camera_model.fromCameraInfo(info_msg);
  if (camera_model.fullResolution().width == 0) { ROS_ERROR_STREAM("camera is not calibrated!"); return; }

  cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(image_msg, "rgb8");
  //cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(image_msg, "mono8");
  const cv::Mat& image = cv_ptr->image;
  //const cv::Mat& image_raw = cv_ptr->image;
  //cv::resize(image,image,cv::Size(image.cols/2,image.rows/2));
  cv::Mat output_image;
  output_image = cv_ptr->image.clone();
  cv_bridge::CvImage output_image_bridge = *cv_ptr;
  output_image_bridge.image = output_image;
  cv::Point pt1 = cv::Point(predict_circle_x, predict_circle_y);
  cv::circle(output_image, pt1, 1, cv::Scalar(0,0,255), 1, CV_AA);
  image_pub.publish(output_image_bridge);

  if (!system)
    system = boost::make_shared<whycon::LocalizationSystem>(targets, image.size().width, image.size().height, cv::Mat(camera_model.fullIntrinsicMatrix()), cv::Mat(camera_model.distortionCoeffs()), parameters);

   std::chrono::time_point<std::chrono::system_clock> start, end; 
   start = std::chrono::system_clock::now(); 
   std::chrono::duration<float> image_elapsed_seconds = start - last_image_income_time; 
   whycon::ElapseTime image_elapsed_time;
   image_elapsed_time.elapsed_time.data = image_elapsed_seconds.count();
   image_elapsed_time.header = image_msg->header;
   image_elapsed_time_pub.publish(image_elapsed_time);
   last_image_income_time = start;

   is_tracking = system->localize(image, should_reset/*!is_tracking*/, max_attempts, max_refine);
   //is_tracking = system->localize(image, !is_tracking, max_attempts, max_refine);
   end = std::chrono::system_clock::now(); 
   std::chrono::duration<float> elapsed_seconds = end - start; 
   //std::cout << "image elapsed time: " << image_elapsed_seconds.count() << "s\n"; 
   //std::cout << "detection elapsed time: " << elapsed_seconds.count() << "s\n"; 
   whycon::ElapseTime elapsed_time;
   elapsed_time.elapsed_time.data = elapsed_seconds.count();
   elapsed_time.header = image_msg->header;
   elapsed_time_pub.publish(elapsed_time);

  if (is_tracking) {
    std::cout << "is_tracking is " << is_tracking << "\n"; 
    publish_results(image_msg->header, cv_ptr);
    should_reset = false;
  }
  
  double current_time = image_msg->header.stamp.toSec();
  if((current_time - last_pub_time > 0.5)&&(copr_status_ == 3)){
   //std_srvs::Trigger trig;
   //emergency_land_client.call(trig); 
   std_msgs::Bool emergency;
   emergency.data = true;
   //emergency_pub.publish(emergency);
   ROS_INFO("Entering emergency landing");
  }


  if (context_pub.getNumSubscribers() != 0) {
    cv_bridge::CvImage cv_img_context;
    cv_img_context.encoding = cv_ptr->encoding;
    cv_img_context.header.stamp = cv_ptr->header.stamp;
    system->detector.context.debug_buffer(cv_ptr->image, cv_img_context.image);
    context_pub.publish(cv_img_context.toImageMsg());
  }

}

bool whycon::WhyConROS::reset(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  should_reset = true;
  return true;
}

void whycon::WhyConROS::publish_results(const std_msgs::Header& header, const cv_bridge::CvImageConstPtr& cv_ptr)
{
  //bool publish_images = (image_pub.getNumSubscribers() != 0);
  bool publish_images = false;
  bool publish_poses = (poses_pub.getNumSubscribers() != 0);
  bool transformed_publish_poses = (transformed_poses_pub.getNumSubscribers() != 0);
  
  if (!publish_images && !transformed_publish_poses) return;
  
  // prepare image output
  cv::Mat output_image;
  if (publish_images)
    output_image = cv_ptr->image.clone();

  geometry_msgs::PoseArray pose_array;
  geometry_msgs::PoseArray transformed_pose_array;
  geometry_msgs::Vector3Stamped transformed_p;
  geometry_msgs::Vector3Stamped original_transformed_p;
  
  // go through detected targets
  for (int i = 0; i < system->targets; i++) {
    const whycon::CircleDetector::Circle& circle = system->get_circle(i);
    whycon::LocalizationSystem::Pose pose = system->get_pose(circle);
    double detected_length_ = sqrt(std::pow(pose.pos(0),2)+std::pow(pose.pos(1),2) + std::pow(pose.pos(2),2));
    if((circle.valid)&&(detected_length_-cable_length_<0.2)&&(detected_length_-cable_length_)>-0.2){
        //std::cout << "target id is "  << i << " and the difference is " << detected_length_-cable_length_ << std::endl;
        double point2D[2] = {(system->get_circle(i).y-cam_center_y)/focal_length_y_, (system->get_circle(i).x-cam_center_x)/focal_length_x_};
        //double point2D[2] = {(system->get_circle(i).y-243.144)/274.931, (system->get_circle(i).x-319.625)/275.078};
        //std::cout << "point 2d results:" << point2D[0] << " " << point2D[1] << std::endl;
        //std::cout << "system results:"  << system->get_circle(i).x << " " << system->get_circle(i).y << std::endl;
        double circle_cen_norm = sqrt(1 +std::pow(point2D[0],2) + std::pow(point2D[1],2));
        Eigen::Vector3f direction_camFrame(point2D[1]/circle_cen_norm,point2D[0]/circle_cen_norm,1/circle_cen_norm);
        // rotate from camera frame to quad frame
        Eigen::Vector3f direction_bodyFrame = RCB_ * direction_camFrame;
        
        // calculate distance camera-load
        double b_term = 2 * tCB_.dot(direction_bodyFrame);
        double c_term = tCB_.dot(tCB_) - std::pow(cable_length_,2);
        double dist = (-b_term + sqrt(b_term * b_term - 4 * c_term)) / 2;
        
        cv::Vec3f coord = pose.pos;
        Eigen::Vector3f relative_position_bodyFrame = tCB_ + dist * direction_bodyFrame; //calculate direction in quad frame
        Eigen::Vector3f cv_relative_position_camFrame = dist * direction_camFrame; //calculate direction in quad frame
        //std::cout << "dist results:" << dist << std::endl;

        // draw each target
        if (publish_images) {
          std::ostringstream ostr;
          ostr << std::fixed << std::setprecision(2);
            		ostr << coord << " " << i;
          circle.draw(output_image, ostr.str(), cv::Vec3b(0,255,255));
			/*whycon::CircleDetector::Circle new_circle = circle.improveEllipse(cv_ptr->image);
			new_circle.draw(output_image, ostr.str(), cv::Vec3b(0,255,0));*/
			//cv::circle(output_image, camera_model.project3dToPixel(cv::Point3d(coord)), 1, cv::Scalar(0,255,255), 1, CV_AA);
          //cv::Point projected_points = camera_model.project3dToPixel(cv::Point3d(coord));
          //std::cout << projected_points.x << " " << projected_points.y << std::endl; 
          //cv::Point constraint_projected_points = camera_model.project3dToPixel(cv::Point3d(cv_relative_position_camFrame[0],cv_relative_position_camFrame[1],cv_relative_position_camFrame[2]));
          //std::cout << "constraint: "<< constraint_projected_points.x << " " << constraint_projected_points.y << std::endl; 
	  cv::circle(output_image, camera_model.project3dToPixel(cv::Point3d(cv_relative_position_camFrame[0],cv_relative_position_camFrame[1],cv_relative_position_camFrame[2])), 1, cv::Scalar(0,0,255), 1, CV_AA);
			//cv::circle(output_image, cv::Point((int)system->get_circle(i).x,(int)system->get_circle(i).y), 1, cv::Scalar(255,0,0), 1, CV_AA);
          }

          if (transformed_publish_poses) {
            geometry_msgs::Pose p;
            geometry_msgs::Pose p_constraint;
            //geometry_msgs::Pose transformed_p;
            Eigen::Vector3f position(pose.pos(0), pose.pos(1), pose.pos(2));
            Eigen::Vector3f pos_robot = RCB_*position + tCB_;
            original_transformed_p.vector.x = pos_robot(0);//position in camera frame.
            original_transformed_p.vector.y = pos_robot(1);
            original_transformed_p.vector.z = pos_robot(2);
            transformed_p.vector.x = relative_position_bodyFrame(0);//position in camera frame.
            transformed_p.vector.y = relative_position_bodyFrame(1);
            transformed_p.vector.z = relative_position_bodyFrame(2);
          
            //transformed_p.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, pose.rot(0), pose.rot(1));
            p_constraint.position.x = cv_relative_position_camFrame(0);
            p_constraint.position.y = cv_relative_position_camFrame(1);
            p_constraint.position.z = cv_relative_position_camFrame(2);
            p_constraint.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, pose.rot(0), pose.rot(1));
            p.position.x = pose.pos(0);
            p.position.y = pose.pos(1);
            p.position.z = pose.pos(2);
            p.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, pose.rot(0), pose.rot(1));
            pose_array.poses.push_back(p);
            pose_array.poses.push_back(p_constraint);

            pose_array.header = header;
            pose_array.header.frame_id = frame_id;
            transformed_p.header = header;
            transformed_p.header.frame_id = frame_id;
            poses_pub.publish(pose_array);
            transformed_poses_pub.publish(transformed_p);
            original_transformed_poses_pub.publish(original_transformed_p);
	    last_pub_time = header.stamp.toSec();
            //transformed_pose_array.poses.push_back(transformed_p);
          }
      }else{
	  should_reset = true;
	  return;
      }
  }

  if (publish_images) {
    cv_bridge::CvImage output_image_bridge = *cv_ptr;
    output_image_bridge.image = output_image;
    image_pub.publish(output_image_bridge);
  }

  /*if (transformed_publish_poses) {
    pose_array.header = header;
    pose_array.header.frame_id = frame_id;
    transformed_p.header = header;
    transformed_p.header.frame_id = frame_id;
    poses_pub.publish(pose_array);
    transformed_poses_pub.publish(transformed_p);
    original_transformed_poses_pub.publish(original_transformed_p);
  }*/

  if (transformation_loaded)
  {
	transform_broadcaster->sendTransform(tf::StampedTransform(similarity, header.stamp, world_frame_id, frame_id));

	whycon::Projection projection_msg;
	projection_msg.header = header;
	for (size_t i = 0; i < projection.size(); i++) projection_msg.projection[i] = projection[i];
	projection_pub.publish(projection_msg);
  } 
}

void whycon::WhyConROS::load_transforms(void)
{
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

	ROS_INFO_STREAM("Loaded transformation from \"" << filename <<  "\"");
}


