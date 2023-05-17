#include "RGBDNode.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc > 1) {
        ROS_WARN ("Arguments supplied via command line are neglected.");
    }

    ros::NodeHandle node_handle;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    image_transport::ImageTransport image_transport (node_handle);

    RGBDNode node (ORB_SLAM2::System::RGBD, node_handle, image_transport);

    node.Init();

    ros::spin();

    ros::shutdown();

    return 0;
}


RGBDNode::RGBDNode (const ORB_SLAM2::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport) : Node (sensor, node_handle, image_transport) {
  rgb_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/d435i/color/image_raw", 1);
  depth_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/d435i/depth/image_raw", 1);
  camera_info_topic_ = "/d435i/color/camera_info";
  // camera_info_topic_ = "/camera/rgb/camera_info";

  sync_ = new message_filters::Synchronizer<sync_pol> (sync_pol(10), *rgb_subscriber_, *depth_subscriber_);
  sync_->registerCallback(boost::bind(&RGBDNode::ImageCallback, this, _1, _2));

  tag_pose_sub_ = node_handle.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/tag_pose", 10, &RGBDNode::tagpose_callback, this);
  loopfuc_sub_ = node_handle.subscribe<std_msgs::Bool>("/static_flag", 10, &RGBDNode::loopfunc_callback, this);
}


RGBDNode::~RGBDNode () {
  delete rgb_subscriber_;
  delete depth_subscriber_;
  delete sync_;
}


void RGBDNode::ImageCallback (const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD) {
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptrRGB;
  try {
      cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  cv_bridge::CvImageConstPtr cv_ptrD;
  try {
    cv_ptrD = cv_bridge::toCvShare(msgD);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  current_frame_time_ = msgRGB->header.stamp;

  orb_slam_->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());

  Update();
}

void RGBDNode::loopfunc_callback(const std_msgs::Bool loop_flag_){
  if(loop_flag_.data == true){
    orb_slam_->mpLoopCloser->CorrectLoopTag();
  }
}

void RGBDNode::tagpose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& tag_pose_){
  // ROS_WARN ("tagpose_callback...");
  
  Eigen::Affine3d transform_tag;  
  tf::poseMsgToEigen(tag_pose_->pose.pose, transform_tag);

  Eigen::Matrix4d m_tag_pose = transform_tag.matrix();

  cv::eigen2cv(m_tag_pose, Twc_tag_global);
  
  // ROS_WARN_STREAM("tagpose : " << Twc_tag_global);

  // orb_slam_loop_->Get_tagpose(Twc_tag_global);

  orb_slam_->mpLoopCloser->Get_tagpose(Twc_tag_global);
}
