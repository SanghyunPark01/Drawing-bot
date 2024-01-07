#include <utility.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pub_img");
  ros::NodeHandle n;

  ros::Publisher imagePub_ = n.advertise<sensor_msgs::Image>("/test_image", 1000);

  ros::Rate loop_rate(30);

  cv::Mat icTestImage;
  icTestImage = cv::imread("/home/psh/kw_ws/src/planning_image_drawing/test_image/test.jpg");
  cv::resize(icTestImage,icTestImage,cv::Size(icTestImage.cols*0.25,icTestImage.rows*0.25));

  cv_bridge::CvImage img_bridge;
  sensor_msgs::Image pubImg; // >> message to be sent
  std_msgs::Header header;
  while (ros::ok())
  {
    header.stamp = ros::Time::now(); // time
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, icTestImage);
    img_bridge.toImageMsg(pubImg); // from cv_bridge to sensor_msgs::Image
    imagePub_.publish(pubImg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}