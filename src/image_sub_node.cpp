
#include <assert.h>
#include <sys/time.h>
#include <unistd.h>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <list>
#include <string>

class ImageSubNode {
  private:
    ros::Subscriber caminfo_sub;
    image_transport::ImageTransport it;
    image_transport::Subscriber img_sub;

    // if set, we publish the images that contain fiducials
    bool publish_images;
    image_transport::Publisher image_pub;

    bool haveCamInfo;
    int frameNum; 

    void imageCallback(const sensor_msgs::ImageConstPtr &msg);
    void camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &msg);

  public:
    ImageSubNode(ros::NodeHandle &nh);
};


void ImageSubNode::camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
    //ROS_WARN("Camera info Callback");
    int i;
}

void ImageSubNode::imageCallback(const sensor_msgs::ImageConstPtr & msg) {
    //ROS_INFO("Got image %d. Republishing", msg->header.seq);
    //sensor_msgs::ImagePtr msg2 = msg;
    image_pub.publish(msg);
}

ImageSubNode::ImageSubNode(ros::NodeHandle & nh) : it(nh)
{
    img_sub = it.subscribe("/camera", 1, &ImageSubNode::imageCallback, this);

    caminfo_sub = nh.subscribe("/camera_info", 1, &ImageSubNode::camInfoCallback, this);
    image_pub = it.advertise("/repub_images", 1);

    ROS_INFO("Subscribed to image and camera_info");
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "image_sub_node");
    ros::NodeHandle nh("~");

    ImageSubNode * node = new ImageSubNode(nh);

    ros::spin();

    return 0;
}
