
#include <assert.h>
#include <sys/time.h>
#include <unistd.h>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <nodelet/nodelet.h>
#include <list> 
#include <string>
#include <pluginlib/class_list_macros.h>
#include <stdio.h>

namespace rr_auto_dock
{
class ImageSubNodelet : public nodelet::Nodelet
{
  public:
    ImageSubNodelet()
    {}
    //: throwaway(0)

  private:
    void onInit()
    {
        NODELET_INFO("Initializing nodelet...");
        ros::NodeHandle& nh = getPrivateNodeHandle();
        image_transport::ImageTransport it(nh);

        img_sub = it.subscribe("/camera", 1, &ImageSubNodelet::imageCallback, this);
        caminfo_sub = nh.subscribe("/camera_info", 1, &ImageSubNodelet::camInfoCallback, this);
        image_pub = it.advertise("/repub_images", 1);

        NODELET_INFO("Subscribed to image and camera_info");
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        //NODELET_INFO("Got height %d", msg->height);
        //NODELET_INFO("Got image %s.", msg->encoding.c_str());
        image_pub.publish(msg);
    }

    void camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg) {
        //NODELET_INFO("Camera info Callback");
        int i;
    }

    ros::Subscriber caminfo_sub;
    image_transport::Subscriber img_sub;
    image_transport::Publisher image_pub;
};

PLUGINLIB_EXPORT_CLASS(rr_auto_dock::ImageSubNodelet, nodelet::Nodelet)
}
