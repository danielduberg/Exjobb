#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <string>

namespace camera_sync {

class SyncNodelet : public nodelet::Nodelet {
public:
    SyncNodelet() : synced(0), missed(0) {}

    ~SyncNodelet() {}

private:
    int synced;
    int missed;

    // Subscriptions
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_left, sub_right;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximatePolicy;
    typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
    boost::shared_ptr<ApproximateSync> approximate_sync;

    // Publications
    ros::Publisher pub_reset, pub_sync;

    virtual void onInit();

    void callback(const sensor_msgs::CameraInfo::ConstPtr & left_msg, const sensor_msgs::CameraInfo::ConstPtr & right_msg);
};

void SyncNodelet::onInit() {
    ros::NodeHandle nh(getNodeHandle());
    ros::NodeHandle priv_nh(getPrivateNodeHandle());

    sub_left.subscribe(nh, "left/camera_info", 1);
    sub_right.subscribe(nh, "right/camera_info", 1);

    approximate_sync.reset(new ApproximateSync(ApproximatePolicy(2), sub_left, sub_right));
    approximate_sync->registerCallback(boost::bind(&SyncNodelet::callback, this, _1, _2));

    pub_reset = nh.advertise<std_msgs::Bool>("right/reset", 1);
    pub_sync = nh.advertise<std_msgs::Float32>("diff", 1);
}

void SyncNodelet::callback(const sensor_msgs::CameraInfo::ConstPtr & left_msg, const sensor_msgs::CameraInfo::ConstPtr & right_msg) {
    int limit = 20000000;
    if (synced > 3) {
        limit = 33000000;
    }

    if (std::abs(left_msg->header.stamp.nsec - right_msg->header.stamp.nsec) < limit) {
        synced++;
    } else {
        missed++;
    }

    if (missed > 3) {
        std_msgs::Bool out;
        out.data = true;
        pub_reset.publish(out);

        synced = 0;
        missed = 0;
    } else if (synced > 10) {
        synced = 0;
        missed = 0;
    }

    std_msgs::Float32 out;
    out.data = std::abs(left_msg->header.stamp.nsec - right_msg->header.stamp.nsec) * 0.000001; // Seconds
    pub_sync.publish(out);
}

}

// Register this plugin with pluginlib.
//
// parameters are: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(camera_sync, Sync, camera_sync::SyncNodelet, nodelet::Nodelet);
