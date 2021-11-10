#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"
#include "tf/transform_listener.h"
#include "filters/filter_chain.h"
#include <std_msgs/Bool.h>

class ScanToScanFilterChain
{
protected:
    // Our NodeHandle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // Components for tf::MessageFilter
    tf::TransformListener *tf_;
    message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan> *tf_filter_;
    double tf_filter_tolerance_;

    // Filter Chain
    filters::FilterChain<sensor_msgs::LaserScan> filter_chain_;

    // Components for publishing
    sensor_msgs::LaserScan msg_;
    ros::Publisher output_pub_;

    // Deprecation helpers
    ros::Timer deprecation_timer_;
    bool using_filter_chain_deprecated_;

    //swich filter status
    std_msgs::Bool switch_status;
    ros::Subscriber switch_sub_;

public:
    // Constructor
    ScanToScanFilterChain() : private_nh_("~"),
                              scan_sub_(nh_, "scan", 50),
                              tf_(NULL),
                              tf_filter_(NULL),
                              filter_chain_("sensor_msgs::LaserScan")
    {
        // Configure filter chain

        using_filter_chain_deprecated_ = private_nh_.hasParam("filter_chain");

        if (using_filter_chain_deprecated_)
            filter_chain_.configure("filter_chain", private_nh_);
        else
            filter_chain_.configure("scan_filter_chain", private_nh_);

        std::string tf_message_filter_target_frame;

        if (private_nh_.hasParam("tf_message_filter_target_frame"))
        {
            private_nh_.getParam("tf_message_filter_target_frame", tf_message_filter_target_frame);

            private_nh_.param("tf_message_filter_tolerance", tf_filter_tolerance_, 0.03);

            tf_ = new tf::TransformListener();
            tf_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(scan_sub_, *tf_, "", 50);
            tf_filter_->setTargetFrame(tf_message_filter_target_frame);
            tf_filter_->setTolerance(ros::Duration(tf_filter_tolerance_));

            // Setup tf::MessageFilter generates callback
            tf_filter_->registerCallback(boost::bind(&ScanToScanFilterChain::callback, this, _1));
        }
        else
        {
            // Pass through if no tf_message_filter_target_frame
            scan_sub_.registerCallback(boost::bind(&ScanToScanFilterChain::callback, this, _1));
        }

        // Advertise output
        output_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan_filtered", 1000);

        // Set up deprecation printout
        deprecation_timer_ = nh_.createTimer(ros::Duration(5.0), boost::bind(&ScanToScanFilterChain::deprecation_warn, this, _1));

        // set swich status
        switch_sub_ = nh_.subscribe("topic", 1000, &ScanToScanFilterChain::switchCallback, this);
    }

    // Destructor
    ~ScanToScanFilterChain()
    {
        if (tf_filter_)
            delete tf_filter_;
        if (tf_)
            delete tf_;
    }

    // Deprecation warning callback
    void deprecation_warn(const ros::TimerEvent &e)
    {
        if (using_filter_chain_deprecated_)
            ROS_WARN("Use of '~filter_chain' parameter in scan_to_scan_filter_chain has been deprecated. Please replace with '~scan_filter_chain'.");
    }

    void switchCallback(std_msgs::Bool state)
    {
        switch_status = state;
    }

    // Callback
    void callback(const sensor_msgs::LaserScan::ConstPtr &msg_in)
    {
        if (switch_status.data == true)
        {
            // Run the filter chain
            if (filter_chain_.update(*msg_in, msg_))
            {
                //only publish result if filter succeeded
                output_pub_.publish(msg_);
            }
            else
            {
                ROS_ERROR_THROTTLE(1, "Filtering the scan from time %i.%i failed.", msg_in->header.stamp.sec, msg_in->header.stamp.nsec);
            }
        }
        else
        {
            output_pub_.publish(msg_in);
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scan_to_scan_filter_chain");

    ScanToScanFilterChain t;
    ros::spin();

    return 0;
}