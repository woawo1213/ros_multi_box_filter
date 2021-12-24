/*
 *  multi_box_filter.h
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *  modified: jm <woawo1213@gmail.com>
 */

#ifndef MULTIBOXFILTER_H
#define MULTIBOXFILTER_H

#include <string>
#include <vector>
#include <ros/ros.h>
#include <filters/filter_base.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <laser_geometry/laser_geometry.h>
#include <geometry_msgs/Polygon.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace laser_filters
{
    /**
     * @brief This is a filter that removes points in a laser scan inside of a cartesian box.
     */
    class Box
    {
    public:
        double x_min_, x_max_, y_min_, y_max_;

        Box(float x = 0, float y = 0, float w = 0, float h = 0)
        {
            x_max_ = x;
            y_max_ = y;
            x_min_ = x - h;
            y_min_ = y - w;
        }

        bool is_in(double px, double py)
        {
            return (x_min_ < px && px < x_max_) && (y_min_ < py && py < y_max_);
        }
    };

    class ParameterPaser
    {

    public:
        std::vector<Box> getbox_;

        ParameterPaser() {}

        static double getNumberFromXMLRPC(XmlRpc::XmlRpcValue &value, const std::string &full_param_name)
        {
            // Make sure that the value we're looking at is either a double or an int.
            if (value.getType() != XmlRpc::XmlRpcValue::TypeInt && value.getType() != XmlRpc::XmlRpcValue::TypeDouble)
            {

                std::string &value_string = value;
                ROS_FATAL("Values in the polygon specification (param %s) must be numbers. Found value %s.",
                          full_param_name.c_str(), value_string.c_str());
                throw std::runtime_error("Values in the polygon specification must be numbers");
            }
            return value.getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(value) : (double)(value);
        }

        static std::vector<Box> makeBoxFromXMLRPC(const XmlRpc::XmlRpcValue &box_xmlrpc, const std::string &full_param_name)
        {
            std::vector<Box> box;

            // Make sure we have an array of at least 1 elements.
            if (box_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeArray || box_xmlrpc.size() < 1)
            {
                ROS_FATAL("The box (parameter %s) must be specified as nested list on the parameter server with at least "
                          "1 box eg: [[x1, y1, w1, h1]]",
                          full_param_name.c_str());

                throw std::runtime_error("The box must be specified as nested list on the parameter server with at least "
                                         "1 box eg: [[x1, y1, w1, h1]]");
            }

            for (int i = 0; i < box_xmlrpc.size(); ++i)
            {
                // Make sure each element of the list is an array of size 4. (x and y and width and height coordinates)
                XmlRpc::XmlRpcValue point = box_xmlrpc[i];
                if (point.getType() != XmlRpc::XmlRpcValue::TypeArray || point.size() != 4)
                {
                    ROS_FATAL("The box (parameter %s) must be specified as list of lists on the parameter server eg: "
                              "[[x1, y1, w1, h1], ..., [xn, yn, wn, hn]], but this spec is not of that form.",
                              full_param_name.c_str());
                    throw std::runtime_error("The box must be specified as list of lists on the parameter server eg: "
                                             "[[x1, y1, w1, h1], ..., [xn, yn, wn, hn]], but this spec is not of that form");
                }

                float x = getNumberFromXMLRPC(point[0], full_param_name);
                float y = getNumberFromXMLRPC(point[1], full_param_name);
                float w = getNumberFromXMLRPC(point[2], full_param_name);
                float h = getNumberFromXMLRPC(point[3], full_param_name);

                box.push_back(Box(x, y, w, h));
            }

            return box;
        }
    };

    class MarkerPublisher
    {
        ros::Publisher pub_marker_;
        visualization_msgs::MarkerArray ma;

    public:
        MarkerPublisher()
        {
            ros::NodeHandle nh_("~");
            pub_marker_ = nh_.advertise<visualization_msgs::MarkerArray>("multi_box_marker", 1);
        }

        void initialize(std::vector<Box> &box)
        {

            geometry_msgs::Point box_points;

            for (int i = 0; i < box.size(); i++)
            {
                visualization_msgs::Marker marker_;
                marker_.header.frame_id = "base_link";
                marker_.header.stamp = ros::Time::now();
                marker_.ns = "multi_box_marker";
                marker_.id = i;
                marker_.type = visualization_msgs::Marker::LINE_STRIP;
                marker_.action = visualization_msgs::Marker::ADD;
                marker_.pose.orientation.w = 1.0;
                marker_.scale.x = 0.008;
                marker_.color.a = 1.0;
                marker_.color.b = 1.0;

                // top left
                box_points.x = box[i].x_max_;
                box_points.y = box[i].y_max_;
                marker_.points.push_back(box_points);

                // top right
                box_points.x = box[i].x_max_;
                box_points.y = box[i].y_min_;
                marker_.points.push_back(box_points);

                // bottom right
                box_points.x = box[i].x_min_;
                box_points.y = box[i].y_min_;
                marker_.points.push_back(box_points);

                // bottom right
                box_points.x = box[i].x_min_;
                box_points.y = box[i].y_max_;
                marker_.points.push_back(box_points);

                // top left
                box_points.x = box[i].x_max_;
                box_points.y = box[i].y_max_;
                marker_.points.push_back(box_points);

                ma.markers.push_back(marker_);
            }
        }

        void publish()
        {
            for (int i = 0; i < ma.markers.size(); i++)
            {
                ma.markers[i].header.stamp = stamp;
            }
            pub_marker_.publish(ma);
        }
    };

    class LaserScanMultiBoxFilter : public filters::FilterBase<sensor_msgs::LaserScan>
    {
        std::string box_frame_;
        std::string box_param_;

        laser_geometry::LaserProjection projector_;

        // tf listener to transform scans into the box_frame
        tf::TransformListener tf_;
        bool invert_filter;
        bool up_and_running_;

        ParameterPaser parser_;
        MarkerPublisher marker_pub_;

    public:
        LaserScanMultiBoxFilter() {}
        bool configure();
        bool update(
            const sensor_msgs::LaserScan &input_scan,
            sensor_msgs::LaserScan &filtered_scan);
    };
}

#endif /* multi_box_filter.h */
