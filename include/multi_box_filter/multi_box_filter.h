/*
 *  multi_box_filter.h
 * 
 *
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *  modified: jm <jmshin@wonik.com>
 */

#ifndef MULTIBOXFILTER_H
#define MULTIBOXFILTER_H

#include <string>
#include <vector>
#include <fstream>
#include <iostream>
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
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/algorithm/string/replace.hpp>

namespace laser_filters
{
/**
 * @brief This is a filter that removes points in a laser scan inside of a cartesian box.
 */
class Box
{
public:
    double x_min_, x_max_, y_min_, y_max_;

    Box(float max_x = 0, float max_y = 0, float min_x = 0, float min_y = 0) {
        x_max_ = max_x;
        y_max_ = max_y;
        x_min_ = min_x;
        y_min_ = min_y;
    }

    bool is_in(double px, double py) {
        return (x_min_ < px && px < x_max_) && (y_min_ < py && py < y_max_);
    }
};


class ParameterPaser
{
    ros::NodeHandle nh_;
    int colum = 4, cell = 2;
    float matrix[4][4][2];
    boost::property_tree::ptree pt;
public:
    ParameterPaser() {}

    void read_param(XmlRpc::XmlRpcValue& box_xmlrpc)
    {
        int row;
        std::string xml_str = box_xmlrpc;
        boost::property_tree::ptree iroot;

        row = count_row(xml_str);

        boost::replace_all(xml_str,"(","[");
        boost::replace_all(xml_str,")","]");
        xml_str="{\"target_model\":" + xml_str + "}";

        std::stringstream ss(xml_str);
        boost::property_tree::read_json(ss, iroot);

        int x_ = 0;
        for (auto &row : iroot.get_child("target_model"))
        {
            int y_ = 0;
            for (auto &colum : row.second)
            {
                int z_ = 0;
                for(auto &cell : colum.second)
                {
                    matrix[x_][y_][z_] = cell.second.get_value<float>();
                    z_++;
                }
                y_++;
            }
            x_++;
        }
    }

    std::vector<Box> generate_box(XmlRpc::XmlRpcValue& box_xmlrpc)
    {
        Box box;
        std::vector<Box> genbox;
        std::string xml_str = box_xmlrpc;

        int row;
        float margin_x = 0.1;
        float margin_y = 0.1;
        float center_offset = 0.341;

        row = count_row(xml_str);

        for (int i = 0; i < row; i++)
        {
            for (int j = 0; j < colum; j+=3)
            {
                for(int k = 0; k < cell; k++)
                {
                    if(j == 0)
                    {
                        if(k == 0)
                            box.x_max_ = matrix[i][j][k] + margin_x - center_offset;
                        else
                            box.y_max_ = matrix[i][j][k] + margin_y ;
                    }
                    else
                    {
                        if(k == 0)
                            box.x_min_ = matrix[i][j][k] - margin_x - center_offset;
                        else
                            box.y_min_ = matrix[i][j][k] - margin_y ;
                    }
                }
            }
            genbox.push_back(box);
        }
        return genbox;       
    }

    int count_row(std::string str)
    {
        int row;
        row = (count(str.begin(), str.end(), ',') / 8) + 1;
        
        return row; 
    }
    

};


class MarkerPublisher
{
    ros::Publisher pub_marker_;
    visualization_msgs::MarkerArray ma;

public:
    MarkerPublisher() 
    {   
        std::string target_model;
        ros::NodeHandle nh_("~");
        pub_marker_=nh_.advertise<visualization_msgs::MarkerArray>("multi_box_marker",1);
    }


    void initialize(std::vector<Box>& box) {

        geometry_msgs::Point box_points;

        for(int i = 0; i < box.size(); i++)
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
            marker_.lifetime=ros::Duration();

            //top left
            box_points.x = box[i].x_max_;
            box_points.y = box[i].y_max_;
            marker_.points.push_back(box_points);

            //top right
            box_points.x = box[i].x_max_;
            box_points.y = box[i].y_min_;
            marker_.points.push_back(box_points);

            //bottom right
            box_points.x = box[i].x_min_;
            box_points.y = box[i].y_min_;
            marker_.points.push_back(box_points);

            //bottom right
            box_points.x = box[i].x_min_;
            box_points.y = box[i].y_max_;
            marker_.points.push_back(box_points);

            //top left
            box_points.x = box[i].x_max_;
            box_points.y = box[i].y_max_;
            marker_.points.push_back(box_points);

            ma.markers.push_back(marker_);
        }

    }

    void publish(ros::Time stamp) 
    {
        for(int i=0;i<ma.markers.size();i++)
        {
            ma.markers[i].header.stamp=stamp;
        }
        pub_marker_.publish(ma);
    }

};



class LaserScanMultiBoxFilter : public filters::FilterBase<sensor_msgs::LaserScan>
{
    std::string box_frame_;
    std::string box_param_;
    std::vector<Box> box_;

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
        const sensor_msgs::LaserScan& input_scan,
        sensor_msgs::LaserScan& filtered_scan);

};

}

#endif /* multi_box_filter.h */
