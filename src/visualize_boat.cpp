//
// Created by giuseppe on 07/06/18.
//

#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose2D.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>



class BoatMarker
{
public:

    visualization_msgs::Marker marker;
    ros::Publisher marker_pub;
    ros::Subscriber marker_sub;
    uint32_t  shape;

    void visualizerCallback(const geometry_msgs::Pose2D::ConstPtr& );


    BoatMarker(ros::NodeHandle nh)
    {
        // Set our shape to a arrow
        shape = visualization_msgs::Marker::ARROW;

        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "/my_frame";

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "boat_shape";
        marker.id = 0;

        // Set the marker type.
        marker.type = shape;

        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::Marker::ADD;


        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 6.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;

        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        // Never autodelete the marker
        marker.lifetime = ros::Duration();

        // Advertise the marker
        marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

        // Subscribe to the boat position
        marker_sub = nh.subscribe("boat_position", 1000, &BoatMarker::visualizerCallback, this);
        //marker_sub = nh.subscribe("boat_talker/boat_position", 1000, &emptyCallback);

    }


    void Publish()
    {
        marker_pub.publish(marker);
    }

};

void BoatMarker::visualizerCallback(const geometry_msgs::Pose2D::ConstPtr& pose)
{
    // Time stamp
    marker.header.stamp = ros::Time::now();

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = pose->x;
    marker.pose.position.y = pose->y;
    marker.pose.position.z = 0;
    tf::Quaternion q = tf::createQuaternionFromYaw(pose->theta);
    marker.pose.orientation.x = q.getX();
    marker.pose.orientation.y = q.getY();
    marker.pose.orientation.z = q.getZ();
    marker.pose.orientation.w = q.getW();
}


int main( int argc, char** argv )
{
    ros::init(argc, argv, "basic_shapes");
    ros::NodeHandle n;
    ros::Rate r(20);

    BoatMarker bm(n);

    while (ros::ok())
    {

        /* Publish the marker
        while (bm.marker_pub.getNumSubscribers() < 1)
        {
            if (!ros::ok())
            {
                return 0;
            }
            ROS_WARN_ONCE("Please create a subscriber to the marker");
            sleep(1);
        }
        */
        bm.Publish();

        ros::spinOnce();
        r.sleep();
    }
}