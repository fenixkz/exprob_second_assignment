/*! 
 * \file aruco_detector.cpp 
 * \brief Node to find the markers in the environment
 * \author Ayan Mazhitov
 * \version 1.0 
 * \date 26/02/2023
 * 
 * \details
 * 
 * Description : 
 * 
 * Node to detect markers around the robot by moving the camera.
 * It request to the marker_server node to know the markrer meaning, and publishes the 
 * received informations to the nodes /markerIDs and tells others nodes that it has finished its execution by publish to a topic /isDone.
 */
#include <ros/ros.h>
#include <algorithm> 
#include <iostream>
#include <std_msgs/Bool.h>
#include <vector>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int64.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <assignment2/RoomInformation.h>
#include <assignment2/RoomConnection.h>
#include <std_msgs/Int64MultiArray.h>

#define MARKER_SIZE 0.05 ///< Size of each marker in meters
#define MARKER_N 7 ///< Number of markers to be found
/*!
 * \class ArucoDetector
 * \brief A class for detecting markers in images and publishing the results.
 * This class is able to find Aruco markers around the robot, it subscribes to image and turn data, 
 * processes the data to detect markers, and publishes the results.
*/
class ArucoDetector {

private:
    ros::NodeHandle nh_; // ROS Node handle
    
    // Aruco detector
    aruco::MarkerDetector detector_; ///< Aruco Marker Detector
    cv::Ptr<cv::aruco::Dictionary> dictionary_; ///< Dictionary of aruco pairs
    std::vector<aruco::Marker> detectedMarkers; ///< vector of detected Marker
    std::vector<int> markerIDs; ///< vector of marker IDs
    aruco::CameraParameters CamParam; ///< Camera parameters
    ros::Publisher done_pub; ///< ROS publisher to publish if the information gathering is done
    // Initialize ROS clients, subscribers and messages
    ros::ServiceClient marker_cli_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber img_sub_;
    ros::ServiceClient room_client;
    cv::Mat image_;
    ros::Publisher marker_pub;
public:
    /*!
   * \brief Constructor for ArucoDetector class.
   *
   * This constructor sets up the ROS subscribers, publishers, and service clients,
   * and initializes the marker detector. 
   */
    ArucoDetector() : it_(nh_) {
        ROS_INFO("Markers to be detected %d", MARKER_N);
        img_sub_ = it_.subscribe("/robot/camera/image_raw", 1, &ArucoDetector::imageCallback, this);
        done_pub = nh_.advertise<std_msgs::Int64>("/isDone", 1);
        room_client = nh_.serviceClient<assignment2::RoomInformation>("/room_info");
        marker_pub = nh_.advertise<std_msgs::Int64MultiArray>("/markerIDs", 10);
        ros::Duration(3).sleep();
        // Calibrate the camera
	    CamParam = aruco::CameraParameters(); 
    }    
    /*!
    * \brief Callback function for image data.
    *
    * This function processes the image data to detect markers, and stores the
    * detected marker IDs in the markerIDs vector. 
    *
    * \param msg The image data.
   */
    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        image_ = cv_ptr->image;
        // Clear already detected markers
        detectedMarkers.clear();
        // Detect the marker
        detector_.detect(image_, detectedMarkers, CamParam);
        // If at least one marker is detected
        if (detectedMarkers.size() > 0) {
            for(std::size_t i = 0; i < detectedMarkers.size(); i++){
                bool found = false;
                // check if the ID is already in vector
                for (int j = 0; j < markerIDs.size(); j++){
                    if (markerIDs[j] == detectedMarkers.at(i).id) {
                        found = true;
                        break;
                    }
                }
                // if the found ID is not in vector, add it
                if (!found){
                    markerIDs.push_back(detectedMarkers.at(i).id);
                }
	        }

        }
        // Check the IDs of the markers, if we have everything that we needed
        std::size_t k = 0;
        for (std::size_t i = 11; i < 18; i++){
            if (std::find(markerIDs.begin(), markerIDs.end(), i) != markerIDs.end()){
                k++;
            }
        }
        // If all markers have been found
        if (k == MARKER_N){
            ROS_INFO("I received all the information");
            std_msgs::Int64 msg;
            msg.data = 1;
            // Publish the message that we are done
            done_pub.publish(msg);
            std_msgs::Int64MultiArray msg_marker;
            msg_marker.data.clear();
            msg_marker.data.insert(msg_marker.data.end(), markerIDs.begin(), markerIDs.end());
            ros::Rate r(100);
            // Publish the IDs of markers to the topic
            for (int i = 0; i < 100; i++){
                marker_pub.publish(msg_marker);
                r.sleep();
            }
            

            // ROS shutdown
            ros::shutdown();
        }
    }

};
/*!
 * \brief Main function for the aruco_detector node.
 *
 * This function creates an instance of the ArucoDetector class.
 * It then enters a loop where it sends camera movement commands and spins the ROS event
 * loop until all markers have been detected.
 *
 *
 * \param argc The number of command line arguments.
 * \param argv The array of command line arguments.
 *
 * \return 0 on success, non-zero otherwise.
 */
int main(int argc, char **argv) {
    ros::init(argc, argv, "aruco_detector");
    ArucoDetector node;
    ros::Rate loop_rate(50);
    while(ros::ok()) {
    	ros::spinOnce();
    	loop_rate.sleep();
    }
    return 0;
}
