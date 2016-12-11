/** @file control_node.hpp
*  @brief File storing declarations of the robot control node.
*/

// ROS library
#include <ros/ros.h>
#include <ros/duration.h>

#include <cstdlib>

#include <guessing_game/text_to_speech.h>
#include <guessing_game/speech_to_text.h>

using namespace std;

class ControlNode{
public:
    // Constructors & Deconstructor
    ControlNode();
    ~ControlNode()
    {
    };

private:
    // Subscribers
    ros::Publisher text_sub; // Object position subscriber

    // Service clients
    ros::ServiceClient text_to_speech_client;

    // ros handler
    ros::NodeHandle nh_;
    
};
