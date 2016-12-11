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

class GuessingGame{
public:
    // Constructors & Deconstructor
    GuessingGame();
    ~GuessingGame()
    {
    };

private:
    // ros handler
    ros::NodeHandle nh_;

    // Service clients
    ros::ServiceClient text_to_speech_client;
    ros::ServiceClient speech_to_text_client;
    ros::ServiceClient image_request_client;

    
};
