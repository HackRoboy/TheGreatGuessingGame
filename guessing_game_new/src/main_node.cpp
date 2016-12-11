/** @file central_node.cpp
*  @brief File storing definitions of the state machine functions.
*/

#include <guessing_game.hpp>

using namespace std;

GuessingGame::GuessingGame()
{
  // Create the service client
  text_to_speech_client = nh.serviceClient<text_to_speech>("text_to_speech");
  // Subscribe to the text
  speech_to_text_client = nh.serviceClient<speech_to_text>("speech_to_text");
}

bool text_to_speech(imitation_game::text_to_speech::Request  &req, imitation_game::text_to_speech::Response &res)
{
  string sentence = text[req.id];

  text_to_speech msg;
  msg.req = sentence;

  // Create the message and call service
  text_to_speech_client.call()
  return true;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "guessing_game");
  GuessingGame cn;
  ros::spin();
  sleep(1);
  return 0;
}
