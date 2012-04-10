/*!
 * \file nao_terminal_talk.cpp
 * \brief Control of the NAO's text-to-speech module over the terminal
 *
 * nao_terminal_talk allows you to type text into the terminal and send it to the NAO's text-to-speech module.
 * Volume and language control is also enabled in this node with use of the volume() and lang() commands.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date December 1, 2011
 */

#include <iostream>
#include <nao_tools/nao_terminal_talk.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <stdio.h>
#include <string>

using namespace std;

nao_terminal_talk::nao_terminal_talk()
{
  // create the published topics
  speech = node.advertise<std_msgs::String> ("nao_say", 1);
  volume = node.advertise<std_msgs::Float32> ("nao_set_volume", 1);
  lang = node.advertise<std_msgs::String> ("nao_set_lang", 1);
  // set with the 'exit()' command
  exit_flag = false;

  ROS_INFO("NAO Terminal Talk Started");

  // print the exit information to the terminal
  cout << "Enter text for the NAO to speak. "
      << "To exit this node, use the 'exit()' command." << endl;
}

void nao_terminal_talk::process_command()
{
  // print out the start of the line
  cout << NAO_TERMINAL_PREFIX;

  // wait for the command
  string cmd;
  float v;
  getline(cin, cmd);
  // language buffer
  char l[cmd.size()];

  // check if it is the exit command
  if (cmd.compare(EXIT_COMMAND) == 0)
    exit_flag = true;
  else if (sscanf(cmd.c_str(), VOLUME_COMMAND, &v) == 1)
  {
    // send the volume command
    std_msgs::Float32 vol;
    vol.data = v;

    volume.publish(vol);
  }
  else if (sscanf(cmd.c_str(), LANG_COMMAND, l) == 1)
  {
    //remove the trailing ')'
    l[strlen(l)-1] = '\0';

    // send the language command
    std_msgs::String language;
    string tmp(l);
    language.data = tmp;
    ROS_INFO("%s", tmp.c_str());
    ROS_INFO("%s", l);
    lang.publish(language);
  }
  else
  {
    // send the text to the nao_ctrl node
    std_msgs::String msg;
    msg.data = cmd;

    speech.publish(msg);
  }
}

bool nao_terminal_talk::exit()
{
  // just return the flag
  return exit_flag;
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "nao_terminal_talk");

  // let any other nodes get up and running
  sleep(2);

  // initialize the controller
  nao_terminal_talk talk;

  /*
   * Continue until a ctrl-c has occurred.
   * Note that since process_command blocks, a ctrl-c will only be
   * processed after each command is handled. Alternatively, an
   * 'exit()' command can be used to quit.
   */
  while (ros::ok() && !talk.exit())
  {
    talk.process_command();
    // for good practice
    ros::spinOnce();
  }
}
