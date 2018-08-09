#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <sound_play/sound_play.h>

 
 int main(int argc, char **argv)
 {
   ros::init(argc, argv, "tts_cpp");
 
   ros::NodeHandle nh;
   ros::Rate loop_rate(1);
   sound_play::SoundClient sc;
 loop_rate.sleep();
   sc.say("Hello world!");
   std::cout << "Hello World!" << std::endl;

   loop_rate.sleep();

   /*while(ros::ok())
   {
    sc.say("test");
    std::cout << "test";
     ros::spinOnce();
     loop_rate.sleep();
    }*/
    return 0;
}
