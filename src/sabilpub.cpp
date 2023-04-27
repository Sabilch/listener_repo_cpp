#include "ros/ros.h"
#include "sabilpub/person_data.h"




int main (int args, char **argv) {
    ros::init(args, argv, "sabilpub");
    ros::NodeHandle n;

    ros::Publisher topic_pub = n.advertise<sabilpub::person_data>("/sabilData",1000);
    ros::Rate loop_rate(1);
    
    while(ros::ok()) {
        sabilpub::person_data person_data;
        person_data.name = "sabil";
        person_data.age = 52;
        person_data.color = "green";

        topic_pub.publish(person_data);
        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
}
