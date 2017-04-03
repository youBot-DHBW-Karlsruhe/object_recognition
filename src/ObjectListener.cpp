#include <ObjectListener.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_listener");

    ros::NodeHandle nh;
    cleaner::ObjectListener listener(nh, "/base_link");
    ros::spin();

    return 0;
}

