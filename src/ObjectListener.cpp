#include <ObjectListener.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_listener");

    cleaner::ObjectListener listener;
    ros::spin();

    return 0;
}

