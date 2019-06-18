#!/usr/bin/env python
import json
import rospy
from std_msgs.msg import String

main_path = "/home/hany/repos/Tensegrity-Robot-IU-Internship19/build/dev/"
application_name = "test"

application_full_path = main_path+application_name+"/data.json"

read_file = open(application_full_path, "r")
data = json.load(read_file)


for i in range(0,6):
    print("Controller {}: {}".format(i,data["Controllers"][i]))

def talker():
    pub = []
    for i in range(0,6):
        pub.append(rospy.Publisher('controller'+str(i), String, queue_size=10))
        #next step to create custome msg
    rospy.init_node('tenegrity_controllers', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        for i in range(0,6):
            pub[i].publish(str(data["Controllers"][i]))
        rate.sleep()
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass