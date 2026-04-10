#!/usr/bin/env python

import rospy,csv,os
from std_msgs.msg import String


class Recorder:
    def __init__(self):
        self.file_storage_path = rospy.get_param("~file_storage_path")
        self.init()
        self.wall_start_time = rospy.Time.now()
        self.record_csv_sub = rospy.Subscriber("/record_csv", String, self.record_csv_cb)
    
    def init(self):
        if not os.path.exists(self.file_storage_path):
            os.makedirs(self.file_storage_path)
        else:
            for file in os.listdir(self.file_storage_path):
                os.remove(self.file_storage_path + "/" + file)
        
    def record_csv_cb(self, msg):
        # record message format: "filename,data_1,data_2,..."
        msg_list = msg.data.split(",")
        filename = msg_list[0]
        time = rospy.Time.now().to_sec() - self.wall_start_time.to_sec()
        data = msg_list[1:]
        with open(self.file_storage_path + "/" + filename + ".csv", "a") as f:
            writer = csv.writer(f)
            writer.writerow([time] + data)

if __name__ == "__main__":
    rospy.init_node("recorder", anonymous=True)
    recorder = Recorder()
    rospy.spin()