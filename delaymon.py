import rospy
import numpy as np
import csv
from threading import Thread
from sensor_msgs.msg import Image as Image
from sensor_msgs.msg import Imu as imu

from numba import njit


""" cam0 = 10
cam1 = 9
cam2 = 8
cam3 = 7
imu = 6 """


class AsyncQuantifier:
  #[cam0, cam1, cam2, IMU] = [0, 0, 0, 0]
  def __init__(self):    
    self.cam0 = 0
    self.cam1 = 0
    self.cam2 = 0
    self.cam3 = 0
    self.IMU = 0
    #self.csv_file = open("time_diffs.csv", "a")
  
  def get_time_diff(self):
    self.async_quantifier()
    print(self.time_diff)
    return self.time_diff

  def get_cam0_header_timestamp(self, data):
    self.cam0 = data.header.stamp.secs+1e-9*data.header.stamp.nsecs

  def get_cam1_header_timestamp(self, data):
    self.cam1 = data.header.stamp.secs+1e-9*data.header.stamp.nsecs  

  def get_cam2_header_timestamp(self, data):
    self.cam2 = data.header.stamp.secs+1e-9*data.header.stamp.nsecs

  def get_cam3_header_timestamp(self, data):
    self.cam3 = data.header.stamp.secs+1e-9*data.header.stamp.nsecs

  def get_IMU_header_timestamp(self, data):
    self.IMU = data.header.stamp.secs+1e-9*data.header.stamp.nsecs

  def async_quantifier(self):
    tstamps = np.matrix([self.cam0, self.cam1, self.cam2, self.cam3, self.IMU])
    tstamps = np.repeat(tstamps, 5, 0)
    t_tstamps = np.transpose(tstamps)
    temp_diff = np.abs(np.triu(t_tstamps - tstamps))
    #rospy.ROS_INFO_ONCE(temp_diff)
    self.time_diff = np.concatenate((temp_diff[0][-4:], temp_diff[1][-3:], temp_diff[2][-2:], temp_diff[3][-1:]), axis=0)    
    print(self.time_diff)

    return self.time_diff
  
def device_poll(bruh):

  data = []
  with open("time_diffs.csv", '+a') as f:
    while not rospy.is_shutdown():   
      np.savetxt(f, data, delimiter=",")
      rospy.Subscriber("/left/image_raw/", Image, bruh.get_cam0_header_timestamp)
      rospy.Subscriber("/center_left/image_raw/", Image, bruh.get_cam1_header_timestamp)
      rospy.Subscriber("/center_right/image_raw/", Image, bruh.get_cam2_header_timestamp)
      rospy.Subscriber("/right/image_raw/", Image, bruh.get_cam3_header_timestamp)
      rospy.Subscriber("/vectornav/IMU", imu, bruh.get_IMU_header_timestamp)
      data.append(bruh.async_quantifier())


if __name__=='__main__':

  try:
    rospy.init_node('Quantifier', anonymous=True)

    bruh1 = AsyncQuantifier()

    thread  = Thread(target=device_poll, args=(bruh1, ))
    thread.start()
    thread.join()

  except KeyboardInterrupt:
    exit()
  

