####################################################################################
# bag file
# 201009_Multi_Project/UV_bag/2020-10-09-10-09-11.bag (from 400s)
####################################################################################

# Basic
import numpy as np
import math
import sys
import time
import os
import datetime
import cv2
import copy

# PySide2
from PySide2 import QtCore
from PySide2.QtWidgets import QApplication, QMainWindow, QLabel
from PySide2.QtCore import QFile, QTimer, SIGNAL
from PySide2.QtGui import QPixmap, QImage
from widget import Ui_Widget

# ROS
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from ads_b_read.msg import Traffic_Report_Array

#######################################
# Define Parameter
WGS84_a_m = 6378137.0
WGS84_e = 0.08181919

d2r = math.pi / 180
r2d = 180 / math.pi

Ownship_ICAO = 7463479

#check
std_lat = 36.917204 #917342
std_lon = 126.920288 #920201
std_alt = 1000

scale_500m = 0.24444444444
scale_200m = 0.61111111111
scale_100m = 1.222222222222
scale_50m = 2.444444444444
scale_20m = 6.111111111111

map_row = 910 
map_col = 1140

icon_row_col = 26
#######################################
# From PyeongTaek
std_sinLat = math.sin(std_lat * d2r)
std_sinLon = math.sin(std_lon * d2r)
std_cosLat = math.cos(std_lat * d2r)
std_cosLon = math.cos(std_lon * d2r)

N = WGS84_a_m / math.sqrt(1 - WGS84_e * WGS84_e * std_sinLat * std_sinLat)
ref_ECEF_x = (N + std_alt) * std_cosLat * std_cosLon
ref_ECEF_y = (N + std_alt) * std_cosLat * std_sinLon
ref_ECEF_z = (N * (1 - WGS84_e * WGS84_e) + std_alt) * std_sinLat

#######################################
# help function
def read_img(img_path, row, col):
    img = cv2.imread(img_path, cv2.IMREAD_COLOR)
    img = cv2.resize(img, dsize=(col, row), interpolation=cv2.INTER_AREA)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    return img

def image_add_function(background, add_image, roi_row, roi_col):
    height, width, channel = add_image.shape
    check_image_size = int(height/2)

    roi = background[roi_row - check_image_size:roi_row + check_image_size, roi_col - check_image_size : roi_col + check_image_size]

    img2gray = cv2.cvtColor(add_image, cv2.COLOR_BGR2GRAY)
    ret, mask = cv2.threshold(img2gray, 10, 255, cv2.THRESH_BINARY)
    mask_inv = cv2.bitwise_not(mask)

    img1_bg = cv2.bitwise_and(roi, roi, mask = mask_inv)
    img2_fg = cv2.bitwise_and(add_image, add_image, mask = mask)

    dst = cv2.add(img1_bg,img2_fg)
    ret_image = copy.copy(background)
    ret_image[roi_row - check_image_size:roi_row + check_image_size, roi_col - check_image_size : roi_col + check_image_size] = dst
    return ret_image

class ros_class():
    def __init__(self):
        self.ROS_TIME = 0

        ####################################
        # Pos information (LLA)
        self.Ownship_lat, self.Ownship_lon, self.Ownship_alt = 0, 0, 0
        self.USV1_lat, self.USV1_lon, self.USV1_alt = 0, 0, 0
        self.USV2_lat, self.USV2_lon, self.USV2_alt = 0, 0, 0
        self.UAV1_lat, self.UAV1_lon, self.UAV1_alt = 0, 0, 0
        self.UAV2_lat, self.UAV2_lon, self.UAV2_alt = 0, 0, 0

        ####################################
        # ECEF Pos
        self.Ownship_ECEF_x, self.Ownship_ECEF_y, self.Ownship_ECEF_z = 0, 0, 0
        self.USV1_ECEF_x, self.USV1_ECEF_y, self.USV1_ECEF_z = 0, 0, 0
        self.USV2_ECEF_x, self.USV2_ECEF_y, self.USV2_ECEF_z = 0, 0, 0
        self.UAV1_ECEF_x, self.UAV1_ECEF_y, self.UAV1_ECEF_z = 0, 0, 0
        self.UAV2_ECEF_x, self.UAV2_ECEF_y, self.UAV2_ECEF_z = 0, 0, 0

        ####################################
        # Local Pos (Origin position is Hansae Univ)
        self.Ownship_local_x, self.Ownship_local_y, self.Ownship_local_z = 0, 0, 0
        self.USV1_local_x, self.USV1_local_y, self.USV1_local_z = 0, 0, 0
        self.USV2_local_x, self.USV2_local_y, self.USV2_local_z = 0, 0, 0
        self.UAV1_local_x, self.UAV1_local_y, self.UAV1_local_z = 0, 0, 0
        self.UAV2_local_x, self.UAV2_local_y, self.UAV2_local_z = 0, 0, 0

        ####################################
        # Heading / Hor / Ver Velocity
        self.Ownship_heading, self.USV1_heading,self.USV2_heading, self.UAV1_heading, self.UAV2_heading = 0, 0, 0, 0, 0
        self.Ownship_hor_vel, self.USV1_hor_vel, self.USV2_hor_vel, self.UAV1_hor_vel, self.UAV2_hor_vel = 0, 0, 0, 0, 0
        self.Ownship_ver_vel, self.USV1_ver_vel, self.USV2_ver_vel, self.UAV1_ver_vel, self.UAV2_ver_vel = 0, 0, 0, 0, 0

        ####################################
        # FOR TEST
        # self.UAV1_lat = std_lat
        # self.UAV1_lon = std_lon

        # self.UAV2_lat = std_lat + 0.001
        # self.UAV2_lon = std_lon

        # self.USV1_lat = std_lat + 0.002
        # self.USV1_lon = std_lon

        # self.USV2_lat = std_lat + 0.003
        # self.USV2_lon = std_lon

        # self.Ownship_lat = std_lat + 0.004
        # self.Ownship_lon = std_lon

        # self.USV1_ECEF_x, self.USV1_ECEF_y, self.USV1_ECEF_z = self.lla_to_ECEF(self.USV1_lat, self.USV1_lon, self.USV1_alt)
        # self.USV1_local_x, self.USV1_local_y, self.USV1_local_z = self.ECEF_to_Local(self.USV1_ECEF_x, self.USV1_ECEF_y, self.USV1_ECEF_z)

        # self.USV2_ECEF_x, self.USV2_ECEF_y, self.USV2_ECEF_z = self.lla_to_ECEF(self.USV2_lat, self.USV2_lon, self.USV2_alt)
        # self.USV2_local_x, self.USV2_local_y, self.USV2_local_z = self.ECEF_to_Local(self.USV2_ECEF_x, self.USV2_ECEF_y, self.USV2_ECEF_z)

        # self.UAV1_ECEF_x, self.UAV1_ECEF_y, self.UAV1_ECEF_z = self.lla_to_ECEF(self.UAV1_lat, self.UAV1_lon, self.UAV1_alt)
        # self.UAV1_local_x, self.UAV1_local_y, self.UAV1_local_z = self.ECEF_to_Local(self.UAV1_ECEF_x, self.UAV1_ECEF_y, self.UAV1_ECEF_z)

        # self.UAV2_ECEF_x, self.UAV2_ECEF_y, self.UAV2_ECEF_z = self.lla_to_ECEF(self.UAV2_lat, self.UAV2_lon, self.UAV2_alt)
        # self.UAV2_local_x, self.UAV2_local_y, self.UAV2_local_z = self.ECEF_to_Local(self.UAV2_ECEF_x, self.UAV2_ECEF_y, self.UAV2_ECEF_z)

        # self.Ownship_ECEF_x, self.Ownship_ECEF_y, self.Ownship_ECEF_z = self.lla_to_ECEF(self.Ownship_lat, self.Ownship_lon, self.Ownship_alt)
        # self.Ownship_local_x, self.Ownship_local_y, self.Ownship_local_z = self.ECEF_to_Local(self.Ownship_ECEF_x, self.Ownship_ECEF_y, self.Ownship_ECEF_z)

    def lla_to_ECEF(self, lat, lon, alt):
        sinLat = math.sin(lat * d2r)
        sinLon = math.sin(lon * d2r)
        cosLat = math.cos(lat * d2r)
        cosLon = math.cos(lon * d2r)

        N = WGS84_a_m / math.sqrt(1 - WGS84_e * WGS84_e * sinLat * sinLat)
        ECEF_x = (N + alt) * cosLat * cosLon
        ECEF_y = (N + alt) * cosLat * sinLon
        ECEF_z = (N * (1 - WGS84_e * WGS84_e) + alt) * sinLat
        return [ECEF_x, ECEF_y, ECEF_z]

    def ECEF_to_Local(self, x, y, z):
        ROT_MAT = [ [-std_sinLat * std_cosLon,       -std_sinLat * std_sinLon,        std_cosLat],
                    [             -std_sinLon,                     std_cosLon,               0.0],
                    [-std_cosLat * std_cosLon,       -std_cosLat * std_sinLon,       -std_sinLat]]

        diff_ECEF = [x - ref_ECEF_x, y - ref_ECEF_y, z - ref_ECEF_z]

        local_pos = np.matmul(np.array(ROT_MAT), np.array(diff_ECEF))
        return local_pos

    def ADS_callback(self, data):
        Traffic_Reports_list = data.Traffic_Reports

        for i in range(len(Traffic_Reports_list)):
            if (Traffic_Reports_list[i].ICAO_address == Ownship_ICAO):
                self.Ownship_lat = Traffic_Reports_list[i].lat / 10000000.0
                self.Ownship_lon = Traffic_Reports_list[i].lon / 10000000.0
                self.Ownship_alt = Traffic_Reports_list[i].altitude / 1000.0
                self.Ownship_heading = Traffic_Reports_list[i].heading / 100.0
                self.Ownship_hor_vel = Traffic_Reports_list[i].hor_velocity / 100.0
                self.Ownship_ver_vel = Traffic_Reports_list[i].ver_velocity / 100.0
                self.Ownship_ECEF_x, self.Ownship_ECEF_y, self.Ownship_ECEF_z = self.lla_to_ECEF(self.Ownship_lat, self.Ownship_lon, self.Ownship_alt)
                self.Ownship_local_x, self.Ownship_local_y, self.Ownship_local_z = self.ECEF_to_Local(self.Ownship_ECEF_x, self.Ownship_ECEF_y, self.Ownship_ECEF_z)

                self.Ownship_local_x = round(self.Ownship_local_x, 3)
                self.Ownship_local_y = round(self.Ownship_local_y, 3)
                self.Ownship_local_z = round(self.Ownship_local_z, 3)
                
    def USV1_callback(self, msg):
        data = msg.data
        self.USV1_lat = round(data[0], 6)
        self.USV1_lon = round(data[1], 6)
        self.USV1_alt = round(data[2], 2)
        self.USV1_heading = round(data[3])
        self.USV1_hor_vel = round(data[4], 2)
        self.USV1_ver_vel = round(data[5], 2)

        self.USV1_ECEF_x, self.USV1_ECEF_y, self.USV1_ECEF_z = self.lla_to_ECEF(self.USV1_lat, self.USV1_lon, self.USV1_alt)
        self.USV1_local_x, self.USV1_local_y, self.USV1_local_z = self.ECEF_to_Local(self.USV1_ECEF_x, self.USV1_ECEF_y, self.USV1_ECEF_z)
        
        self.USV1_local_x = round(self.USV1_local_x, 3)
        self.USV1_local_y = round(self.USV1_local_y, 3)
        self.USV1_local_z = round(self.USV1_local_z, 3)

    # MOLIN
    def USV2_callback(self, msg):
        data = msg.data
        self.USV2_lat = round(data[1], 6)
        self.USV2_lon = round(data[0], 6)
        self.USV2_alt = round(data[2], 2)
        self.USV2_heading = round(data[3])
        self.USV2_hor_vel = round(data[4], 2)
        self.USV2_ver_vel = round(data[5], 2)

        self.USV2_ECEF_x, self.USV2_ECEF_y, self.USV2_ECEF_z = self.lla_to_ECEF(self.USV2_lat, self.USV2_lon, self.USV2_alt)
        self.USV2_local_x, self.USV2_local_y, self.USV2_local_z = self.ECEF_to_Local(self.USV2_ECEF_x, self.USV2_ECEF_y, self.USV2_ECEF_z)
        
        self.USV2_local_x = round(self.USV2_local_x, 3)
        self.USV2_local_y = round(self.USV2_local_y, 3)
        self.USV2_local_z = round(self.USV2_local_z, 3)

    def UAV1_callback(self, msg):
        data = msg.data
        self.UAV1_lat = round(data[0], 6)
        self.UAV1_lon = round(data[1], 6)
        self.UAV1_alt = round(data[2], 2)
        self.UAV1_heading = round(data[3])
        self.UAV1_hor_vel = round(math.sqrt(data[4]*data[4] + data[5]*data[5]), 2)
        self.UAV1_ver_vel = round(data[6], 2)

        self.UAV1_ECEF_x, self.UAV1_ECEF_y, self.UAV1_ECEF_z = self.lla_to_ECEF(self.UAV1_lat, self.UAV1_lon, self.UAV1_alt)
        self.UAV1_local_x, self.UAV1_local_y, self.UAV1_local_z = self.ECEF_to_Local(self.UAV1_ECEF_x, self.UAV1_ECEF_y, self.UAV1_ECEF_z)
        
        self.UAV1_local_x = round(self.UAV1_local_x, 3)
        self.UAV1_local_y = round(self.UAV1_local_y, 3)
        self.UAV1_local_z = round(self.UAV1_local_z, 3)

    def UAV2_callback(self, msg):
        data = msg.data
        self.UAV2_lat = round(data[0], 6)
        self.UAV2_lon = round(data[1], 6)
        self.UAV2_alt = round(data[2], 2)
        self.UAV2_heading = round(data[3])
        self.UAV2_hor_vel = round(math.sqrt(data[4]*data[4] + data[5]*data[5]), 2)
        self.UAV2_ver_vel = round(data[6], 2)

        self.UAV2_ECEF_x, self.UAV2_ECEF_y, self.UAV2_ECEF_z = self.lla_to_ECEF(self.UAV2_lat, self.UAV2_lon, self.UAV2_alt)
        self.UAV2_local_x, self.UAV2_local_y, self.UAV2_local_z = self.ECEF_to_Local(self.UAV2_ECEF_x, self.UAV2_ECEF_y, self.UAV2_ECEF_z)
        
        self.UAV2_local_x = round(self.UAV2_local_x, 3)
        self.UAV2_local_y = round(self.UAV2_local_y, 3)
        self.UAV2_local_z = round(self.UAV2_local_z, 3)

    def timer_callback(self, data):
        self.ROS_TIME = data.header.stamp.secs + (data.header.stamp.nsecs) * 0.000000001
        self.ROS_TIME = math.floor(self.ROS_TIME * 10) / 10

ros = ros_class()
class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.ui = Ui_Widget()
        self.ui.setupUi(self)

        ####################################
        # Check aircraft is in MAP
        self.check_Ownship_in_map = 0
        self.check_USV1_in_map = 0
        self.check_USV2_in_map = 0
        self.check_UAV1_in_map = 0
        self.check_UAV2_in_map = 0

        ####################################
        # Shared Pose
        self.shared_lat_pos = 0
        self.shared_lon_pos = 0

        self.shared_local_x = 0
        self.shared_local_y = 0

        self.pos_sharing = 0
        
        ####################################
        # Button Interactive
        self.ui.map_change_Down.clicked.connect(self.map_change_down_clicked)
        self.ui.map_change.clicked.connect(self.mpa_change_clicked)

        self.ui.OPV_pos_share.clicked.connect(self.OPV_pos_share_clicked)
        self.ui.OPV_shared_pos_clear.clicked.connect(self.OPV_shared_pos_clear_clicked)

        self.ui.OPV_stby_button.clicked.connect(self.OPV_stby_clicked)
        self.ui.OPV_done_button.clicked.connect(self.OPV_done_clicked)
        self.ui.OPV_mission_button.clicked.connect(self.OPV_mission_clicked)

        self.ui.USV1_stby_button.clicked.connect(self.USV1_stby_clicked)
        self.ui.USV1_done_button.clicked.connect(self.USV1_done_clicked)
        self.ui.USV1_mission_button.clicked.connect(self.USV1_mission_clicked)

        self.ui.USV2_stby_button.clicked.connect(self.USV2_stby_clicked)
        self.ui.USV2_done_button.clicked.connect(self.USV2_done_clicked)
        self.ui.USV2_mission_button.clicked.connect(self.USV2_mission_clicked)

        self.ui.UAV1_stby_button.clicked.connect(self.UAV1_stby_clicked)
        self.ui.UAV1_done_button.clicked.connect(self.UAV1_done_clicked)
        self.ui.UAV1_mission_button.clicked.connect(self.UAV1_mission_clicked)

        self.ui.UAV2_stby_button.clicked.connect(self.UAV2_stby_clicked)
        self.ui.UAV2_done_button.clicked.connect(self.UAV2_done_clicked)
        self.ui.UAV2_mission_button.clicked.connect(self.UAV2_mission_clicked)

        ####################################
        # Img Path
        self.map_20_path = "img/MAP_20.png"
        self.map_50_path = "img/MAP_50.png"
        self.map_100_path = "img/MAP_100.png"
        self.map_200_path = "img/MAP_200.png"
        self.map_500_path = "img/MAP_500.png"
        self.Scale_factor = scale_100m

        self.ownship_img_path = "img/icon/Ownship Symbol.png"
        self.USV1_img_path = "img/icon/Ship_logo.png"
        self.USV2_img_path = "img/icon/Ship_logo2.png"
        self.UAV1_img_path = "img/icon/drone1_logo.png"
        self.UAV2_img_path = "img/icon/drone2_logo.png"

        self.stby_img_path = "img/stby.png"
        self.mission_img_path = "img/mission.png"
        self.done_img_path = "img/done.png"
        self.error_img_path = "img/error.png"
        self.connect_img_path = "img/connect.png"
        self.disconnect_img_path = "img/disconnected.png"
        
        ####################################
        # Map Information
        self.map_size = 100 # 20, 50, 100
        self.map_img = read_img(self.map_100_path, map_row, map_col)

        # Status Initialize
        pixmap = QPixmap(QImage(self.disconnect_img_path))
        pixmap = pixmap.scaled(100, 20, QtCore.Qt.KeepAspectRatio) 
        self.ui.OPV_Status.setPixmap(pixmap)

        pixmap = QPixmap(QImage(self.disconnect_img_path))
        pixmap = pixmap.scaled(100, 20, QtCore.Qt.KeepAspectRatio) 
        self.ui.USV1_Status.setPixmap(pixmap)

        pixmap = QPixmap(QImage(self.disconnect_img_path))
        pixmap = pixmap.scaled(100, 20, QtCore.Qt.KeepAspectRatio) 
        self.ui.USV2_Status.setPixmap(pixmap)

        pixmap = QPixmap(QImage(self.disconnect_img_path))
        pixmap = pixmap.scaled(100, 20, QtCore.Qt.KeepAspectRatio) 
        self.ui.UAV1_Status.setPixmap(pixmap)

        pixmap = QPixmap(QImage(self.disconnect_img_path))
        pixmap = pixmap.scaled(100, 20, QtCore.Qt.KeepAspectRatio) 
        self.ui.UAV2_Status.setPixmap(pixmap)

        ####################################
        # Update GUI
        self.timer = QTimer()
        self.connect(self.timer, SIGNAL("timeout()"), self.visual_UI)
        self.timer.start(50)

    # OPV find human, shared pos
    def OPV_pos_share_clicked(self):
        self.shared_lat_pos = ros.Ownship_lat
        self.shared_lon_pos = ros.Ownship_lon

        self.shared_local_x = ros.Ownship_local_x
        self.shared_local_y = ros.Ownship_local_y

        self.ui.OPV_share_lat.setText(str(self.shared_lat_pos))
        self.ui.OPV_share_lon.setText(str(self.shared_lon_pos))

        self.pos_sharing = 1
    
    def OPV_shared_pos_clear_clicked(self):
        self.pos_sharing = 0

        self.ui.OPV_share_lat.setText(str(0.0))
        self.ui.OPV_share_lon.setText(str(0.0))

        self.ui.OPV_acc_lat.setText(str(0.0))
        self.ui.OPV_acc_lon.setText(str(0.0))

        self.ui.USV1_acc_lat.setText(str(0.0))
        self.ui.USV1_acc_lon.setText(str(0.0))

        self.ui.USV2_acc_lat.setText(str(0.0))
        self.ui.USV2_acc_lon.setText(str(0.0))

        self.ui.UAV1_acc_lat.setText(str(0.0))
        self.ui.UAV1_acc_lon.setText(str(0.0))

        self.ui.UAV2_acc_lat.setText(str(0.0))
        self.ui.UAV2_acc_lon.setText(str(0.0))

    # OPV Status
    def OPV_stby_clicked(self):
        pixmap = QPixmap(QImage(self.stby_img_path))
        pixmap = pixmap.scaled(100, 20, QtCore.Qt.KeepAspectRatio) 
        self.ui.OPV_Status.setPixmap(pixmap)

    def OPV_mission_clicked(self):
        pixmap = QPixmap(QImage(self.mission_img_path))
        pixmap = pixmap.scaled(100, 20, QtCore.Qt.KeepAspectRatio) 
        self.ui.OPV_Status.setPixmap(pixmap)

    def OPV_done_clicked(self):
        pixmap = QPixmap(QImage(self.done_img_path))
        pixmap = pixmap.scaled(100, 20, QtCore.Qt.KeepAspectRatio) 
        self.ui.OPV_Status.setPixmap(pixmap)

    # USV1 Status
    def USV1_stby_clicked(self):
        pixmap = QPixmap(QImage(self.stby_img_path))
        pixmap = pixmap.scaled(100, 20, QtCore.Qt.KeepAspectRatio) 
        self.ui.USV1_Status.setPixmap(pixmap)

    def USV1_mission_clicked(self):
        pixmap = QPixmap(QImage(self.mission_img_path))
        pixmap = pixmap.scaled(100, 20, QtCore.Qt.KeepAspectRatio) 
        self.ui.USV1_Status.setPixmap(pixmap)

    def USV1_done_clicked(self):
        pixmap = QPixmap(QImage(self.done_img_path))
        pixmap = pixmap.scaled(100, 20, QtCore.Qt.KeepAspectRatio) 
        self.ui.USV1_Status.setPixmap(pixmap)

    # USV2 Status
    def USV2_stby_clicked(self):
        pixmap = QPixmap(QImage(self.stby_img_path))
        pixmap = pixmap.scaled(100, 20, QtCore.Qt.KeepAspectRatio) 
        self.ui.USV2_Status.setPixmap(pixmap)

    def USV2_mission_clicked(self):
        pixmap = QPixmap(QImage(self.mission_img_path))
        pixmap = pixmap.scaled(100, 20, QtCore.Qt.KeepAspectRatio) 
        self.ui.USV2_Status.setPixmap(pixmap)

    def USV2_done_clicked(self):
        pixmap = QPixmap(QImage(self.done_img_path))
        pixmap = pixmap.scaled(100, 20, QtCore.Qt.KeepAspectRatio) 
        self.ui.USV2_Status.setPixmap(pixmap)

    # UAV1 Status
    def UAV1_stby_clicked(self):
        pixmap = QPixmap(QImage(self.stby_img_path))
        pixmap = pixmap.scaled(100, 20, QtCore.Qt.KeepAspectRatio) 
        self.ui.UAV1_Status.setPixmap(pixmap)

    def UAV1_mission_clicked(self):
        pixmap = QPixmap(QImage(self.mission_img_path))
        pixmap = pixmap.scaled(100, 20, QtCore.Qt.KeepAspectRatio) 
        self.ui.UAV1_Status.setPixmap(pixmap)

    def UAV1_done_clicked(self):
        pixmap = QPixmap(QImage(self.done_img_path))
        pixmap = pixmap.scaled(100, 20, QtCore.Qt.KeepAspectRatio) 
        self.ui.UAV1_Status.setPixmap(pixmap)

    # UAV2 Status
    def UAV2_stby_clicked(self):
        pixmap = QPixmap(QImage(self.stby_img_path))
        pixmap = pixmap.scaled(100, 20, QtCore.Qt.KeepAspectRatio) 
        self.ui.UAV2_Status.setPixmap(pixmap)

    def UAV2_mission_clicked(self):
        pixmap = QPixmap(QImage(self.mission_img_path))
        pixmap = pixmap.scaled(100, 20, QtCore.Qt.KeepAspectRatio) 
        self.ui.UAV2_Status.setPixmap(pixmap)

    def UAV2_done_clicked(self):
        pixmap = QPixmap(QImage(self.done_img_path))
        pixmap = pixmap.scaled(100, 20, QtCore.Qt.KeepAspectRatio) 
        self.ui.UAV2_Status.setPixmap(pixmap)

    def map_change_down_clicked(self):
        if (self.map_size == 50):
            self.map_size = 20
            self.Scale_factor = scale_20m
        elif (self.map_size == 100):
            self.map_size = 50
            self.Scale_factor = scale_50m
        elif (self.map_size == 200):
            self.map_size = 100
            self.Scale_factor = scale_100m
        elif (self.map_size == 500):
            self.map_size = 200
            self.Scale_factor = scale_200m
        self.visual_map()
        
    def mpa_change_clicked(self):
        if (self.map_size == 20):
            self.map_size = 50
            self.Scale_factor = scale_50m
        elif (self.map_size == 50):
            self.map_size = 100
            self.Scale_factor = scale_100m
        elif (self.map_size == 100):
            self.map_size = 200
            self.Scale_factor = scale_200m
        elif (self.map_size == 200):
            self.map_size = 500
            self.Scale_factor = scale_500m
        self.visual_map()

    def visual_UI(self):
        self.visual_map()
        self.visual_time()
        self.visual_txt()

    def visual_txt(self):
        # OPV
        self.ui.IN_Ownship_lat.setText(str(ros.Ownship_lat))
        self.ui.IN_Ownship_lon.setText(str(ros.Ownship_lon))
        self.ui.IN_Ownship_Alt.setText(str(ros.Ownship_alt))
        self.ui.IN_Ownship_Head.setText(str(ros.Ownship_heading))
        self.ui.IN_Ownship_Hor_vel.setText(str(ros.Ownship_hor_vel))
        self.ui.IN_Ownship_Ver_vel.setText(str(ros.Ownship_ver_vel))
        self.ui.IN_Ownship_x.setText(str(ros.Ownship_local_x))
        self.ui.IN_Ownship_y.setText(str(ros.Ownship_local_y))

        # USV1
        self.ui.USV1_lat.setText(str(ros.USV1_lat))
        self.ui.USV1_lon.setText(str(ros.USV1_lon))
        self.ui.USV1_alt.setText(str(ros.USV1_alt))
        self.ui.USV1_head.setText(str(ros.USV1_heading))
        self.ui.IN_USV1_Hor_vel.setText(str(ros.USV1_hor_vel))
        self.ui.IN_USV1_Ver_vel.setText(str(ros.USV1_ver_vel))
        self.ui.IN_USV1_x.setText(str(ros.USV1_local_x))
        self.ui.IN_USV1_y.setText(str(ros.USV1_local_y))

        # USV2
        self.ui.USV2_lat.setText(str(ros.USV2_lat))
        self.ui.USV2_lon.setText(str(ros.USV2_lon))
        self.ui.USV2_alt.setText(str(ros.USV2_alt))
        self.ui.USV2_head.setText(str(ros.USV2_heading))
        self.ui.IN_USV2_Hor_vel.setText(str(ros.USV2_hor_vel))
        self.ui.IN_USV2_Ver_vel.setText(str(ros.USV2_ver_vel))
        self.ui.IN_USV2_x.setText(str(ros.USV2_local_x))
        self.ui.IN_USV2_y.setText(str(ros.USV2_local_y))

        # UAV1
        self.ui.UAV1_lat.setText(str(ros.UAV1_lat))
        self.ui.UAV1_lon.setText(str(ros.UAV1_lon))
        self.ui.UAV1_alt.setText(str(ros.UAV1_alt))
        self.ui.UAV1_head.setText(str(ros.UAV1_heading))
        self.ui.IN_UAV1_Hor_vel.setText(str(ros.UAV1_hor_vel))
        self.ui.IN_UAV1_Ver_vel.setText(str(ros.UAV1_ver_vel))
        self.ui.IN_UAV1_x.setText(str(ros.UAV1_local_x))
        self.ui.IN_UAV1_y.setText(str(ros.UAV1_local_y))

        # UAV2
        self.ui.UAV2_lat.setText(str(ros.UAV2_lat))
        self.ui.UAV2_lon.setText(str(ros.UAV2_lon))
        self.ui.UAV2_alt.setText(str(ros.UAV2_alt))
        self.ui.UAV2_head.setText(str(ros.UAV2_heading))
        self.ui.IN_UAV2_Hor_vel.setText(str(ros.UAV2_hor_vel))
        self.ui.IN_UAV2_Ver_vel.setText(str(ros.UAV2_ver_vel))
        self.ui.IN_UAV2_x.setText(str(ros.UAV2_local_x))
        self.ui.IN_UAV2_y.setText(str(ros.UAV2_local_y))

        # Accident Distance
        if (self.pos_sharing):
            diff_x = round(ros.Ownship_local_x - self.shared_local_x, 3)
            diff_y = round(ros.Ownship_local_y - self.shared_local_y, 3)
            self.ui.OPV_acc_lat.setText(str(diff_x))
            self.ui.OPV_acc_lon.setText(str(diff_y))
        
            diff_x = round(ros.USV1_local_x - self.shared_local_x, 3)
            diff_y = round(ros.USV1_local_y - self.shared_local_y, 3)
            self.ui.USV1_acc_lat.setText(str(diff_x))
            self.ui.USV1_acc_lon.setText(str(diff_y))

            diff_x = round(ros.USV2_local_x - self.shared_local_x, 3)
            diff_y = round(ros.USV2_local_y - self.shared_local_y, 3)
            self.ui.USV2_acc_lat.setText(str(diff_x))
            self.ui.USV2_acc_lon.setText(str(diff_y))

            diff_x = round(ros.UAV1_local_x - self.shared_local_x, 3)
            diff_y = round(ros.UAV1_local_y - self.shared_local_y, 3)
            self.ui.UAV1_acc_lat.setText(str(diff_x))
            self.ui.UAV1_acc_lon.setText(str(diff_y))

            diff_x = round(ros.UAV2_local_x - self.shared_local_x, 3)
            diff_y = round(ros.UAV2_local_y - self.shared_local_y, 3)
            self.ui.UAV2_acc_lat.setText(str(diff_x))
            self.ui.UAV2_acc_lon.setText(str(diff_y))

    def visual_time(self):
        self.ui.IN_ROS_TIME.setText(str(ros.ROS_TIME))

        year = datetime.datetime.fromtimestamp(ros.ROS_TIME).year
        month = datetime.datetime.fromtimestamp(ros.ROS_TIME).month
        if (len(str(month)) == 1):
            month = "0" + str(month)
        day = datetime.datetime.fromtimestamp(ros.ROS_TIME).day
        if (len(str(day)) == 1):
            day = "0" + str(day)
        hour = datetime.datetime.fromtimestamp(ros.ROS_TIME).hour
        if (len(str(hour)) == 1):
            hour = "0" + str(hour)
        minute = datetime.datetime.fromtimestamp(ros.ROS_TIME).minute
        if (len(str(minute)) == 1):
            minute = "0" + str(minute)
        second = datetime.datetime.fromtimestamp(ros.ROS_TIME).second
        if (len(str(second)) == 1):
            second = "0" + str(second)

        ROS_datetime = str(year) + '-' + str(month) + "-" + str(day) + " " + str(hour) + ":" + str(minute) + ":" + str(second)
        self.ui.CurrentTime.setText(str(ROS_datetime))

    def visual_map(self):
        map_path_list = [self.map_20_path, self.map_50_path, self.map_100_path]
        if (self.map_size == 20):
            self.map_img = read_img(self.map_20_path, map_row, map_col)
        if (self.map_size == 50):
            self.map_img = read_img(self.map_50_path, map_row, map_col)
        if (self.map_size == 100):
            self.map_img = read_img(self.map_100_path, map_row, map_col)
        if (self.map_size == 200):
            self.map_img = read_img(self.map_200_path, map_row, map_col)
        if (self.map_size == 500):
            self.map_img = read_img(self.map_500_path, map_row, map_col)

        ownship_img = read_img(self.ownship_img_path, icon_row_col, icon_row_col)
        USV1_img = read_img(self.USV1_img_path, icon_row_col, icon_row_col)
        USV2_img = read_img(self.USV2_img_path, icon_row_col, icon_row_col)
        UAV1_img = read_img(self.UAV1_img_path, icon_row_col, icon_row_col)
        UAV2_img = read_img(self.UAV2_img_path, icon_row_col, icon_row_col)

        height, width, channel = ownship_img.shape
        matrix = cv2.getRotationMatrix2D((width/2, height/2), -ros.Ownship_heading, 1)
        rot_ownship_img = cv2.warpAffine(ownship_img, matrix, (width, height))

        matrix = cv2.getRotationMatrix2D((width/2, height/2), -ros.USV1_heading, 1)
        rot_USV1_img = cv2.warpAffine(USV1_img, matrix, (width, height))

        matrix = cv2.getRotationMatrix2D((width/2, height/2), -ros.USV1_heading, 1)
        rot_USV2_img = cv2.warpAffine(USV2_img, matrix, (width, height))

        matrix = cv2.getRotationMatrix2D((width/2, height/2), -ros.USV1_heading, 1)
        rot_UAV1_img = cv2.warpAffine(UAV1_img, matrix, (width, height))

        matrix = cv2.getRotationMatrix2D((width/2, height/2), -ros.USV1_heading, 1)
        rot_UAV2_img = cv2.warpAffine(UAV2_img, matrix, (width, height))

        ownship_row_in_map = map_row / 2 - int(ros.Ownship_local_x * self.Scale_factor)
        ownship_col_in_map = map_col / 2 + int(ros.Ownship_local_y * self.Scale_factor)

        USV1_row_in_map = map_row / 2 - int(ros.USV1_local_x * self.Scale_factor)
        USV1_col_in_map = map_col / 2 + int(ros.USV1_local_y * self.Scale_factor)

        USV2_row_in_map = map_row / 2 - int(ros.USV2_local_x * self.Scale_factor)
        USV2_col_in_map = map_col / 2 + int(ros.USV2_local_y * self.Scale_factor)

        UAV1_row_in_map = map_row / 2 - int(ros.UAV1_local_x * self.Scale_factor)
        UAV1_col_in_map = map_col / 2 + int(ros.UAV1_local_y * self.Scale_factor)

        UAV2_row_in_map = map_row / 2 - int(ros.UAV2_local_x * self.Scale_factor)
        UAV2_col_in_map = map_col / 2 + int(ros.UAV2_local_y * self.Scale_factor)

        height, width, channel = rot_USV1_img.shape
        check_image_size = int(height/2)

        # ADD USV1 image to MAP
        if (USV1_row_in_map < map_row - check_image_size and USV1_row_in_map > check_image_size and USV1_col_in_map < map_col - check_image_size and USV1_col_in_map > check_image_size):
            self.overwrite_map_img = image_add_function(self.map_img, rot_USV1_img, USV1_row_in_map, USV1_col_in_map)
            self.check_USV1_in_map = 1
        else:
            self.check_USV1_in_map = 0

        # ADD USV2 image to MAP
        if (USV2_row_in_map < map_row - check_image_size and USV2_row_in_map > check_image_size and USV2_col_in_map < map_col - check_image_size and USV2_col_in_map > check_image_size):
            if (self.check_USV1_in_map):
                self.overwrite_map_img = image_add_function(self.overwrite_map_img, rot_USV2_img, USV2_row_in_map, USV2_col_in_map)
            else:
                self.overwrite_map_img = image_add_function(self.map_img, rot_USV2_img, USV2_row_in_map, USV2_col_in_map)
            self.check_USV2_in_map = 1
        else:
            self.check_USV2_in_map = 0

        # ADD UAV1 image to MAP
        if (UAV1_row_in_map < map_row - check_image_size and UAV1_row_in_map > check_image_size and UAV1_col_in_map < map_col - check_image_size and UAV1_col_in_map > check_image_size):
            if (self.check_USV1_in_map or self.check_USV2_in_map):
                self.overwrite_map_img = image_add_function(self.overwrite_map_img, rot_UAV1_img, UAV1_row_in_map, UAV1_col_in_map)
            else:
                self.overwrite_map_img = image_add_function(self.map_img, rot_UAV1_img, UAV1_row_in_map, UAV1_col_in_map)
            self.check_UAV1_in_map = 1
        else:
            self.check_UAV1_in_map = 0

        # ADD UAV2 image to MAP
        if (UAV2_row_in_map < map_row - check_image_size and UAV2_row_in_map > check_image_size and UAV2_col_in_map < map_col - check_image_size and UAV2_col_in_map > check_image_size):
            if (self.check_USV1_in_map or self.check_USV2_in_map or self.check_UAV1_in_map):
                self.overwrite_map_img = image_add_function(self.overwrite_map_img, rot_UAV2_img, UAV2_row_in_map, UAV2_col_in_map)
            else:
                self.overwrite_map_img = image_add_function(self.map_img, rot_UAV2_img, UAV2_row_in_map, UAV2_col_in_map)
            self.check_UAV2_in_map = 1
        else:
            self.check_UAV2_in_map = 0

        # ADD Ownship image to MAP
        if (ownship_row_in_map < map_row - check_image_size and ownship_row_in_map > check_image_size and ownship_col_in_map < map_col - check_image_size and ownship_col_in_map > check_image_size):
            if (self.check_UAV1_in_map or self.check_UAV2_in_map or self.check_USV1_in_map or self.check_USV2_in_map):
                self.overwrite_map_img = image_add_function(self.overwrite_map_img, rot_ownship_img, ownship_row_in_map, ownship_col_in_map)
            else:
                self.overwrite_map_img = image_add_function(self.map_img, rot_ownship_img, ownship_row_in_map, ownship_col_in_map)
            self.check_Ownship_in_map = 1
        else:
            self.check_Ownship_in_map = 0

        # Add indicator Text
        # map, text, location, font, fontsclae, color, thickness
        if (self.check_USV1_in_map):
            cv2.putText(self.overwrite_map_img, "USV1", (USV1_col_in_map - 20, USV1_row_in_map - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        if (self.check_USV2_in_map):
            cv2.putText(self.overwrite_map_img, "USV2", (USV2_col_in_map - 20, USV2_row_in_map - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        if (self.check_UAV1_in_map):
            cv2.putText(self.overwrite_map_img, "UAV1", (UAV1_col_in_map - 20, UAV1_row_in_map - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        if (self.check_UAV1_in_map):
            cv2.putText(self.overwrite_map_img, "UAV2", (UAV2_col_in_map - 20, UAV2_row_in_map - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        if (self.check_USV1_in_map):
            cv2.putText(self.overwrite_map_img, "Ownship", (ownship_col_in_map - 30, ownship_row_in_map - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        # Indicate Shared Pos in MAP
        if (self.pos_sharing):
            shared_pos_row_in_map = map_row / 2 - int(self.shared_local_x * self.Scale_factor)
            shared_pos_col_in_map = map_col / 2 + int(self.shared_local_y * self.Scale_factor)
            cv2.putText(self.overwrite_map_img, "X", (shared_pos_col_in_map, shared_pos_row_in_map), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 1)

        # Convert cv image to Q image
        if (self.check_Ownship_in_map == 0 and self.check_UAV1_in_map == 0 and self.check_UAV2_in_map == 0 and self.check_USV1_in_map == 0 and self.check_USV2_in_map == 0):
            height, width, channel = self.map_img.shape
            bytesPerLine = 3 * width
            qImg = QImage(self.map_img.data, width, height, bytesPerLine, QImage.Format_RGB888)
        else:
            height, width, channel = self.overwrite_map_img.shape
            bytesPerLine = 3 * width
            qImg = QImage(self.overwrite_map_img.data, width, height, bytesPerLine, QImage.Format_RGB888)

        # ADD Q image to Widget
        pixmap = QPixmap(qImg)
        pixmap = pixmap.scaled(map_col, map_row, QtCore.Qt.KeepAspectRatio) 
        self.ui.MAP.setPixmap(pixmap)
        

app = QApplication(sys.argv)
window = MainWindow()

if __name__ == "__main__":
    rospy.init_node('widget', anonymous = True)
    rospy.Subscriber("/ADS_B/Traffic_Report_Array", Traffic_Report_Array, ros.ADS_callback, queue_size = 1)

    # rospy.Subscriber("/OPV/GPS", Float32MultiArray, ros.USV2_callback, queue_size = 1) # USV2

    rospy.Subscriber("/UAV1/GPS", Float32MultiArray, ros.UAV1_callback, queue_size = 1)
    rospy.Subscriber("/UAV2/GPS", Float32MultiArray, ros.UAV2_callback, queue_size = 1)
    rospy.Subscriber("/USV1/GPS", Float32MultiArray, ros.USV1_callback, queue_size = 1)
    rospy.Subscriber("/USV2/GPS", Float32MultiArray, ros.USV2_callback, queue_size = 1)

    rospy.Subscriber("mavros/imu/data", Imu, ros.timer_callback, queue_size = 1)

    window.show()
    sys.exit(app.exec_())

