#! /usr/bin/env python
import os
import sys
import json
import math
import yaml
import rospy
import signal
import pymysql
import numpy as np
from skimage import draw
from nav_msgs.msg import Path, OccupancyGrid
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import PoseWithCovarianceStamped,PointStamped,PoseStamped


with open(os.environ['HOME'] + '/forx_utils/db/db_params.yaml') as data:
    db = yaml.load(data,Loader=yaml.FullLoader)
    USER     = db['user']
    PASSWORD = db['password']
    DATABASE = db['database']
    HOST     = db['host']

STATION_LIST = [1,2,3]

class StationInfo:
    def __init__(self,station_id,station_pose):
        self.id    = station_id
        self.pos_x = station_pose['position']['x']
        self.pos_y = station_pose['position']['y']
        self.ort_z = station_pose['orientation']['z']
        self.ort_w = station_pose['orientation']['w']

# class MapInfo:
#     def __init__(self,map_data):
#         self.width = map_data.info.width
#         self.height = map_data.info.height
#         self.array = np.reshape(np.array(map_data.data),(self.width,self.height),order="F")
#         self.array[:,:] = 100

class MapInfo:
    def __init__(self,map_data):
        self.info        = map_data.info
        self.width       = map_data.info.width
        self.height      = map_data.info.height
        self.origin_x    = map_data.info.origin.position.x
        self.origin_y    = map_data.info.origin.position.y
        self.resolution  = map_data.info.resolution

class PathCreater:
    def __init__(self):
        signal.signal(signal.SIGINT,  self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        self.map_array    = []
        self.station_list = []
        self.path_array   = []
        self.junction_array = []
        self.path_to_pub  = Path()
        self.path_to_pub.header.frame_id = "map"
        self.junction_map = OccupancyGrid()
        self.junction_marker_array = MarkerArray()
        self.path_marker_array = MarkerArray()
        self.path_pos_pub = rospy.Publisher('path_visualizer', Path, queue_size=10)
        self.text_marker_array = MarkerArray()



    def connectDB(self):
        return pymysql.connect(host=HOST, user=USER, password=PASSWORD, database=DATABASE,cursorclass=pymysql.cursors.DictCursor)

    @staticmethod
    def pointToPose(point):
        pose = PoseStamped()
        pose.pose.position.x = point.point.x
        pose.pose.position.y = point.point.y
        return pose

    @staticmethod
    def removePoseCov(pose_in):
        pose = PoseStamped()
        pose.pose.position.x = pose_in.pose.pose.position.x
        pose.pose.position.y = pose_in.pose.pose.position.y
        return pose

    def getStationInitPoses(self):
        connection = self.connectDB()
        try:
            with connection.cursor() as cursor:
                sql = "SELECT STATION_ID,STATION_INIT_POSE FROM STATIONS"
                cursor.execute(sql)
                fetched = cursor.fetchall()
                connection.close()
                return fetched
        except Exception as e:
            print e

    def signal_handler(self,signal,frame):
        print("Terminating path creater...")
        sys.exit(1)

    def mapCallback(self,data):
        # map_data = MapInfo(data)
        # self.map_array = map_data.array
        # print(self.map_array)
        self.map = MapInfo(data)
        junction_map_array = np.full((self.map.width * self.map.height,), 60, dtype=int)
        self.junction_map_array = np.reshape(junction_map_array, (self.map.width, self.map.height), order="F")
        self.junction_map.info = self.map.info

    def pathPosCallback(self,data):
        self.path_array.append(str(data.point.x) + "," + str(data.point.y) + ",")
        print("Appended Path Position: {}".format(self.path_array[-1]))
        cur_poses = [x for x in self.path_to_pub.poses]
        cur_poses.append(self.pointToPose(data))
        self.path_to_pub.poses = cur_poses
        self.path_pos_pub.publish(self.path_to_pub)
        self.visualizePathPoints(self.path_array)
            
    def lastPathPosCallback(self,data):
        last_pos = self.removePoseCov(data)
        self.path_array.append(str(last_pos.pose.position.x) + "," + str(last_pos.pose.position.y) + ",")
        print("Last Appended Path Position: {}".format(self.path_array[-1]))
        cur_poses = [x for x in self.path_to_pub.poses]
        cur_poses.append(last_pos)
        self.path_to_pub.poses = cur_poses
        self.path_pos_pub.publish(self.path_to_pub)
        self.savePathPositions()

    def junctionCallback(self,data):
        self.junction_array.append(str(data.point.x) + "," + str(data.point.y) + ",")
        self.visualizeJunctions(self.junction_array)
    
    def createStationData(self):
        stations = self.getStationInitPoses()
        station_list = []
        for station in stations:
            station_id = station['STATION_ID']
            if(station_id in STATION_LIST):
                station_info = StationInfo(station_id,json.loads(station['STATION_INIT_POSE']))
                station_list.append(station_info)
        print(station_list)
        return station_list

    def interpolateTwoPoints(self,x_start, x_end, y_start, y_end):
        resolution = 0.2
        partial_list = []
        y_diff = float(y_end - y_start)
        x_diff = float(x_end - x_start)
        if x_diff == 0:
            return
        hypo = math.sqrt(x_diff ** 2 + y_diff ** 2)
        slope = math.atan(y_diff / x_diff)
        if x_start > x_end:
            slope += math.pi
        cur_hypo = 0
        interp_x = [x_start]
        interp_y = [y_start]
        while cur_hypo + resolution < hypo:
            interp_x.append(interp_x[-1] + resolution * math.cos(slope))
            interp_y.append(interp_y[-1] + resolution * math.sin(slope))
            cur_hypo += resolution
        for cur in range(0, len(interp_x)):
            partial_list.append(str(interp_x[cur]) + "," + str(interp_y[cur]) + ",")
        return partial_list

    def interpolatePathArray(self,path_rr):
        ret_list = []
        if len(path_rr) == 0:
            return ret_list
        for cur_ind in range(0, len(path_rr) - 1):
            x0 = float(path_rr[cur_ind].split(",")[0])
            x1 = float(path_rr[cur_ind + 1].split(",")[0])
            y0 = float(path_rr[cur_ind].split(",")[1].strip(","))
            y1 = float(path_rr[cur_ind + 1].split(",")[1].strip(","))
            ret_list += self.interpolateTwoPoints(x0, x1, y0, y1)
        ret_list.append(path_rr[-1])
        return ret_list

    def savePathPositions(self):
        path_desc  = "path_desc"
        path_title = "path_title"
        path_name  = "path_name"
        interpolated  = self.interpolatePathArray(self.path_array)
        raw_path = json.dumps(self.path_array)
        interpolated_path = json.dumps(interpolated)
        junctions = json.dumps(self.junction_array)
        try:
            connection = self.connectDB()
            with connection as cursor:
                sql = "INSERT INTO `PATHS2` (`PATH_TITLE`, `PATH_DESC`, `PATH_RAW_ARRAY`, `PATH_INTERPOLATE_ARRAY`, `PATH_JUNCTION_ARRAY`,`PATH_LENGTH`, `PATH_MAP_ID`) VALUES ('{}','{}','{}','{}','{}', '{}','{}')".format(
                    path_title, path_desc, raw_path, interpolated_path,junctions, len(interpolated_path[:-1]), 26)
                cursor.execute(sql)
                connection.commit()
                print("Saved Path ID: {}".format(cursor.lastrowid)) 
        except Exception as e:
            print e

        pass

    def createStationMarker(self,station):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.scale.x = 0.7
        marker.scale.y = 0.2
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.text = "station1"
        marker.id = station.id
        marker.pose.position.x    = station.pos_x
        marker.pose.position.y    = station.pos_y
        marker.pose.orientation.z = station.ort_z
        marker.pose.orientation.w = station.ort_w

        return marker

    def visualizeStations(self):
        marker_array = MarkerArray()
        for station in self.station_list:
            marker_array.markers.append(self.createStationMarker(station))
        return marker_array

    def createJunctionMarker(self,index,x,y):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.scale.x = 4.0
        marker.scale.y = 4.0
        marker.scale.z = 0.1
        marker.color.a = 0.35
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.id = index
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0

        return marker
    
    def visualizeJunctions(self,junction_array):
        self.junction_marker_array = MarkerArray()
        self.text_marker_array = MarkerArray()
        for index in range(0,len(junction_array)):
            junction = junction_array[index].split(",")
            self.junction_marker_array.markers.append(self.createJunctionMarker(index,float(junction[0]),float(junction[1])))
            self.text_marker_array.markers.append(self.createTextMarker(index,float(junction[0]),float(junction[1])))


    def createPathMarker(self,index,x,y):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.id = index
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0

        return marker

    def visualizePathPoints(self,path_array):
        self.path_marker_array = MarkerArray()
        for index in range(0,len(path_array)):
            path = path_array[index].split(",")
            self.path_marker_array.markers.append(self.createPathMarker(index,float(path[0]),float(path[1])))

    def createTextMarker(self,index,x,y):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.type = marker.TEXT_VIEW_FACING
        marker.action = marker.ADD
        marker.scale.x = 5.0
        marker.scale.y = 5.0
        marker.scale.z = 1.0
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.id = index
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0
        marker.text = str(index)

        return marker

    def run(self):
        rospy.init_node("path_creater")


        rospy.Subscriber("map",OccupancyGrid,self.mapCallback)
        # rospy.Subscriber("junction", PointStamped, self.junctionCallback)
        # rospy.Subscriber("clicked_point", PointStamped, self.pathPosCallback)
        # rospy.Subscriber("finalpose", PoseWithCovarianceStamped, self.lastPathPosCallback)


        # path_marker_pub     = rospy.Publisher('path_markers',MarkerArray,queue_size=1)
        station_marker_pub  = rospy.Publisher('station_markers', MarkerArray, queue_size=1)
        # junction_marker_pub = rospy.Publisher('junction_markers',MarkerArray,queue_size=1)
        # text_marker_pub     = rospy.Publisher('junction_text_markers',MarkerArray,queue_size=1)


        rate = rospy.Rate(10)
        self.station_list = self.createStationData()
        station_markers = self.visualizeStations()
        while not rospy.is_shutdown():
            station_marker_pub.publish(station_markers)
            # junction_marker_pub.publish(self.junction_marker_array)
            # path_marker_pub.publish(self.path_marker_array)
            # text_marker_pub.publish(self.text_marker_array)

        rate.sleep()


if __name__ == "__main__":
    pc = PathCreater()
    pc.run()