#!/usr/bin/env python
import rospy
import tf
import roslib
from pyclustering.cluster.bsas import bsas
import numpy as np
import matplotlib.pyplot as plt
import cv2
import heapq
import yaml
from std_msgs.msg import String
from aras_iref.msg import aras_iref
from pyquaternion import Quaternion
import time
from yaml import load
import argparse

fig_size =[12,9]
plt.rcParams["figure.figsize"] = fig_size

ap = argparse.ArgumentParser()
ap.add_argument("-g", "--global_path", required=True,
	help="Full path to the yaml file containing the location of the markers in the global frame")
ap.add_argument("-c", "--calib_path", required=True,
        help="Full path to the yaml file containing the calibration parameters of the camera")
ap.add_argument("-d", "--dev_num", required=True,
        help="Number of the device pointing to the camera. e.g. 0 for /dev/video0")
args = vars(ap.parse_args())

print(args['global_path'], args['calib_path'], args['dev_num'])

# Classes and Functions
class markerExteractor(object):
    def __init__(self):
        self.max_clusters = 6
        self.threshold = 20
        self.blubParams = cv2.SimpleBlobDetector_Params()
        self.blubParams.minThreshold = 100
        self.blubParams.minDistBetweenBlobs = 1
        self.blubParams.maxThreshold = 255
        self.blubParams.filterByArea = True
        self.blubParams.minArea = 0
        self.blubParams.filterByCircularity = True
        self.blubParams.minCircularity = 0.1
        self.blubParams.filterByConvexity = True
        self.blubParams.minConvexity = 0.4
        self.blubParams.filterByInertia = True
        self.blubParams.minInertiaRatio = 0.1
        self.blubParams.blobColor = 255
        ver = (cv2.__version__).split('.')
        if int(ver[0]) < 3 :
            self.blubDetector = cv2.SimpleBlobDetector(self.blubParams)
        else : 
            self.blubDetector = cv2.SimpleBlobDetector_create(self.blubParams)
        
    def detect(self,frame):
        # This function uses the BSAS function in the pyclustering package
        # in https://github.com/annoviko/pyclustering/. Thereafter, we use
        # blob detection from OpenCV package to extract each marker's center.
        self.cms=[]
        self.image_ROIs=[]
        self.keypoints=[]
        img_gray=cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret,img_thresh = cv2.threshold(img_gray,100,255,cv2.THRESH_BINARY)
        self.nonzro_samples = cv2.findNonZero(img_thresh)
        if self.nonzro_samples is None:
            return None
        else:
            self.nonzro_samples=self.nonzro_samples.reshape(-1, 2).astype('float32')
        bsas_instance = bsas(self.nonzro_samples, self.max_clusters, self.threshold)
        bsas_instance.process()
        clusters = bsas_instance.get_clusters()
        self.ROIs=np.zeros((len(clusters),4))
        for i,cluster in enumerate(clusters):
            current_batch=self.nonzro_samples[cluster]
            self.cms.append(np.sum(current_batch,axis=0)/current_batch.shape[0])
            row_max=np.max(current_batch[:,1],axis=0)+4
            row_min=np.min(current_batch[:,1],axis=0)-4
            col_max=np.max(current_batch[:,0],axis=0)+4
            col_min=np.min(current_batch[:,0],axis=0)-4
            self.ROIs[i,:]=[row_min,row_max,col_min,col_max]
        for roi in self.ROIs.astype('int32'):
            self.image_ROIs.append(img_gray.copy()[roi[0]:roi[1],roi[2]:roi[3]])
        marker_points=[]
        for i,roi in enumerate(self.image_ROIs):
            keys_in_roi=self.blubDetector.detect(roi)
            for key in keys_in_roi:
                marker_points.append([key.pt[0]+self.ROIs.astype('float32')[i,2],key.pt[1]+self.ROIs.astype('float32')[i,0]])
        return np.array(marker_points)
    
    def redFilter(self, pts, thresh):
        # This function checks for marker redundancy and if two or more of
        # the markers are too close to each other it will remove the
        # redundant ones.
        threshed = []
        for i in range(len(pts)):
            temp=None
            for j in range(i+1, len(pts)):
                if np.linalg.norm(np.subtract(pts[i], pts[j]))<thresh:
                    temp=1
            if temp is None:
                threshed.append(pts[i])
        return np.array(threshed).reshape(-1, 2)

    
class undistrodMarkers:
    # A function for undistoring the marker points using the calibration file
    # which is the output of the ROS/OpenCV calibration framework.
    # http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration
    def __init__(self,config_file_name):
        with open(config_file_name, 'r') as f:
            calib = yaml.safe_load(f.read())
        self.K = np.array(calib['camera_matrix']['data']).reshape(calib['camera_matrix']['rows'],calib['camera_matrix']['cols'])
        self.D = np.array(calib['distortion_coefficients']['data']).reshape(-1, 5)
        self.P = np.array(calib['projection_matrix']['data']).reshape(3, 4)
        self.R = np.array(calib['rectification_matrix']['data']).reshape(3, 3)
        self.img_width = calib['image_width']
        self.img_height = calib['image_height']

    def process(self,points):
        return cv2.undistortPoints(points.reshape(-1,1,2).astype(np.float32), self.K, self.D,P=self.P,R=self.R)

    
def gpLoad(path):
    # Loading the global points
    f = open(path, 'r')
    yamlfile = load(f)
    f.close()
    return yamlfile

def frameLabel(frame, LED):
    # Labeling frames
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(frame,
                'LL_H',
                tuple(LED[2].astype('int')),
                font, 1,(255,255,255),1,cv2.LINE_AA)
    cv2.putText(frame,
                'LL_L',
                tuple(LED[1].astype('int')),
                font, 1,(255,255,255),1,cv2.LINE_AA)
    cv2.putText(frame,
                'LR',
                tuple(LED[0].astype('int')),
                font, 1,(255,255,255),1,cv2.LINE_AA)
    cv2.putText(frame,
                'UR_L',
                tuple(LED[4].astype('int')),
                font, 1,(255,255,255),1,cv2.LINE_AA)
    cv2.putText(frame,
                'UR_R',
                tuple(LED[5].astype('int')),
                font, 1,(255,255,255),1,cv2.LINE_AA)
    cv2.putText(frame,
                'UL',
                tuple(LED[3].astype('int')),
                font, 1,(255,255,255),1,cv2.LINE_AA)
    cv2.putText(frame,
                'O',
                tuple(origin.astype('int')),
                font, 1,(255,255,255),1,cv2.LINE_AA)
    cv2.circle(frame, tuple(origin.astype('int')), 1, color=(0,255,255))
    return frame


# Initilizations
marker_udistorder=undistrodMarkers(args['calib_path'])
pub = rospy.Publisher('ARAS_IREF', aras_iref, queue_size=1)
rospy.init_node('iref_pose', anonymous=True)
br = tf.TransformBroadcaster()
iref = aras_iref()
objyaml = gpLoad(args['global_path'])
objNames = ['LR', 'LL_L', 'LL_H', 'UL', 'UR_L', 'UR_R']
obj_points = np.zeros((6, 3), dtype=np.float)
for obj in objNames:
    obj_points[objNames.index(obj)]=objyaml['global_points'][obj]


# Run the Pipeline
markerExteractor_inst=markerExteractor()
cap=cv2.VideoCapture(int(args['dev_num']))
w = 640
h = 480
fps = 80
print(cap.set(cv2.CAP_PROP_FRAME_WIDTH, w))
print(cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h))
print(cap.set(cv2.CAP_PROP_FPS, fps))
print("Frame Details:")
print("Width = "+str(w)+", Height = "+str(h)+", At "+str(fps)+ "FPS")
font = cv2.FONT_HERSHEY_SIMPLEX
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output_debug.avi', fourcc, 50.0, (int(cap.get(3)),int(cap.get(4))))
verboseFault = 0
filThresh=0.1
ret, img = cap.read()
while ret==False:
    if verboseFault==0:
        print("Failed to Capture Initilization Frame")
        verboseFault=1
    ret, img = cap.read()
verboseFault = 0
points=markerExteractor_inst.detect(img)
while points is None:
    if verboseFault==0:
        print("No Markers Detected")
        verboseFault=1
    ret, img = cap.read()
    points=markerExteractor_inst.detect(img)
points = markerExteractor_inst.redFilter(points, filThresh)
verboseFault=0
while len(points)!=6:
    if verboseFault==0:
        print("Detected "+str(len(points))+" instead of 6 markers. Wrote the most recent image to script directory")
        verboseFault=1
    cv2.imwrite('faultymarkercount.jpg', img)
    ret, img = cap.read()
    points=markerExteractor_inst.detect(img)
    points = markerExteractor_inst.redFilter(points, filThresh)
centers = points.reshape(-1, 2)
rec = np.vstack([centers[np.array(heapq.nsmallest(2, range(len(centers[:, 0])), centers[:, 0].take))], centers[np.array(heapq.nlargest(2, range(len(centers[:, 0])), centers[:, 0].take))]])
origin = np.sum(rec, axis=0)/rec.shape[0]
center = centers-origin
LED=np.array([int(centers[(center[:, 0]>0) & (center[:, 1]>0)][0][0]),int(centers[(center[:, 0]>0) & (center[:, 1]>0)][0][1]),
             int(centers[centers[:, 1]==np.max(centers[(center[:, 0]<0) & (center[:, 1]>0)][:, 1])][0][0]),int(centers[centers[:, 1]==np.max(centers[(center[:, 0]<0) & (center[:, 1]>0)][:, 1])][0][1]),
             int(centers[centers[:, 1]==np.min(centers[(center[:, 0]<0) & (center[:, 1]>0)][:, 1])][0][0]),int(centers[centers[:, 1]==np.min(centers[(center[:, 0]<0) & (center[:, 1]>0)][:, 1])][0][1]),
             int(centers[(center[:, 0]<0) & (center[:, 1]<0)][0][0]),int(centers[(center[:, 0]<0) & (center[:, 1]<0)][0][1]),
             int(centers[(centers[:, 0]==np.min(centers[(center[:, 0]>0) & (center[:, 1]<0)][:, 0]))][0][0]),int(centers[(centers[:, 0]==np.min(centers[(center[:, 0]>0) & (center[:, 1]<0)][:, 0]))][0][1]),
             int(centers[(centers[:, 0]==np.max(centers[(center[:, 0]>0) & (center[:, 1]<0)][:, 0]))][0][0]),int(centers[(centers[:, 0]==np.max(centers[(center[:, 0]>0) & (center[:, 1]<0)][:, 0]))][0][1])]).reshape(6, 2)
frame = img.copy()
frame = frameLabel(frame, LED)
ret, rvec, T = cv2.solvePnP(obj_points, LED.astype('float').reshape(6, 1, 2), marker_udistorder.K, marker_udistorder.D, flags=cv2.SOLVEPNP_ITERATIVE)
R, jacobian=cv2.Rodrigues(rvec)
cv2.imshow('Initilization Frame', frame)
cv2.waitKey(0)
while cap.isOpened() and not rospy.is_shutdown():
    ret,img=cap.read()
    verboseFault=0
    while ret==False:
        if verboseFault==0:
            print("Failed to Capture a Frame")
            verboseFault=1
        ret, img = cap.read()
    verboseFault=0
    start=time.time()
    points=markerExteractor_inst.detect(img)
    while points is None:
        if verboseFault==0:
            print("No Markers Detected")
            verboseFault=1
        ret, img = cap.read()
        points=markerExteractor_inst.detect(img)
    points = markerExteractor_inst.redFilter(points, filThresh)
    verboseFault=0
    while len(points)!=6:
        if verboseFault==0:
            print("Detected "+str(len(points))+" instead of 6 markers. Wrote the most recent image to script directory")
            cv2.imwrite('faultymarkercount.jpg', img)
            verboseFault=1
        ret, img = cap.read()
        points=markerExteractor_inst.detect(img)
        while points is None:
            if verboseFault==0:
                print("No Markers Detected")
                verboseFault=1
            ret, img = cap.read()
            points=markerExteractor_inst.detect(img)
        points = markerExteractor_inst.redFilter(points, filThresh)
    points = points.reshape(6, -1)
    points = markerExteractor_inst.redFilter(points, filThresh)
    uLED = marker_udistorder.process(points).reshape(-1, 2)
    RKin = np.matmul(np.linalg.inv(R), np.linalg.inv(marker_udistorder.K))
    alpha = np.matmul(np.linalg.inv(R), T.reshape(3,-1))[2]/(np.matmul(RKin, np.vstack([uLED.transpose(), np.ones([1, 6])])))[2]
    GP = alpha*np.matmul(RKin, np.vstack([uLED.transpose(), np.ones([1, 6])]))-np.matmul(np.linalg.inv(R), T.reshape(3,-1))
    GP = GP.transpose()[:, 0:2]
    centers = GP.reshape(-1, 2)
    orFinder=[]
    smallest = centers[np.array(heapq.nsmallest(3, range(len(centers[:, 0])), centers[:, 0].take))]
    largest = centers[np.array(heapq.nlargest(2, range(len(centers[:, 0])), centers[:, 0].take))]
    orFinder.append(smallest[smallest[:, 1]==max(smallest[:, 1])])
    orFinder.append(smallest[smallest[:, 1]==min(smallest[:, 1])])
    orFinder = np.array(orFinder)
    rec = np.vstack([orFinder.reshape(2, -1), largest.reshape(2, -1)])
    origin = np.sum(rec, axis=0)/rec.shape[0]
    center = centers-origin
    GPstar=np.array([centers[(center[:, 0]>0) & (center[:, 1]>0)][0][0],
                     centers[(center[:, 0]>0) & (center[:, 1]>0)][0][1],
                     centers[centers[:, 1]==np.max(centers[(center[:, 0]<0) & (center[:, 1]>0)][:, 1])][0][0],
                     centers[centers[:, 1]==np.max(centers[(center[:, 0]<0) & (center[:, 1]>0)][:, 1])][0][1],
                     centers[centers[:, 1]==np.min(centers[(center[:, 0]<0) & (center[:, 1]>0)][:, 1])][0][0],
                     centers[centers[:, 1]==np.min(centers[(center[:, 0]<0) & (center[:, 1]>0)][:, 1])][0][1],
                     centers[(center[:, 0]<0) & (center[:, 1]<0)][0][0],
                     centers[(center[:, 0]<0) & (center[:, 1]<0)][0][1],
                     centers[(centers[:, 0]==np.min(centers[(center[:, 0]>0) & (center[:, 1]<0)][:, 0]))][0][0],
                     centers[(centers[:, 0]==np.min(centers[(center[:, 0]>0) & (center[:, 1]<0)][:, 0]))][0][1],
                     centers[(centers[:, 0]==np.max(centers[(center[:, 0]>0) & (center[:, 1]<0)][:, 0]))][0][0],
                     centers[(centers[:, 0]==np.max(centers[(center[:, 0]>0) & (center[:, 1]<0)][:, 0]))][0][1]]).reshape(6, 2)
    LED = np.array([points[np.where(GP==GPstar[0])],
                    points[np.where(GP==GPstar[1])],
                    points[np.where(GP==GPstar[2])],
                    points[np.where(GP==GPstar[3])],
                    points[np.where(GP==GPstar[4])],
                    points[np.where(GP==GPstar[5])]]).reshape(6,2)
    font = cv2.FONT_HERSHEY_SIMPLEX
    frame = img.copy()
    frame = frameLabel(frame, LED)
    ret, rvec, T = cv2.solvePnP(obj_points, LED.astype('float').reshape(6, 1, 2), marker_udistorder.K, marker_udistorder.D, rvec, T, 1, flags=cv2.SOLVEPNP_ITERATIVE)
    R, jacobian=cv2.Rodrigues(rvec)
    img_pts, jac = cv2.projectPoints(obj_points, rvec, T, marker_udistorder.K, marker_udistorder.D)
    img_pts = img_pts.reshape(6, 2)
    for i in range(len(img_pts)):
        cv2.circle(frame,(int(img_pts[i,0]), int(img_pts[i,1])),
                   2, (0,0,255), -1)
    br = tf.TransformBroadcaster()
    q = Quaternion(matrix=R)
    br.sendTransform((T[0], T[1], T[2]),
                     q.elements,
                     rospy.Time.now(),
                     "ARAS_IREF",
                     "world")
    iref.pose.orientation.w = q[0]
    iref.pose.orientation.x = q[1]
    iref.pose.orientation.y = q[2]
    iref.pose.orientation.z = q[3]
    iref.pose.position.x = T[0]
    iref.pose.position.y = T[1]
    iref.pose.position.z = T[2]
    pub.publish(iref)
    cv2.imshow('Frame',frame)
    if cv2.waitKey(3) & 0xFF == ord('q'):
        break
    out.write(img)
cv2.destroyAllWindows()
out.release()
cap.release()

