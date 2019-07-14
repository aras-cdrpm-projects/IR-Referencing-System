'''
This code reads frames from a h264 file and after deriving the orientation, sends the quaternion values encoded by UTF-8 through UDP
'''
import argparse
import cv2
import numpy as np
import time
from pyquaternion import Quaternion
import pickle

import socket

def send(data, port=50000, addr='192.168.1.7'):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 20)
        s.sendto(data, (addr, port))

def recv(port=50000, addr="192.168.1.7", buf_size=1024):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
                s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except AttributeError:
                pass
        s.setsockopt(socket.SOL_IP, socket.IP_MULTICAST_TTL, 20)
        s.setsockopt(socket.SOL_IP, socket.IP_MULTICAST_LOOP, 1)
        s.bind(('', port))

        intf = socket.gethostbyname(socket.gethostname())
        s.setsockopt(socket.SOL_IP, socket.IP_MULTICAST_IF, socket.inet_aton(intf))
        s.setsockopt(socket.SOL_IP, socket.IP_ADD_MEMBERSHIP, socket.inet_aton(addr) + socket.inet_aton(intf))

        data, sender_addr = s.recvfrom(buf_size)
        s.setsockopt(socket.SOL_IP, socket.IP_DROP_MEMBERSHIP, socket.inet_aton(addr) + socket.inet_aton('0.0.0.0'))
        s.close()
        return data

vidpath = 'robot_vga_100.h264'
i=0
font = cv2.FONT_HERSHEY_SIMPLEX
LED=np.zeros((4,2))
LED_lab_rev=np.zeros((4, 2))
pnp_imgpoints=np.zeros((4, 2))
cap = cv2.VideoCapture(vidpath)
obj_points = np.array([[0.1, -0.065, 0], [-0.1, 0.065, 0], [0.1, 0.065, 0], [-0.1, -0.065, 0]])
cameraMatrix = np.array([[2.638354209087043e+03, 0, 0], [0, 2.632718241855754e+03, 0], [1.650514473415123e+03, 1.271476936986648e+03, 1]])
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 4, 1.0)

ret, frame = cap.read()
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
nonzero = cv2.findNonZero(gray)
nonzero = np.array(nonzero).reshape(nonzero.shape[0], 2)
nonzero = np.float32(nonzero)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
ret,label,center = cv2.kmeans(nonzero,4,None,criteria,10,cv2.KMEANS_RANDOM_CENTERS)
origin = ((center[0,0]+center[1,0]+center[2,0]+center[3,0])/4, (center[0,1]+center[1,1]+center[2,1]+center[3,1])/4)
for j in range(4):
    LED[j]=center[j]-origin
LED_lab=np.array([[LED[(LED[:, 0]>0) & (LED[:, 1]>0)][0][0]+origin[0],LED[(LED[:, 0]>0) & (LED[:, 1]>0)][0][1]+origin[1]],
 [LED[(LED[:, 0]<0) & (LED[:, 1]>0)][0][0]+origin[0],LED[(LED[:, 0]<0) & (LED[:, 1]>0)][0][1]+origin[1]],
 [LED[(LED[:, 0]>0) & (LED[:, 1]<0)][0][0]+origin[0],LED[(LED[:, 0]>0) & (LED[:, 1]<0)][0][1]+origin[1]],
 [LED[(LED[:, 0]<0) & (LED[:, 1]<0)][0][0]+origin[0],LED[(LED[:, 0]<0) & (LED[:, 1]<0)][0][1]+origin[1]]])
ledCenters_mismatched = LED_lab.reshape((4,2))
ledCenters = np.zeros(ledCenters_mismatched.shape)
ledCenters[0] = ledCenters_mismatched[2]
ledCenters[1] = ledCenters_mismatched[3]
ledCenters[2] = ledCenters_mismatched[0]
ledCenters[3] = ledCenters_mismatched[1]
while i<250:
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    nonZero = cv2.findNonZero(gray)
    nonZero = np.array(nonZero)
    nonZero = np.float32(nonZero.reshape(nonZero.shape[0], 2))
    ret,label,centers = cv2.kmeans(nonZero,4,None,criteria,10,cv2.KMEANS_RANDOM_CENTERS)
    minBox=centers-ledCenters[0]
    newUR_center=centers[np.where(minBox[:, 0]*minBox[:, 0]+minBox[:, 1]*minBox[:, 1]<10)]
    minBox=centers-ledCenters[1]
    newUL_center=centers[np.where(minBox[:, 0]*minBox[:, 0]+minBox[:, 1]*minBox[:, 1]<10)]
    minBox=centers-ledCenters[2]
    newLR_center=centers[np.where(minBox[:, 0]*minBox[:, 0]+minBox[:, 1]*minBox[:, 1]<10)]
    minBox=centers-ledCenters[3]
    newLL_center=centers[np.where(minBox[:, 0]*minBox[:, 0]+minBox[:, 1]*minBox[:, 1]<10)]
    ledCenters = np.array([newUR_center, newUL_center, newLR_center, newLL_center]).reshape(4,2)
    pnp_imgpoints[:, 1]=480 - ledCenters[:, 1]
    pnp_imgpoints[:, 0]=ledCenters[:, 0]
    ret, rvec, tvec = cv2.solvePnP(obj_points,
                                   pnp_imgpoints,
                                   cameraMatrix,
                                   0)
    dst, jac = cv2.Rodrigues(rvec)
    qmat = Quaternion(matrix=dst)
    send((str(qmat[0])+','+str(qmat[1])+','+str(qmat[2])+','+str(qmat[3])).encode('utf-8'))
    #time.sleep(0.01)
    i=i+1
