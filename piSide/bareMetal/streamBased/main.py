'''
This code reads frames from a h264 file and after deriving the orientation, sends the quaternion values encoded using binarization of struct through UDP
'''
import argparse
import cv2
import numpy as np
import time
#from pyquaternion import Quaternion
import pickle
import struct
import socket
import subprocess as sp
import atexit
import math


parser = argparse.ArgumentParser()
parser.add_argument("--int", help="The intrinsic matrix of the camera")
args = parser.parse_args()

(w,h) = (640,480)
bytesPerFrame = w * h
fps = 30

camParams = np.load(args.int)
videoCmd = "raspividyuv -w "+str(w)+" -h "+str(h)+" --output - --timeout 0 --framerate "+str(fps)+" --nopreview"+" -ex auto -dg 1 -awb off -sa -75 --luma"
videoCmd = "raspividyuv -w "+str(w)+" -h "+str(h)+" --output - --timeout 0 --framerate "+str(fps)+" --nopreview"+" -ex off -dg 2 -awb off -sa -20 --luma -ss 3500"

videoCmd = videoCmd.split()
cameraProcess = sp.Popen(videoCmd, stdout=sp.PIPE)
atexit.register(cameraProcess.terminate)
#atexit.register(out.release)
for _ in range(100):
    rawStream = cameraProcess.stdout.read(bytesPerFrame)

def send(data, port=50000, addr='192.168.10.10'):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 20)
        s.sendto(data, (addr, port))

def recv(port=50000, addr="192.168.10.10", buf_size=1024):
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

i=0
#cameraProcess.stdout.flush()
for _ in range(100):
    cameraProcess.stdout.flush()
    frame = np.fromfile(cameraProcess.stdout, count=bytesPerFrame, dtype=np.uint8)

if frame.size != bytesPerFrame:
    print("Error: Camera stream closed unexpectedly at the start")
    exit(0)
frame = frame.reshape((h, w))
frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)
font = cv2.FONT_HERSHEY_SIMPLEX
LED=np.zeros((4,2))
LED_lab_rev=np.zeros((4, 2))
pnp_imgpoints=np.zeros((4, 2))
obj_points = np.array([[0.1, 0.065, 0], [0, 0.065, 0], [0.1, 0, 0], [0, 0, 0]])
dist_coeffs= np.array([-0.405925403776238, 0.206430617505333, 0, 0, 0], dtype = np.float32)
cameraMatrix = camParams
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
font = cv2.FONT_HERSHEY_SIMPLEX

def centerExt(frame):
    global criteria
    LED=np.zeros((4,2))
    #cv2.imwrite('ft.jpg', frame)
    frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    nonzero = cv2.findNonZero(frame)
    nonzero = np.array(nonzero).reshape(nonzero.shape[0], 2)
    nonzero = np.float32(nonzero)
    ret,label,center = cv2.kmeans(nonzero,4,criteria,10,0)
    origin = ((center[0,0]+center[1,0]+center[2,0]+center[3,0])/4, (center[0,1]+center[1,1]+center[2,1]+center[3,1])/4)
    for j in range(4):
        LED[j]=center[j]-origin
    LED_lab=np.array([[LED[(LED[:, 0]>0) & (LED[:, 1]>0)][0][0]+origin[0],LED[(LED[:, 0]>0) & (LED[:, 1]>0)][0][1]+origin[1]],
                      [LED[(LED[:, 0]<0) & (LED[:, 1]>0)][0][0]+origin[0],LED[(LED[:, 0]<0) & (LED[:, 1]>0)][0][1]+origin[1]],
                      [LED[(LED[:, 0]>0) & (LED[:, 1]<0)][0][0]+origin[0],LED[(LED[:, 0]>0) & (LED[:, 1]<0)][0][1]+origin[1]],
                      [LED[(LED[:, 0]<0) & (LED[:, 1]<0)][0][0]+origin[0],LED[(LED[:, 0]<0) & (LED[:, 1]<0)][0][1]+origin[1]]])
    return LED_lab

def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

def rotationMatrixToEulerAngles(R) :
    assert(isRotationMatrix(R))
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
    return np.array([x, y, z])


criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 20, 0.5)
ledCenters = centerExt(frame)
frames = []
out = cv2.VideoWriter("motion.avi", cv2.cv.CV_FOURCC(*"MJPG"), 30, (w,h))


print("Initialization Successful")

while i<1000:
    start=time.time()
    cameraProcess.stdout.flush()
    frame = np.fromfile(cameraProcess.stdout, count=bytesPerFrame, dtype=np.uint8)
    if frame.size != bytesPerFrame:
        print("Error: Camera stream closed unexpectedly")
        break
    frame = frame.reshape((h, w))
    ret, frame = cv2.threshold(frame,100,255,cv2.THRESH_BINARY)
    #cv2.imwrite('ft.jpg', frame)
    nonZero = cv2.findNonZero(frame)
    nonZero = np.array(nonZero)
    nonZero = np.float32(nonZero.reshape(nonZero.shape[0], 2))
    ret,label,centers = cv2.kmeans(nonZero,4,criteria,5,cv2.KMEANS_PP_CENTERS, ledCenters)
    
    
    minBox=centers-ledCenters[0]
    newLR_center=centers[np.where(np.sqrt(minBox[:, 0]*minBox[:, 0]+minBox[:, 1]*minBox[:, 1])==np.min(np.sqrt(minBox[:, 0]*minBox[:, 0]+minBox[:, 1]*minBox[:, 1])))]
    #print(np.min(np.sqrt(minBox[:, 0]*minBox[:, 0]+minBox[:, 1]*minBox[:, 1])))
    minBox=centers-ledCenters[1]
    newLL_center=centers[np.where(np.sqrt(minBox[:, 0]*minBox[:, 0]+minBox[:, 1]*minBox[:, 1])==np.min(np.sqrt(minBox[:, 0]*minBox[:, 0]+minBox[:, 1]*minBox[:, 1])))]
    minBox=centers-ledCenters[2]
    newUR_center=centers[np.where(np.sqrt(minBox[:, 0]*minBox[:, 0]+minBox[:, 1]*minBox[:, 1])==np.min(np.sqrt(minBox[:, 0]*minBox[:, 0]+minBox[:, 1]*minBox[:, 1])))]
    minBox=centers-ledCenters[3]
    newUL_center=centers[np.where(np.sqrt(minBox[:, 0]*minBox[:, 0]+minBox[:, 1]*minBox[:, 1])==np.min(np.sqrt(minBox[:, 0]*minBox[:, 0]+minBox[:, 1]*minBox[:, 1])))]
    ledCenters = np.array([newLR_center[0], newLL_center[0], newUR_center[0], newUL_center[0]]).reshape(4,2)
    cv2.putText(frame,'LR',(int(ledCenters[0][0]), int(ledCenters[0][1])), font, 0.38,(255,255,255),1)
    cv2.putText(frame,'UL',(int(ledCenters[3][0]), int(ledCenters[3][1])), font, 0.38,(255,255,255),1)
    frames.append(frame)
    ret, rvec, tvec = cv2.solvePnP(obj_points,
                                   ledCenters,
                                   cameraMatrix,
                                   0)
    dst, jac = cv2.Rodrigues(rvec)
    #qmat = Quaternion(matrix=dst)
    send(struct.pack('dddddd',rvec[0], rvec[1], rvec[2], tvec[0], tvec[1], tvec[2]))
    #send(struct.pack('dddddd', rvec[0], time.time()-start, tvec[0], newUR_center[0][0].item(), newUR_center[0][1].item(), newUL_center[0][0].item()))
    #print(time.time()-start)
    i=i+1


for n in range(999):
    #cv2.imwrite("frame"+str(n)+".png", frames[n]) # save frame as a PNG image
    frame_rgb = cv2.cvtColor(frames[n],cv2.COLOR_GRAY2RGB) # video codec requires RGB image
    out.write(frame_rgb)
out.release()

cameraProcess.terminate()
out.release()
