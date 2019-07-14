import socket
import time
import pickle
import pyquaternion

def recv(port=50000, addr="192.168.1.5", buf_size=1024):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(('', port))
        data, sender_addr = s.recvfrom(buf_size)
        s.close()
        return data

i=0
while True:
	#time.sleep(0.001)
	print((recv()).decode('utf_8'))
	if i==0:
		start=time.time()
	i=i+1
	if i==250:
		break
print(time.time()-start)
