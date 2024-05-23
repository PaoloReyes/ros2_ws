import numpy as np

def quaternion_from_euler(roll, pitch, yaw): 
	cr = np.cos(roll/2)
	sr = np.sin(roll/2)
	cp = np.cos(pitch/2)
	sp = np.sin(pitch/2)
	cy = np.cos(yaw/2)
	sy = np.sin(yaw/2)

	q = np.zeros(4)
	q[0] = sr*cp*cy - cr*sp*sy
	q[1] = cr*sp*cy + sr*cp*sy
	q[2] = cr*cp*sy - sr*sp*cy
	q[3] = cr*cp*cy + sr*sp*sy
	
	return q 