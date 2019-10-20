import vrep
import time
import numpy as np
from math import cos, sin
from scipy.linalg import expm,logm


# ================================== Forward kinematics function  ===================================== #

def getS_screw(S,i):
	screw_i = np.array([[0,        -S[2][i], S[1][i],      S[3][i]],
						[S[2][i] , 0       ,-S[0][i],      S[4][i]],
						[-S[1][i], S[0][i] , 0      ,      S[5][i]],
						[0,        0       , 0      ,      0      ]])
	return screw_i

def forward_k(theta):

	M = np.array([[0       , 1       , 0      ,      540], \
				  [0       , 0       , 1      ,      250], \
				  [1       , 0       , 0      ,      152], \
				  [0       , 0       , 0      ,      1  ]])
	S = np.array([[0   ,0    , 0  ,  0 ,  1  ,  0 ],
				  [0   ,1    , 1  ,  1 ,  0  ,  1 ],
				  [1   ,0    , 0  ,  0 ,  0  ,  0 ],
				  [0   ,-152 ,-152,-152,  0  ,  -152 ],
				  [0   ,0    , 0  ,  0 , 152 ,  0 ],
				  [0   ,0    ,244 , 457,-110 ,  540 ]])
	T = M

	for i in range(6):
		ex = expm((getS_screw(S,5-i))*theta[5-i])
		T = np.dot(ex, T)
	
	print("Foward kinematics calculated:\n")	
	print(str(T) + "\n")
	return

# ======================================================================================================== #
grap_state = 0
# ========================================= Jaco Hand function =========================================== #
def JacoHandGrap():
	global grap_state

	# input
	input_invalid = 1
	while(input_invalid):
		input_msg = input("grap or release? (please enter grap or release)")
		if((input_msg=="grap") | (input_msg=="release")):
			input_invalid = 0
		else:
			print("invalid input!\n")
	
	# grap
	if (input_msg == "grap"):
		if (grap_state == 0):

			vrep.simxSetStringSignal(clientID,'jacoHand','true',vrep.simx_opmode_oneshot)
			time.sleep(1)
			grap_state = 1

		else:
			print("Already grapped. Ignore your command.")
			return

	# release
	if (input_msg == "release"):
		if (grap_state == 1):
			vrep.simxSetStringSignal(clientID,'jacoHand','false',vrep.simx_opmode_oneshot)
			grap_state = 0
		else:
			print("Already release. Ignore your command.")
			return

# ======================================================================================================== #


# Get distances measurements from each joint center to base frame (useful for forward kinematics)
def get_joint():
	X = []
	Y = []
	Z = []
	result,vector=vrep.simxGetObjectPosition(clientID, joint_one_handle,base_handle,vrep.simx_opmode_blocking)
	X.append(vector[0])
	Y.append(vector[1])
	Z.append(vector[2])
	result,vector=vrep.simxGetObjectPosition(clientID, joint_two_handle,base_handle,vrep.simx_opmode_blocking)
	X.append(vector[0])
	Y.append(vector[1])
	Z.append(vector[2])
	result,vector=vrep.simxGetObjectPosition(clientID, joint_three_handle,base_handle,vrep.simx_opmode_blocking)
	X.append(vector[0])
	Y.append(vector[1])
	Z.append(vector[2])
	result,vector=vrep.simxGetObjectPosition(clientID, joint_four_handle,base_handle,vrep.simx_opmode_blocking)
	X.append(vector[0])
	Y.append(vector[1])
	Z.append(vector[2])
	result,vector=vrep.simxGetObjectPosition(clientID, joint_five_handle,base_handle,vrep.simx_opmode_blocking)
	X.append(vector[0])
	Y.append(vector[1])
	Z.append(vector[2])
	result,vector=vrep.simxGetObjectPosition(clientID, joint_six_handle,base_handle,vrep.simx_opmode_blocking)
	X.append(vector[0])
	Y.append(vector[1])
	Z.append(vector[2])
	result,vector=vrep.simxGetObjectPosition(clientID, end_handle,base_handle,vrep.simx_opmode_blocking)
	X.append(vector[0])
	Y.append(vector[1])
	Z.append(vector[2])
	X = np.round(X, decimals = 3)
	Y = np.round(Y, decimals = 3)
	Z = np.round(Z, decimals = 3)
	return X,Y,Z

# Function that used to move joints
def SetJointPosition(theta):
	vrep.simxSetJointTargetPosition(clientID, joint_one_handle, theta[0], vrep.simx_opmode_oneshot)
	time.sleep(0.5)
	vrep.simxSetJointTargetPosition(clientID, joint_two_handle, theta[1], vrep.simx_opmode_oneshot)
	time.sleep(0.5)
	vrep.simxSetJointTargetPosition(clientID, joint_three_handle, theta[2], vrep.simx_opmode_oneshot)
	time.sleep(0.5)
	vrep.simxSetJointTargetPosition(clientID, joint_four_handle, theta[3], vrep.simx_opmode_oneshot)
	time.sleep(0.5)
	vrep.simxSetJointTargetPosition(clientID, joint_five_handle, theta[4], vrep.simx_opmode_oneshot)
	time.sleep(0.5)
	vrep.simxSetJointTargetPosition(clientID, joint_six_handle, theta[5], vrep.simx_opmode_oneshot)
	time.sleep(0.5)

# Function that used to read joint angles
def GetJointAngle():
	result, theta1 = vrep.simxGetJointPosition(clientID, joint_one_handle, vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get 1 joint variable')
	result, theta2 = vrep.simxGetJointPosition(clientID, joint_two_handle, vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get 2 joint variable')
	result, theta3 = vrep.simxGetJointPosition(clientID, joint_three_handle, vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get 3 joint variable')
	result, theta4 = vrep.simxGetJointPosition(clientID, joint_four_handle, vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get 4 joint variable')
	result, theta5 = vrep.simxGetJointPosition(clientID, joint_five_handle, vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get 5 joint variable')
	result, theta6 = vrep.simxGetJointPosition(clientID, joint_six_handle, vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get 6 joint variable')
	theta = np.array([[theta1],[theta2],[theta3],[theta4],[theta5],[theta6]])
	return theta



# ======================================================================================================= #
# ======================================= Start Simulation ============================================== #
# ======================================================================================================= #

# Close all open connections (Clear bad cache)
vrep.simxFinish(-1)
# Connect to V-REP (raise exception on failure)
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID == -1:
	raise Exception('Failed connecting to remote API server')

# ======================================== Setup "handle"  =========================================== #

'''
# Print object name list
result,joint_name,intData,floatData,stringData = vrep.simxGetObjectGroupData(clientID,vrep.sim_appobj_object_type,0,vrep.simx_opmode_blocking)
print(stringData)
'''

# Get "handle" to the base of robot
result, base_handle = vrep.simxGetObjectHandle(clientID, 'UR3_link1_visible', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for base frame')
    
# Get "handle" to the all joints of robot
result, joint_one_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint1', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for first joint')
result, joint_two_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint2', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for second joint')
result, joint_three_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint3', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for third joint')
result, joint_four_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint4', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for fourth joint')
result, joint_five_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint5', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for fifth joint')
result, joint_six_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint6', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for sixth joint')

# Get "handle" to the end-effector of robot
result, end_handle = vrep.simxGetObjectHandle(clientID, 'UR3_link7_visible', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for end effector')
# ==================================================================================================== #

# Start simulation
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

# ******************************** Your robot control code goes here  ******************************** #
time.sleep(1)

pi = np.pi
Goal_joint_angles = np.array([0,pi/2,0.,0.,0.,0.])
SetJointPosition(Goal_joint_angles)

# Wait two seconds
time.sleep(3)
# *************************************************************************************************** #

command = np.array([0,0,0,0,0,0])
vrep.simxSetStringSignal(clientID,'jacoHand','false',vrep.simx_opmode_oneshot)
j=0
while(j<10):
	invalid = 1
	while(invalid):
		input_msg = input("input six angles(degrees) to be added on the current axis angles seprated by space:")
		input_angle = input_msg.split(" ")
		if(len(input_angle)==6):
			invalid = 0
		else:
			print("invalid input!\n")
	for i in range(6):
		command[i] = command[i] + (float(input_angle[i])*pi)/180
	SetJointPosition(command)
	forward_k(command)
	
        ### Global variables



	# grap open or close #
	JacoHandGrap()

	j+=1
print("your 10 chances are over, simulation will stop\n")

# Stop simulation
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID)
# Close the connection to V-REP
vrep.simxFinish(clientID)
print("==================== ** Simulation Ended ** ====================")

# ======================================================================================================= #
# ======================================== End Simulation =============================================== #
# ======================================================================================================= #
