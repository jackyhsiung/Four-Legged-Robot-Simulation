import pybullet as p
import time
import pybullet_data
import math
import random
import numpy as np
import json
# from random_search import stand

def show_best(robot_urdf, params_file, iterations, maxForce=500):
	'''
		Animate how the robot move under the params
		with pybullet GUI
	'''

	physicsClient = p.connect(p.GUI)
	p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
	p.setGravity(0, 0, -10)
	planeId = p.loadURDF("plane.urdf")
	cubeStartPos = [0, 0, 0.2]
	cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
	
	# Load the robot model
	boxId = p.loadURDF(robot_urdf, cubeStartPos, cubeStartOrientation)
	start_x = p.getBasePositionAndOrientation(boxId)[0][0]

	# Load the parameters saved
	with open(params_file, 'r') as read_file:
		best_params = json.load(read_file)

	a_s = best_params['a_s']
	b_s = best_params['b_s']
	w_s = 0.05
	c_s = [best_params['c_s_1'], best_params['c_s_2'], best_params['c_s_3'], best_params['c_s_4']]
	a_a = best_params['a_a']
	b_a = best_params['b_a']
	w_a = 0.05
	c_a = [best_params['c_a_1'], best_params['c_a_2'], best_params['c_a_3'], best_params['c_a_4']]
	joint_indices_s = [1, 4, 7, 10]
	joint_indices_a = [2, 5, 8, 11]

	### Get the robot move for a certain period of time
	for i in range(iterations):
		############# Stand ###############	
		stand(boxId, maxForce)

		if (i >= 150):
			for j, joint_index in enumerate(joint_indices_s):
				targetPos = a_s + b_s * math.sin(w_s * (i - 150) + c_s[j])
				p.setJointMotorControl2(boxId, joint_index, controlMode=p.POSITION_CONTROL, 
						targetPosition= targetPos)
			for j, joint_index in enumerate(joint_indices_a):
				targetPos = a_a + b_a * math.sin(w_a * (i - 150) + c_a[j])
				p.setJointMotorControl2(boxId, joint_index, controlMode=p.POSITION_CONTROL, 
						targetPosition= targetPos)
						
		p.stepSimulation()
		time.sleep(1./240.)
	p.disconnect()

def stand(boxId, maxForce):
	# Change to vector later
	# Put in the handtuned_gait utils later 
	targetPos1 = - 30 / 180 * math.pi
	targetPos2 = 45 / 180 * math.pi
	p.setJointMotorControl2(bodyUniqueId=boxId,
	                  jointIndex=1,
                      controlMode=p.POSITION_CONTROL,
                      targetPosition = targetPos1,
                      force = maxForce)

	p.setJointMotorControl2(bodyUniqueId=boxId,
	                  jointIndex=2,
                      controlMode=p.POSITION_CONTROL,
                      targetPosition = targetPos2,
                      force = maxForce)

	p.setJointMotorControl2(bodyUniqueId=boxId,
	                  jointIndex=4,
                      controlMode=p.POSITION_CONTROL,
                      targetPosition = targetPos1,
                      force = maxForce)
	p.setJointMotorControl2(bodyUniqueId=boxId,
	                  jointIndex=5,
                      controlMode=p.POSITION_CONTROL,
                      targetPosition = targetPos2,
                      force = maxForce)
	p.setJointMotorControl2(bodyUniqueId=boxId,
	                  jointIndex=7,
                      controlMode=p.POSITION_CONTROL,
                      targetPosition = targetPos1,
                      force = maxForce)
	p.setJointMotorControl2(bodyUniqueId=boxId,
	                  jointIndex=8,
                      controlMode=p.POSITION_CONTROL,
                      targetPosition = targetPos2,
                      force = maxForce)
	p.setJointMotorControl2(bodyUniqueId=boxId,
	                  jointIndex=10,
                      controlMode=p.POSITION_CONTROL,
                      targetPosition = targetPos1,
                      force = maxForce)
	p.setJointMotorControl2(bodyUniqueId=boxId,
	                  jointIndex=11,
                      controlMode=p.POSITION_CONTROL,
                      targetPosition = targetPos2,
                      force = maxForce)


if __name__ == '__main__':

	robot_urdf = "whole3.urdf"
	epochs = 100 # epochs for Searching
	iterations = 1000 # iterations (time) of each epoch
	maxForce = 500 # joint maximum force
	output_path = 'test.json'	
	show_best(robot_urdf, output_path, iterations)

