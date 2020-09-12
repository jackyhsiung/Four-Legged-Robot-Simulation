import pybullet as p
import time
import pybullet_data
import math
import random
import numpy as np
import json


def random_search(robot_urdf, epochs, iterations, output_path, maxForce=500):
	'''
		Random Search for parameters of sin function 
		of the shoulder and ankle joints of the robot

		Input Arguments
			epochs: epochs for Searching
			iterations: iterations (time) of each epoch
			maxForce: joint maximum force
			show_best: use GUI to animate how the robot move under the parameters searched
	''' 

	# Set Up the Environment 
	physicsClient = p.connect(p.DIRECT)#or p.DIRECT for non-graphical version 
	p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
	p.setGravity(0, 0, -10)
	planeId = p.loadURDF("plane.urdf")
	cubeStartPos = [0, 0, 0.2]
	cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
	# boxId = p.loadURDF("whole3.urdf",cubeStartPos, cubeStartOrientation)

	max_distance = 0
	max_distance_list = []	
	best_params = {}

	### Initial params for shoulder joints ###
	a_s = - 45 / 180 * math.pi
	b_s = - (60 - 30) / 180 * math.pi / 2
	w_s = 0.05
	c_s = [-2.804294668930453, -0.5946210730977946, -1.5707963267948966, -5.390875268393517]
	joint_indices_s = [1, 4, 7, 10]

	### Initial params for ankle joints ###
	a_a = 45 / 180 * math.pi
	b_a = (60 - 30) / 180 * math.pi / 2
	w_a = 0.05
	c_a = [0, 0, 0, 0]
	joint_indices_a = [2, 5, 8, 11]

	for epoch in range(epochs):
		# Load the robot model	
		boxId = p.loadURDF(robot_urdf, cubeStartPos, cubeStartOrientation)
		fall = False
		
		# params for shoulder joints
		# a_s += np.random.uniform(-45 / 180 * math.pi, 45 / 180 * math.pi)
		a_s = np.random.normal(- 45 / 180 * math.pi, 0.2)
		b_s += random.uniform(- 30 / 180 * math.pi / 2, 15 / 180 * math.pi / 2)
		k = random.randint(0, 3)
		c_s[k] += random.uniform(- math.pi, math.pi)

		# params for ankle joints
		# a_a += np.random.uniform(-45 / 180 * math.pi, 45 / 180 * math.pi)
		a_a = np.random.normal(45 / 180 * math.pi, 0.1)
		b_a += random.uniform(- 30 / 180 * math.pi / 2, 50 / 180 * math.pi / 2)
		c_a[0] += random.uniform(- math.pi, math.pi)
		c_a[1] += random.uniform(- math.pi, math.pi)
		c_a[2] += random.uniform(- math.pi, math.pi)
		c_a[3] += random.uniform(- math.pi, math.pi)

		# the start point of the robot in x coordinate
		start_x = p.getBasePositionAndOrientation(boxId)[0][0]

		# get the robot move for a certain period of time
		for i in range(iterations):
			
			# Make the robot stand up before start running
			stand(boxId)
			z = p.getBasePositionAndOrientation(boxId)[0][2]

			# If the robot falls down, stop this epoch 
			if (z > 0.4 or z < 0.15):
				fall = True
				print('Fell Down')
				break;

			# change the value of each joints
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

		end_x = p.getBasePositionAndOrientation(boxId)[0][0]
		distance = end_x - start_x
		if fall:
			distance = 0

		print(f'Distance of iteration{iterations}: ', distance)
		if (distance > max_distance):
			max_distance = distance
			# best_params = c_s	
			best_params['a_s'] = a_s
			best_params['b_s'] = b_s
			best_params['c_s_1'] = c_s[0]
			best_params['c_s_2'] = c_s[1]
			best_params['c_s_3'] = c_s[2]
			best_params['c_s_4'] = c_s[3]
			best_params['a_a'] = a_a
			best_params['b_a'] = b_a
			best_params['c_a_1'] = c_a[0]
			best_params['c_a_2'] = c_a[1]
			best_params['c_a_3'] = c_a[2]
			best_params['c_a_4'] = c_a[3]

		print('Max distance: ', max_distance)
		max_distance_list.append(max_distance)
		p.removeBody(boxId)

	print('Best params: ', best_params)
	print('Max distance: ', max_distance)
	p.disconnect()


	

	# find a better way of savinf the data. maybe json or sql
	# with open(output_path, 'w', newline='') as csvfile:
	# 	writer = csv.writer(csvfile)
	# 	writer.writerow(['max_distance_list'] + max_distance_list)
	# 	writer.writerow(['max_distance', max_distance])

	# 	for key, value in best_params.items():
	# 		writer.writerow([key, value])

	with open(output_path, 'w') as outfile:
		outfile.write(json.dumps(best_params, indent=4, sort_keys=True))


def stand(boxId):
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


# This function should probably be a independent file
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
		stand(boxId)

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


if __name__ == '__main__':

	robot_urdf = "whole3.urdf"
	epochs = 100 # epochs for Searching
	iterations = 1000 # iterations (time) of each epoch
	maxForce = 500 # joint maximum force
	output_path = 'random_search.json'	
	random_search(robot_urdf, epochs, iterations, output_path, maxForce)
	show_best(robot_urdf, output_path, iterations)



