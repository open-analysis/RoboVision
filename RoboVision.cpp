// File:          RoboVision.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes

#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Camera.hpp>
#include<webots/Motor.hpp>


 /*
  * You may want to add macros here.
  */
#define TIME_STEP 64
#define MAX_SPEED 6.28

// All the webots classes are defined in the "webots" namespace
using namespace webots;

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv)
{
	/* necessary to initialize webots stuff */
	Robot *robot = new Robot();

	/*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */
	DistanceSensor *ps[8];
	char psNames[8][4] = { "ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7" };

	for (int i = 0; i < 8; i++) {
		ps[i] = robot->getDistanceSensor(psNames[i]);
		ps[i]->enable(TIME_STEP);
	}

	Camera *camera;
	camera = robot->getCamera("camera");
	camera->enable(TIME_STEP);


	Motor *leftMotor = robot->getMotor("left wheel motor");
	Motor *rightMotor = robot->getMotor("right wheel motor");
	leftMotor->setPosition(INFINITY);
	rightMotor->setPosition(INFINITY);
	leftMotor->setVelocity(0.0);
	rightMotor->setVelocity(0.0);

	/* main loop
	 * Perform simulation steps of TIME_STEP milliseconds
	 * and leave the loop when the simulation is over
	 */
	while (robot->step(TIME_STEP) != -1) {

		/*
		 * Read the sensors :
		 * Enter here functions to read sensor data, like:
		 *  double val = wb_distance_sensor_get_value(my_sensor);
		 */
		double psValues[8];
		for (int i = 0; i < 8; i++)
			psValues[i] = ps[i]->getValue();

		const unsigned char *image = camera->getImage();
		unsigned char greyImage = camera->imageGetGray(image, camera->getWidth(), 0, 0);

		/* Process sensor data here */

		bool right_obstacle =
			psValues[0] > 70.0 ||
			psValues[1] > 70.0 ||
			psValues[2] > 70.0;
		bool left_obstacle =
			psValues[5] > 70.0 ||
			psValues[6] > 70.0 ||
			psValues[7] > 70.0;

		/*
		 * Enter here functions to send actuator commands, like:
		 * wb_differential_wheels_set_speed(100.0,100.0);
		 */
		double leftSpeed = .5 * MAX_SPEED;
		double rightSpeed = .5 * MAX_SPEED;

		if (left_obstacle) {
			leftSpeed += 0.5 * MAX_SPEED;
			rightSpeed -= 0.5 * MAX_SPEED;
		}
		else if (right_obstacle) {
			// turn left
			leftSpeed -= 0.5 * MAX_SPEED;
			rightSpeed += 0.5 * MAX_SPEED;
		}
		// write actuators inputs
		leftMotor->setVelocity(leftSpeed);
		rightMotor->setVelocity(rightSpeed);

	};

	/* Enter your cleanup code here */

	/* This is necessary to cleanup webots resources */
	delete robot;

	return 0;
}