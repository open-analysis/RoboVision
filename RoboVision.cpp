// File:			RoboVision.cpp
// Date:			9/10/19
// Description:		Program will allow an e-puck robot to "see" its environment to guide itself through it (eg navigating a cave or tunnel).
// Author:			Josh Chica
// Modifications:	Many


#include <stdio.h>
#include <ctime>
#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Camera.hpp>
#include <webots/Motor.hpp>
#include <opencv2/opencv.hpp>


// Macros
#define TIME_STEP 64
#define MAX_SPEED 6.28

// namespaces
using namespace webots;
using namespace cv;

// forward declare OpenCV function
void EdgeDetect();


// main
int main(int argc, char **argv)
{
	/* necessary to initialize webots stuff */
	Robot *robot = new Robot();

	/*
   * Declaring Distance sensor 
   * Declaring Camera
   * Declaring Left+Right Motors
   * Declaring clock_t 
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

	clock_t pastTime = clock();
	double secondsPassed;
	/* main loop
	 * Perform simulation steps of TIME_STEP milliseconds
	 * and leave the loop when the simulation is over
	 */
	printf("Everything's loaded, about to move\n");

	while (robot->step(TIME_STEP) != -1) {

		/* Read the sensors */

		// read in distance sensors
		double psValues[8];
		for (int i = 0; i < 8; i++)
			psValues[i] = ps[i]->getValue();

		// taking a picture with camera
		const unsigned char *image = camera->getImage();
		//unsigned char greyImage = camera->imageGetGray(image, camera->getWidth(), 0, 0);

		/* Process sensor data here */

		/*
		* Determines if the robot should turn
		* checks if the distance sensors detect anything close by
		*/
		bool right_obstacle =
			psValues[0] > 70.0 ||
			psValues[1] > 70.0 ||
			psValues[2] > 70.0;
		bool left_obstacle =
			psValues[5] > 70.0 ||
			psValues[6] > 70.0 ||
			psValues[7] > 70.0;


		/* Actuator commands here */

		// setting the speed that will be assigned to the motors
		double leftSpeed = .5 * MAX_SPEED;
		double rightSpeed = .5 * MAX_SPEED;

		// determines if the robot should turn because something's too close
		if (left_obstacle) {
			// turn right
			leftSpeed += 0.5 * MAX_SPEED;
			rightSpeed -= 0.5 * MAX_SPEED;
		}
		else if (right_obstacle) {
			// turn left
			leftSpeed -= 0.5 * MAX_SPEED;
			rightSpeed += 0.5 * MAX_SPEED;
		}

		// actually setting the motors' velocities based on the above data
		//leftMotor->setVelocity(leftSpeed);
		//rightMotor->setVelocity(rightSpeed);

		// determing how much time has passed & if it should process the image
		secondsPassed = (clock() - pastTime) / CLOCKS_PER_SEC;
		if (secondsPassed >= 3) {
			if (camera->saveImage("image.png", 100) == 0);
			else {
				printf("couldn't save image\n");
				continue;
			}

			// declaring variable to assign to the see how long the function takes
			// before clock so it's not part of the count
			double functionTime = 0.0;
			// starting clock to see how long it takes EdgeDetect to run
			clock_t timeCount = clock();
			
			// detecting the edges on the saved image
			EdgeDetect();
			// Time elapsed during edge detect
			//functionTime = (clock() - timeCount) / CLOCKS_PER_SEC;
			//printf("%.20f\n", functionTime);
			
			// reset camera clock
			pastTime = clock();
		}
	};

	/* Enter your cleanup code here */

	/* This is necessary to cleanup webots resources */
	delete robot;

	return 0;
}

/*
	Function:	EdgeDetect
	Parameters: None
	Outputs:	None
	Purpose:	Processes the image taken by the camera to find the edges in the image
	Notes:		To be used in the final version (ie not debug) && extra datatypes commented out/removed to save space for data
*/
void EdgeDetect(){

	printf("Detecting Edges\n");

	//Mat src, src_gray;
	//Mat dst, detected_edges;
	Mat src;

	int lowThreshold = 30;
	//const int max_lowThreshold = 100;
	const int ratio = 3;
	const int kernel_size = 3; 

	src = imread("image.png", IMREAD_COLOR);
	if (src.empty()) {
		printf("image doesn't exist\n");
		return;
	}
	//dst.create(src.size(), src.type());

	// possible get rid of this if I just grab the grey image from the bot itself
	cvtColor(src, src, COLOR_BGR2GRAY);

	blur(src, src, Size(3, 3));
	Canny(src, src, lowThreshold, lowThreshold*ratio, kernel_size);
	//dst = Scalar::all(0);
	//src.copyTo(dst, src);

	// save image again so that it can be run through a NN
	// commented out bc it's not necessary atm
	imwrite("lines.png", src);

	printf("Done detecting\n");
}
