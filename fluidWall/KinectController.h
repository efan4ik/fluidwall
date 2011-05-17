/**
 * @file      KinectController.h
 * @author    Naureen Mahmood 
 * @copyright 2011 Austin Hines, Naureen Mahmood, and Texas A&M Dept. of Visualization
 * @version   1.0.0
 * 
 * This file is part of Fluid Wall. You can redistribute it and/or modify            
 * it under the terms of the GNU Lesser General Public License as published  
 * by the Free Software Foundation, either version 3 of the License, or     
 * (at your option) any later version.                                       
 *                                                                             
 * Fluid Wall is distributed in the hope that it will be useful,                 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of            
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the              
 * GNU Lesser General Public License for more details.                       
 *                                                                            
 * You should have received a copy of the GNU Lesser General Public License  
 * along with Fluid Wall. If not, see <http://www.gnu.org/licenses/>.            
 *
 */

#ifndef KINECT_CONTROLLER_H
#define KINECT_CONTROLLER_H

//OpenNI includes
#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>

//OpenCV includes
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <iostream>

//CL NUI includes
#include <CLNUIDevice.h>


#define COLOR_RANGE		255
#define Y_RES			XN_VGA_Y_RES
#define X_RES			XN_VGA_X_RES
#define SAMPLE_XML_PATH "Data/SamplesConfig.xml"
#define CHECK_RC(nRetVal, what)										\
	if (nRetVal != XN_STATUS_OK)									\
	{																\
		printf("%s failed: %s\n", what, xnGetStatusString(nRetVal));\
		return xnRetVal;											\
	}

using namespace std;
using namespace cv;

//! KinectController Class
/*!
	KinectController Class initializes and runs all the modules 
	for controlling the kinect camera and motor devices.
*/
class KinectController
{
public:

	/*
	* (Default) Constructor
	* @param	maxUsers		variable to initialize maximum number of users to be detected by the system
	* @param	nIterate		variable to initialize maximum iterations of the depth procedures before restarting 
	*							them (to clear the system every once in a while)
	* @param	vDepth			variable to initialize the depth threshold for the Kinect camera
	* @param	vMotor			variable to initialize the motor angle for the Kinect motor [up/down: +/-]
	*/
	KinectController    (	int userCount	= 6,	int iterationCount	= 10000, 
							int depthValue	= 6000, int motorAngle		= 10000);
	~KinectController() {	kinectCleanupExit();	}
	
	/*! Initialize all KinectController variables & modules	*/
	XnStatus init();
	/*! Depth & User Tracking Modules	*/
	XnStatus update();
	/*! Update the XnOpenNI Depth & User tracking data for each frame of video captured */
	XnStatus reset();

	/*! Set Depth Threshold		*/
	void setDepth(int depthDelta)	{	depthThresh+=depthDelta;	}
	/*! Set Kinect Motor Angle	[range: (up/down) 15000/-15000	*/
	void setMotorAngle(int angle);
	/*! Reset Kinect Motor to 'initAngle' value passed at intialization */
	void resetMotorAngle();
	
	/*! Get depth matrix for current video frame 	*/
	Mat getDepthMat()		{	return depthMatrix; }
	/*! Get matrix	of tracked users for current video frame */
	Mat getUsersMat()		{	return usersMatrix; }

private: 
	// OPENNI DEPTH & USER TRACKING VARIABLES

	xn::Context			xnContext;			/*! context object that creates depth and user data nodes	*/
	xn::DepthGenerator	xnDepthGenerator;	/*! captures and returns depth values at each frame	*/
	xn::UserGenerator	xnUserGenerator;	/*! captures and returns user detection data at each frame	*/

	xn::SceneMetaData	xnSceneMD;			/*! scene metadata: gives access to IDs of detected users at each pixel of a captured frame	*/
	xn::DepthMetaData	xnDepthMD;			/*! depth metadata: gives access to depth data at each pixel of a captured frame	*/


	XnStatus xnRetVal;						/*! used to check the status of each call to an XNOpenNI function	*/
	int		maxUsers;						/*! users to detect	*/
	int		maxIterate;						/*! iterations to run before reset	*/
	int		iterations;						/*! running iterations so far (goes up to nIterate then resets to 0)	*/
	int		depthThresh;					/*! depth threshold for how far the Kinect should capture	*/
	float	colorByDepth;
	Mat		depthMatrix;					/*! image-sized matrix containing the depth values at each pixel	*/
	Mat		usersMatrix;					/*! image-sized matrix containing the userID's of detected people at		
											/*! each pixel (or 0 if no detected user at that pixel)	*/
	
	// MOTOR CONTROL VARIABLES

	CLNUIMotor	nuiMotor;					/*! motor object	*/
	int			initAngle;					/*! motor's initial angle set at the start of program	*/
	int			nuiAngle;					/*! motor's current angle	*/
		

	/*! Initialize XnOpenNI depth control & user tracking modules */
	XnStatus initDepthControl();
	/*! Destroy & shutdown XnOpenNI depth control & user tracking modules */
	void stopDepthControl()		{	xnContext.Shutdown();		}
	/*! Initialize CLNUI motor control modules	*/
	void initMotorControl();
	/*! Destroy & shutdown CLNUI motor control modules	*/
	void stopMotorControl()		{	DestroyNUIMotor (nuiMotor); }
	/*! Run Shutdown functions for Depth and Motor control 	*/
	void kinectCleanupExit();
};

#endif