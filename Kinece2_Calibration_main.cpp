#include "BackgroundDepthSubtraction.h"
#include <ActivityMap_Utils.h>
#include "KinectSensor.h"
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <list>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <ctype.h>
#include "XnCppWrapper.h"

ofstream outFPK0 ("C:\\Dropbox\\PhD\\Matlab\\Calibration_wks\\kinect0_full.txt");
ofstream outFPK1 ("C:\\Dropbox\\PhD\\Matlab\\Calibration_wks\\kinect1_full.txt");
ofstream outFPK2 ("C:\\Dropbox\\PhD\\Matlab\\Calibration_wks\\kinect2_full.txt");

int frames = 0;
int KINECTS_DISPLACEMENT = 0; 
float MAX_RANGE;
const int NUM_SENSORS = 3;
int debugFrame = -1;


void updateActivityMap(const XnPoint3D* p3D, const int nP, int idKinect)
{
	for (int i = 0; i < nP; i++)
	{
		Point p2D = ActivityMap_Utils::findMoACoordinate(&p3D[i], MAX_RANGE);

		if (p2D.x != -1)
		{
			if (frames == debugFrame)
			{
				ofstream* out;
				if (idKinect == 0)
					out = &outFPK0;
				else if (idKinect == 1)
					out = &outFPK1;
				else
					out = &outFPK2;

				*out << (int)p2D.x << " " << (int)p2D.y << endl;
			}
		}
	}
}

XnPoint3D p3D[3];
KinectSensor kinects[NUM_SENSORS];
Mat depthImages[NUM_SENSORS];
Mat rgbImages[NUM_SENSORS];
const XnDepthPixel* depthMaps[NUM_SENSORS];
char* nWindows[NUM_SENSORS];


void pointSelection_onMouse_single(int event, int x, int y, int flags, void* param)
{
	
	if (event == CV_EVENT_FLAG_LBUTTON)
	{
		int* id = (int*)param;
		int idRef = id[0];
		int idNoRef = id[1];
		if (x != -1 && y != -1)
		{
			int depth = depthMaps[idRef][y*XN_VGA_X_RES+x];
			if (depth != 0)
			{
				XnPoint3D p2D, p3D;
				p2D.X = x; p2D.Y = y; p2D.Z = depth;
				kinects[idRef].arrayBackProject(&p2D, &p3D, 1);
				if (idRef != REF_CAM)
					kinects[idRef].transformArrayNoTilt(&p3D, 1);
				else
					kinects[idRef].transformArrayNoTilt_rev(&p3D, 1, idNoRef);

				XnPoint3D* pp= kinects[idNoRef].arrayProject(&p3D, 1);
				Point p = Point(pp->X, pp->Y);
				delete pp;
				circle(depthImages[idNoRef], p, 2, Scalar(255,0,0));
				circle(depthImages[idRef], Point(x,y), 2, Scalar(255,0,0));
				imshow(nWindows[idNoRef], depthImages[idNoRef]);
				imshow(nWindows[idRef], depthImages[idRef]);
				waitKey(0);
			}	
		}
	}
}

int nPointsTotal = 0;
void pointSelection_onMouse(int event, int x, int y, int flags, void* param)
{
	
	if (event == CV_EVENT_FLAG_LBUTTON)
	{
		int* id = (int*)param;

		if (x != -1 && y != -1)
		{
			int depth = depthMaps[*id][y*XN_VGA_X_RES+x];
			if (depth != 0)
			{
				XnPoint3D p2D;
				p2D.X = x; p2D.Y = y; p2D.Z = depth;
				kinects[*id].arrayBackProject(&p2D, &p3D[*id], 1);

				if (*id == 0)
					outFPK0 << (float)p3D[0].X << " " << (float)p3D[0].Y << " " << (float)p3D[0].Z << endl;
				else if (*id == 1)
					outFPK1 << (float)p3D[1].X << " " << (float)p3D[1].Y << " " << (float)p3D[1].Z << endl;
				else
					outFPK2 << (float)p3D[2].X << " " << (float)p3D[2].Y << " " << (float)p3D[2].Z << endl;

				p3D[*id].X = -1;
				circle(rgbImages[*id], Point(x,y), 2, Scalar(0,0,255));
				imshow(nWindows[*id], rgbImages[*id]);
				cout << "Point selected "  << ++nPointsTotal << endl;
				waitKey(0);
			}
			
		}
	}
}



void createDepthMatrix(const XnDepthPixel* dMap, Mat& depthMat)
{
	ushort* d_data = (ushort*)depthMat.data;
	int d_step = depthMat.step/sizeof(ushort);

	for (int i = 0; i < XN_VGA_Y_RES; i++)
	{
		ushort* ptr = d_data + i*d_step;
		for (int j = 0; j < XN_VGA_X_RES; j++)
		{
			ptr[j] = dMap[i*XN_VGA_X_RES+j];
		}
	}

}

void combineRGBD(Mat& img, const XnDepthPixel* dMap)
{
	uchar* data = img.data; 
	int step = img.step;
	for (int y = 0; y < XN_VGA_Y_RES; y++)
	{
		uchar* ptr = data + y*step;
		for (int x = 0; x < XN_VGA_X_RES; x++)
		{
			int depth = dMap[y*XN_VGA_X_RES+x];
			if (depth == 0)
			{
				ptr[3*x] = 255; ptr[3*x + 1] = 255; ptr[3*x + 2] = 255;
			}
		}
	}
}

ifstream tiltTXT("D:\\CameraCalibrations\\extrinsics\\tilt.txt");

void pointCorrespondences()
{
	int tilt = 0;
	tiltTXT >> tilt;
	char* paths[3];
/*	paths[0] = "d:/Emilio/Tracking/DataSet/sb125/CalibrationVideos/kinect0_calib.oni";
	paths[1] = "d:/Emilio/Tracking/DataSet/sb125/CalibrationVideos/kinect1_calib.oni";
	paths[2] = "d:/Emilio/Tracking/DataSet/sb125/CalibrationVideos/kinect2_calib.oni";
*/
	paths[0] = "d:/Emilio/Tracking/DataSet/kinect0_calib.oni";
	paths[1] = "d:/Emilio/Tracking/DataSet/kinect1_calib.oni";
	paths[2] = "d:/Emilio/Tracking/DataSet/kinect2_calib.oni";

//	p3D[0].X = -1; p3D[1].X = -1;
	
	const XnRGB24Pixel* rgbMaps[NUM_SENSORS];

	for (int i = 0; i < NUM_SENSORS; i++)
	{
		kinects[i].initDevice(i, REF_CAM, true, paths[i]);
		kinects[i].startDevice();
		kinects[i].tilt(tilt);
	}

	nWindows[0] =  "depth 0";
	nWindows[1] =  "depth 1";
	nWindows[2] =  "depth 2";

	namedWindow(nWindows[0]);
	namedWindow(nWindows[1]);
	namedWindow(nWindows[2]);
//	Point p0, p1;
//	p1.x = p0.x = -1; p1.y =  p0.y = -1;
	int id0 = 0;
	int id1 = 1;
	int id2 = 2;
	//Select points for calibrations
	cvSetMouseCallback(nWindows[0], pointSelection_onMouse, (int*)&id0);
	cvSetMouseCallback(nWindows[1], pointSelection_onMouse, (int*)&id1);
	cvSetMouseCallback(nWindows[2], pointSelection_onMouse, (int*)&id2);
	

	//flags
	bool bShouldStop = false;

	
	Mat depthMat[NUM_SENSORS];

	for (int i = 0; i < NUM_SENSORS; i++)
	{
		depthImages[i] = Mat(XN_VGA_Y_RES, XN_VGA_X_RES, CV_8UC3);
		rgbImages[i] = Mat(XN_VGA_Y_RES, XN_VGA_X_RES, CV_8UC3);
		depthMat[i] = Mat(XN_VGA_Y_RES, XN_VGA_X_RES, CV_16U);
	}

	KINECTS_DISPLACEMENT = max(abs(kinects[0].translation(0)), abs(kinects[2].translation(0))); //MAXIMUM TRANSLATION IN THE HORIZONTAL AXIS
	MAX_RANGE = ActivityMap_Utils::MAX_Z_TRANS + KINECTS_DISPLACEMENT; 
	bool first = true;

	int waitTime = 1;

	int nPoints = 0;
	while (!bShouldStop)
	{		
		cout << "Frames: " << frames << endl;

		for (int i = 0; i < NUM_SENSORS; i++)
			kinects[i].waitAndUpdate();
		
		for (int i = 0; i < NUM_SENSORS;  i++)
		{
			depthMaps[i] = kinects[i].getDepthMap();
			rgbMaps[i] = kinects[i].getRGBMap();
			//new part
			kinects[i].getDepthImage(depthImages[i]);
			kinects[i].getRGBImage(rgbImages[i]);
		}

		
		
		//for (int i = 0; i < NUM_SENSORS; i++)
		//{
		//	combineRGBD(rgbImages[i], depthMaps[i]);
		//}

		imshow(nWindows[0], rgbImages[0]);
		imshow(nWindows[1], rgbImages[1]);
		imshow(nWindows[2], rgbImages[2]);
		int c = waitKey(waitTime);
		if (c == 13)
			waitTime = 0;
		
		if (c == 27)
			bShouldStop = true;
		
		cout << "Frame: " << frames << endl;
		frames++;
	}

	for (int i = 0; i < NUM_SENSORS; i++)
	{
		kinects[i].stopDevice();
  		kinects[i].shutDown();
	}
}


//Print the foreground point cloud
void pointCloudCorrespondence()
{
	int tilt = 0;
	tiltTXT >> tilt;
	char* paths[3];
/*	paths[0] = "d:/Emilio/Tracking/DataSet/sb125/CalibrationVideos/kinect0_calib.oni";
	paths[1] = "d:/Emilio/Tracking/DataSet/sb125/CalibrationVideos/kinect1_calib.oni";
	paths[2] = "d:/Emilio/Tracking/DataSet/sb125/CalibrationVideos/kinect2_calib.oni";
*/
	paths[0] = "d:/Emilio/Tracking/DataSet/kinect0_calib.oni";
	paths[1] = "d:/Emilio/Tracking/DataSet/kinect1_calib.oni";
	paths[2] = "d:/Emilio/Tracking/DataSet/kinect2_calib.oni";

	p3D[0].X = -1; p3D[1].X = -1;
	
	const XnRGB24Pixel* rgbMaps[NUM_SENSORS];

	for (int i = 0; i < NUM_SENSORS; i++)
	{
		kinects[i].initDevice(i, REF_CAM, true, paths[i]);
		kinects[i].startDevice();
		kinects[i].tilt(tilt);
	}

	Mat whiteBack;

	nWindows[0] =  "depth 0";
	nWindows[1] =  "depth 1";
	nWindows[2] =  "depth 2";

	namedWindow(nWindows[0]);
	namedWindow(nWindows[1]);
	namedWindow(nWindows[2]);
	
	//flags
	bool bShouldStop = false;
	bool bgComplete = true;

	//Mat rgbImages[NUM_SENSORS];
	Mat depthMat[NUM_SENSORS];
	Mat masks[NUM_SENSORS];
	Mat grey;

	BackgroundDepthSubtraction subtractors[NUM_SENSORS];
	int numberOfForegroundPoints[NUM_SENSORS];

	XnPoint3D* pointsFore2D [NUM_SENSORS];
	XnPoint3D* points3D[NUM_SENSORS];

	for (int i = 0; i < NUM_SENSORS; i++)
	{
		depthImages[i] = Mat(XN_VGA_Y_RES, XN_VGA_X_RES, CV_8UC3);
		rgbImages[i] = Mat(XN_VGA_Y_RES, XN_VGA_X_RES, CV_8UC3);
		depthMat[i] = Mat(XN_VGA_Y_RES, XN_VGA_X_RES, CV_16U);
		pointsFore2D[i] = new XnPoint3D[MAX_FORGROUND_POINTS];
		numberOfForegroundPoints[i] = 0;
	}

	KINECTS_DISPLACEMENT = max(abs(kinects[0].translation(0)), abs(kinects[2].translation(0))); //MAXIMUM TRANSLATION IN THE HORIZONTAL AXIS
	MAX_RANGE = ActivityMap_Utils::MAX_Z_TRANS + KINECTS_DISPLACEMENT; 
	bool first = true;

	int waitTime = 1;

	int nPoints = 0;
	while (!bShouldStop)
	{		
		cout << "Frames: " << frames << endl;

		for (int i = 0; i < NUM_SENSORS; i++)
			kinects[i].waitAndUpdate();
		
		for (int i = 0; i < NUM_SENSORS;  i++)
		{
			depthMaps[i] = kinects[i].getDepthMap();
			rgbMaps[i] = kinects[i].getRGBMap();
			//new part
			kinects[i].getDepthImage(depthImages[i]);
			kinects[i].getRGBImage(rgbImages[i]);
			
			//Creates a matrxi with depth values (ushort)
			createDepthMatrix(depthMaps[i], depthMat[i]);

			//to create a mask for the noise (depth img is threhold)
			cvtColor(depthImages[i],grey,CV_RGB2GRAY);
			masks[i] = grey > 250; //mask that identifies the noise (1)
		}
		nPoints = 0;
		if (bgComplete)// && frames > 84) //Trans must be true
		{
			for (int i = 0; i < NUM_SENSORS; i++)
			{
				numberOfForegroundPoints[i] = subtractors[i].subtraction(pointsFore2D[i], &(depthMat[i]), &(masks[i]));
				int totalPoints = numberOfForegroundPoints[i];
				nPoints += totalPoints;
				if (totalPoints > 0)
				{
					points3D[i] = new XnPoint3D[totalPoints];
					kinects[i].arrayBackProject(pointsFore2D[i], points3D[i], totalPoints);
					
					//kinects[i].tiltCorrection(points3D[i], totalPoints);
					//frame 809 to see how the x spreads with distance
					if (frames == 729)
					{
						ofstream* out;
						if (i == 0)
							out = &outFPK0;
						else if (i == 1)
							out = &outFPK1;
						else
							out = &outFPK2;

						for (int n = 0; n < totalPoints; n++)
						{
							XnPoint3D p = points3D[i][n];
							if (p.Z != 0)
							{
								*out << (float)p.X << " " << (float)p.Y << " " << (float)p.Z << endl;
							}
						}
					}
					delete []points3D[i];
					
				}
			}
		}

		for (int i = 0; i < NUM_SENSORS; i++)
		{
			combineRGBD(rgbImages[i], depthMaps[i]);
		}

		imshow(nWindows[0], rgbImages[0]);
		imshow(nWindows[1], rgbImages[1]);
		imshow(nWindows[2], rgbImages[2]);
		int c = waitKey(waitTime);
		if (c == 13)
		{
			waitTime = 0;
			debugFrame = frames + 1;
		}	
		if (c == 27)
			bShouldStop = true;
		cout << "Frame: " << frames << endl;
		frames++;
	}

	for (int i = 0; i < NUM_SENSORS; i++)
	{
		kinects[i].stopDevice();
  		kinects[i].shutDown();
	}
}

void checkCalibration()
{
	int tilt = 0;
	tiltTXT >> tilt;
	char* paths[3];
/*	paths[0] = "d:/Emilio/Tracking/DataSet/sb125/CalibrationVideos/kinect0_calib.oni";
	paths[1] = "d:/Emilio/Tracking/DataSet/sb125/CalibrationVideos/kinect1_calib.oni";
	paths[2] = "d:/Emilio/Tracking/DataSet/sb125/CalibrationVideos/kinect2_calib.oni";
*/
	paths[0] = "d:/Emilio/Tracking/DataSet/kinect0_calib.oni";
	paths[1] = "d:/Emilio/Tracking/DataSet/kinect1_calib.oni";
	paths[2] = "d:/Emilio/Tracking/DataSet/kinect2_calib.oni";
	
	for (int i = 0; i < NUM_SENSORS; i++)
	{
		kinects[i].initDevice(i, REF_CAM, true, paths[i]);
		kinects[i].startDevice();
		kinects[i].tilt(tilt);
	}

	nWindows[0] =  "depth 0";
	nWindows[1] =  "depth 1";
	nWindows[2] =  "depth 2";

	namedWindow(nWindows[0]);
	namedWindow(nWindows[1]);
	namedWindow(nWindows[2]);

	int id0 = 0;
	int id1 = 1;
	int id2 = 2;
	//Check calibration accuracy
	int ids01[2] = {id0, id1};
	int ids10[2] = {id1, id0};
	//int ids12[2] = {id1, id2};
	//int ids21[2] = {id2, id1};
	//01 and 10
	cvSetMouseCallback(nWindows[0], pointSelection_onMouse_single, ids01);
	cvSetMouseCallback(nWindows[1], pointSelection_onMouse_single, ids10);
	//12 and 21
	//cvSetMouseCallback(nWindows[2], pointSelection_onMouse_single, ids21);
	//cvSetMouseCallback(nWindows[1], pointSelection_onMouse_single, ids12);
	
	//flags
	bool bShouldStop = false;
	//Mat rgbImages[NUM_SENSORS];
	for (int i = 0; i < NUM_SENSORS; i++)
	{
		depthImages[i] = Mat(XN_VGA_Y_RES, XN_VGA_X_RES, CV_8UC3);
		rgbImages[i] = Mat(XN_VGA_Y_RES, XN_VGA_X_RES, CV_8UC3);
	}

	KINECTS_DISPLACEMENT = max(abs(kinects[0].translation(0)), abs(kinects[2].translation(0))); //MAXIMUM TRANSLATION IN THE HORIZONTAL AXIS
	MAX_RANGE = ActivityMap_Utils::MAX_Z_TRANS + KINECTS_DISPLACEMENT; 
	bool first = true;

	int waitTime = 1;
	while (!bShouldStop)
	{		
		cout << "Frames: " << frames << endl;

		for (int i = 0; i < NUM_SENSORS; i++)
			kinects[i].waitAndUpdate();
		
		for (int i = 0; i < NUM_SENSORS;  i++)
		{
			depthMaps[i] = kinects[i].getDepthMap();
			//new part
			kinects[i].getDepthImage(depthImages[i]);
			kinects[i].getRGBImage(rgbImages[i]);
			combineRGBD(rgbImages[i], depthMaps[i]);
		}
		
		imshow(nWindows[0], depthImages[0]);
		imshow(nWindows[1], depthImages[1]);
		imshow(nWindows[2], depthImages[2]);
		int c = waitKey(waitTime);
		if (c == 13)
			waitTime = 0;
		
		if (c == 27)
			bShouldStop = true;
		
		cout << "Frame: " << frames << endl;
		frames++;
	}

	for (int i = 0; i < NUM_SENSORS; i++)
	{
		kinects[i].stopDevice();
  		kinects[i].shutDown();
	}
}
int main(int argc, char* argv[])
{

	pointCorrespondences();

	//pointCloudCorrespondence();

	//checkCalibration();

	return 0;

}