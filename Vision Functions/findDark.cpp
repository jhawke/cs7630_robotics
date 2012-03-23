// OpenCV_Helloworld.cpp : Defines the entry point for the console application.
// Created for build/install tutorial, Microsoft Visual Studio and OpenCV 2.2.0

#include <iostream>
#include "stdafx.h"
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

int *findDark(int widthRes, int lengthRes, IplImage* img){
	// To convert from ros to iplimage
	//img = bridge.imgmsg_to_cv(image_message, desired_encoding="passthrough");


	/*
	[wxl] s=[(0,0),(0,1),...,(0,l),(1,0),...,(1,l),...,(w,0),...,(w,l)]
     l=image height; w= image width;

	 function outputs the avg brightness of grid of set resolution. The higher the returned value for the grid the brighter the grid location is. 
	*/
		int *s = new int[widthRes*lengthRes]; // set size of output int

		//Full HSV color Image
		IplImage* imageHSV = cvCreateImage( cvGetSize(img),8,3); 
		cvCvtColor(img,imageHSV,CV_BGR2HSV);

		//get separate planes
		IplImage* planeH = cvCreateImage(cvGetSize(img),8,1); //Hue
		IplImage* planeS = cvCreateImage(cvGetSize(img),8,1); //Saturation
		IplImage* planeV = cvCreateImage(cvGetSize(img),8,1); //Brightness
		cvCvtPixToPlane(imageHSV, planeH,planeS,planeV,0);//Extract the 3 color components

		//Intialize Loop Variables
		int widthint= (int) planeV->width/widthRes; // pixel width of grid
		int lengthint= (int) planeV->height/lengthRes;//pixel height (length) of grid
		double n= 0; // total num of pixels per section
		IplImage* grid= cvCreateImage(cvSize(widthint,lengthint),imageHSV->depth,1); //Grid Image
	
		//Loop cycling throught each grid of image and calculating average brigntness of each grid
		for(int w=0; w<widthRes; w++){
			for(int l=0; l<lengthRes;l++){
				s[w*3+l]=0;
				cvSetImageROI(planeV,cvRect(widthint*w,lengthint*l,widthint,lengthint));
				cvCopy(planeV,grid);
				cvResetImageROI(planeV);
				n=0;
				for( int y = 0; y < grid->height; y++ )
				{
				   unsigned char* row = &CV_IMAGE_ELEM( grid, unsigned char, y, 0 );
				   for( int x = 0; x < grid->width*grid->nChannels; x += grid->nChannels )
					 {
					   s[w*3+l]+=row[x]; // access the pixel in the first channel 
					   n++;
					   }
				}

				s[w*3+l]/=n;
			}
		}

		//Output
		return s;

}