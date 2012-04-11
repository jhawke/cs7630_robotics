/*!
 * \file raptor_find_dark.cpp
 * \brief Image processing framework.
 *
 * This file provides hooks to image subscriptions and sets up the output server for the mcom node.
 *
 * \author omernick
 * \date March 22,2012
 */

#include <raptor/raptor_find_dark.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <raptor/polar_histogram.h>
#include <raptor/obstacle_histogram.h>
#include <string>
#include <limits.h>
#include <boost/array.hpp>
#include <cxcore.h>
#include <iostream>
#include <opencv/cv.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <cv_bridge/cv_bridge.h>
#include "cv_bridge/CvBridge.h"
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>

//Put any local defines here.

#define YOUR_MOTHER "town bicycle"
#define MAX_V 255

using namespace std;

raptor_find_dark::raptor_find_dark()
{
  isFirstImg = true;
    //Subscribe to the image producing publisher. (/gscam/image_raw)
    image_subscription = node.subscribe<sensor_msgs::Image>("/gscam/image_raw",1,&raptor_find_dark::handle_new_image, this);
    //Advertise your service.
     cv::namedWindow("View",1);
    vector_gen = node.advertiseService("raptor_find_dark_srv",&raptor_find_dark::get_vector_field,this);
    volume = 1;
}
double *findDark(int widthRes, int lengthRes, IplImage* img){
	// To convert from ros to iplimage
	//img = bridge.imgmsg_to_cv(image_message, desired_encoding="passthrough");


	/*
	[wxl] s=[(0,0),(0,1),...,(0,l),(1,0),...,(1,l),...,(w,0),...,(w,l)]
     l=image height; w= image width;

	 function outputs the avg brightness of grid of set resolution. The higher the returned value for the grid the brighter the grid location is. 
	*/
		double *s = new double[widthRes*lengthRes]; // set size of output int

		//Full HSV color Image
		IplImage* imageHSV = cvCreateImage( cvGetSize(img),8,3); 
		cvCvtColor(img,imageHSV,CV_BGR2HSV);
		ROS_DEBUG("preimagebreakout");
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
		ROS_DEBUG("preloop");
		//Loop cycling throught each grid of image and calculating average brigntness of each grid
		for(int w=0; w<widthRes; w++){
			for(int l=0; l<lengthRes;l++){
				s[w*lengthRes+l]=0;
				cvSetImageROI(planeV,cvRect(widthint*w,lengthint*l,widthint,lengthint));
				cvCopy(planeV,grid);
				cvResetImageROI(planeV);
				n=0;
				for( int y = 1; y <= grid->height; y++ ){
				   unsigned char* row = &CV_IMAGE_ELEM( grid, unsigned char, y, 0 );
				   for( int x = 1; x <= grid->width*grid->nChannels; x += grid->nChannels ){
					   s[w*lengthRes+l]+=(MAX_V-row[x]); // access the pixel in the first channel 
					   n++;
					   }
				}

				s[w*lengthRes+l]/=n;
				//cout<<w*lengthRes+l<<"_";
				//cout<<s[w*2+1]<<"  ";
			}
		}
		ROS_DEBUG("prerelease");
		cvReleaseImage(&planeH);
		cvReleaseImage(&planeS);
		cvReleaseImage(&planeV);
		cvReleaseImage(&imageHSV);
		cvReleaseImage(&grid);
		ROS_DEBUG("done");
		//Output
		return s;

}

raptor_find_dark::~raptor_find_dark()
{
  
}

bool raptor_find_dark::get_vector_field(raptor::polar_histogram::Request &req, raptor::polar_histogram::Response &res)
{
  ROS_DEBUG("Function 'name' called.");
  //Your image processing shit goes here.
  double* Out = new double[360];
	for(int i=0;i<360;i++)
		Out[i]=0;

	int viewFieldMin = 159;// view range of robot out of 360
	int viewFieldMax = 199;

	double depthIntensity[5] = {0,0,.5,1,.5};
	//

	double* dark= findDark(viewFieldMax-viewFieldMin,sizeof(depthIntensity)/sizeof(depthIntensity[0]),img);
	//[wxl] dark=[(0,0),(0,1),...,(0,l),(1,0),...,(1,l),...,(w,0),...,(w,l)]
	for(int k=0;k<360;k++){
		if(k<viewFieldMin)
			Out[k]=0;
		else if(k>=viewFieldMax)
			Out[k]=0;
		else{
			for(int j=0;j<sizeof(depthIntensity)/sizeof(depthIntensity[0]);j++){
				Out[k]+=dark[(k-viewFieldMin)*((sizeof(depthIntensity)/sizeof(depthIntensity[0])))+j]*depthIntensity[j];
			}
			//if(Out[k]<0)
				//Out[k]=0;
		}
	}
	for(int i=0;i<360;i++)
		res.hist[i]=(int)(Out[i]*volume);
  //Finally, fill up the array res.hist
  //res.hist = WHATEVER;
  return true;
}

void raptor_find_dark::handle_new_image(const sensor_msgs::Image::ConstPtr& msg)
{
    string cv_encoding="passthrough";
    sensor_msgs::CvBridge bridge;
    cv_bridge::CvImagePtr cv_ptr;
   try
   {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
   }
   catch (cv_bridge::Exception& e)
   {
   ROS_ERROR("cv_bridge exception: %s", e.what());
   return;
   }
  donkey_kong=(IplImage)cv_ptr->image; //IT'S ON LIKE DONKEY KONG
  img = &donkey_kong;
  //img= cvGetImage(&(cv_ptr->image),&stub);
  //img = bridge.imgmsg_to_cv(msg, desired_encoding="passthrough");
  if(isFirstImg)
  {
    cvShowImage("View",img);
    //cv::imshow("View",cv_ptr->image);
    cv::waitKey(3);
    //isFirstImg = false;
    //ROS_INFO("ASS"); 
    
  }
  
  //Put your pre-handler shite in here. You can just copy the latest or some damn thing.
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"raptor_find_dark");
  
  raptor_find_dark ip_core;
  
  ros::spin();
  
  return EXIT_SUCCESS; 
}
