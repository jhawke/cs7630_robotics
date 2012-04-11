/*!
 * \file raptor_generic_ip.cpp
 * \brief Image processing framework.
 *
 * This file provides hooks to image subscriptions and sets up the output server for the mcom node.
 *
 * \author omernick
 * \date March 22,2012
 */

#include <raptor/raptor_generic_ip.h>

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
#include <blob/blob.h>
#include <blob/BlobResult.h>

//Put any local defines here.

#define YOUR_MOTHER "town bicycle"
#define MAX_V 255

using namespace std;

raptor_generic_ip::raptor_generic_ip()
{
  isFirstImg = true;
    //Subscribe to the image producing publisher. (/gscam/image_raw)
    image_subscription = node.subscribe<sensor_msgs::Image>("/gscam/image_raw",1,&raptor_generic_ip::handle_new_image, this);
    //Advertise your service.
     cv::namedWindow("View",1);
    vector_gen = node.advertiseService("raptor_generic_ip_srv",&raptor_generic_ip::get_vector_field,this);
    volume = 1;
}
int HSV_filter(int h, int s, int v, int threshold, int hue, int sat, int val) {
	int FilteredColor[3] = {hue, sat, val}; 
	int diff =  (FilteredColor[0]-h)*(FilteredColor[0]-h) +
				(FilteredColor[1]-s)*(FilteredColor[1]-s);
	
	if(diff < threshold) return abs(diff-threshold); /** If here, it has passed! */
	return 0; /** With 0 this is discarded */
}
double *findObstacle(IplImage *img, int hue,int sat,int val,int threshold, double blobLowLimit,double blobHighLimit){
	// Input HSV value of color blob your seeking, acceptable threshold of that color, and Min and Max blob sizes beeing sought out. 
	//Ouput: pointer to data array, size[#ofblobs*3+1]; Format data=[Number of Blobs, Area1,X of center1, y of center1, Area2,X of center2,y of center2,...,areaN,X of centerN, Y of centerN];
    cvShowImage("View",img);
    //cv::imshow("View",cv_ptr->image);
    cv::waitKey(3);
    

	// Image variables
	IplImage* imageSmooth = cvCreateImage( cvGetSize(img),8,3);//Gausian Filtered image
	IplImage* imageHSV = cvCreateImage( cvGetSize(img),8,3); //HSV image
	IplImage* i1 = cvCreateImage( cvGetSize(img),8,1);//desired color filtered image
	IplImage* planeH = cvCreateImage(cvGetSize(img),8,1); //Hue
	IplImage* planeS = cvCreateImage(cvGetSize(img),8,1); //Saturation
	IplImage* planeV = cvCreateImage(cvGetSize(img),8,1); //Brightness
	//Blob variables
	CBlobResult blobs;
	CBlob blob;
	CBlobGetXCenter getXCenter;
	CBlobGetYCenter getYCenter;
	//Output Variable
	//Gausian Filter
	cvSmooth(img,imageSmooth,CV_GAUSSIAN,7,9,0,0);
	//Covert RGB to HSV
	cvCvtColor(imageSmooth,imageHSV,CV_BGR2HSV);
	cvCvtPixToPlane(imageHSV, planeH,planeS,planeV,0);//Extract the 3 color components
	//Filter image for desired Color, output image with only desired color highlighted remaining
	for( int y = 0; y < planeH->height; y++ ){
		unsigned char* h = &CV_IMAGE_ELEM( planeH, unsigned char, y, 0 );
		unsigned char* s = &CV_IMAGE_ELEM( planeS, unsigned char, y, 0 );
		unsigned char* v = &CV_IMAGE_ELEM( planeV, unsigned char, y, 0 );
		for( int x = 0; x < planeH->width*planeH->nChannels; x += planeH->nChannels ){
			 int f= HSV_filter(h[x],s[x],v[x],threshold,hue,sat,val);
			 if(f){
				 ((uchar *)(i1->imageData + y*i1->widthStep))[x]=0;
			 }else{
				 ((uchar *)(i1->imageData + y*i1->widthStep))[x]=255;
			 }
		}
	}//debug
	cvNamedWindow("i1",1);
	cvShowImage("i1",i1);
	cvWaitKey(3);
	//Blob stuff
	blobs = CBlobResult(i1,NULL,0);   //Get blobs of image
	blobs.Filter(blobs,B_INCLUDE,CBlobGetArea(),B_INSIDE,blobLowLimit,blobHighLimit);  // Filter Blobs with min and max size
	//Set up data array
	double *data= new double[blobs.GetNumBlobs()*4+1];
	data[0]=blobs.GetNumBlobs();// Set first data value to total number of blobs
	//cout<<data[0]<<"  ";
	for (int i = 0; i < blobs.GetNumBlobs(); i++ ){ // Get Blob Data 
	    blob = blobs.GetBlob(i);//cycle through each blob
		//data[i*3+1]=blob.area;//blob area
		//cout<<blob.area<<"   ";
		data[i*4+2]= blob.MinX(); //X min
		data[i*4+3]= blob.MaxX(); //X max
		data[i*4+4]= blob.MaxY(); //Y max

		//debug
		blob.FillBlob(imageSmooth, cvScalar(255, 0, 0)); // This line will give you a visual marker on image for the blob if you want it for testing or something
    }
    cvNamedWindow("imSmooth",1);
    cvShowImage("imSmooth",imageSmooth);
    //cv::imshow("View",cv_ptr->image);
    cv::waitKey(3);
    
    
      cvReleaseImage(&imageSmooth);
      cvReleaseImage(&imageHSV);
      cvReleaseImage(&i1);
      cvReleaseImage(&planeH);
      cvReleaseImage(&planeS);
      cvReleaseImage(&planeV);
	return data; //return pointer to data array
}

raptor_generic_ip::~raptor_generic_ip()
{
  
}

bool raptor_generic_ip::get_vector_field(raptor::obstacle_histogram::Request &req, raptor::obstacle_histogram::Response &res)
{
 double* Out = new double[360];
	for(int i=0;i<360;i++)
		Out[i]=0;
	int viewFieldMin = 159;// view range of robot out of 360
	int viewFieldMax = 200;
	int triangleSlope = 3; // triangle slope for fuzzyfilter
	int depthIntensity[8] = {0,0,0,0,0,5,10,15};
	//
	int H= 67;// H value of all searched colors
	int S= 205;//s val
	int V= 219;//V val
	int threshold = 5000;// color search threshold
	double blobLowLimit = 500;// blob size limits
	double blobHighLimit = 1000000;

	double* blob= findObstacle(img,H,S,V,threshold,blobLowLimit,blobHighLimit);
	ROS_DEBUG("There are %g blobs.",blob[0]);
	for(int j=0; j<((int)blob[0]);j++){// make fuzzy
		int blobXmin= blob[j*4+2];
		int blobXmax= blob[j*4+3];
		int blobY= blob[j*4+4];
		ROS_DEBUG("Blob y-centroid is %d",blobY);
		int maxV=0;
		maxV=depthIntensity[(int)blobY/(img->height/(sizeof(depthIntensity)/sizeof(depthIntensity[0])))];
		ROS_DEBUG("Depth multiplier is %d",maxV);
		for(int k=0;k<360;k++){
			int val;
			//debug
			if(k<((int)((double)blobXmin/(double)img->width*(double)(viewFieldMax-viewFieldMin))+viewFieldMin))
				val = maxV-triangleSlope*abs(k-((int)((double)blobXmin/(double)img->width*(double)(viewFieldMax-viewFieldMin))+viewFieldMin));
			else if(k>((int)((double)blobXmax/(double)img->width*(double)(viewFieldMax-viewFieldMin))+viewFieldMin))
				val = maxV-triangleSlope*abs(k-((int)((double)blobXmax/(double)img->width*(double)(viewFieldMax-viewFieldMin))+viewFieldMin));
			else
				val= maxV;
			if(val<0) val=0;
			Out[k]+=val;
		}
	}// UNFUZZY
	
	for(int k=0;k<360;k++){
		if(Out[k]<5)
			Out[k]=0;
		else 
			Out[k]=1;
	}
	int r=0;
	for(int i=viewFieldMin;i<viewFieldMax;i++)
	{
		res.hist[r]=(int)(Out[i]*volume);
		r++;
	}
  //Finally, fill up the array res.hist
  //res.hist = WHATEVER;
  return true;
}

void raptor_generic_ip::handle_new_image(const sensor_msgs::Image::ConstPtr& msg)
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
    //cv::imshow("View",cv_ptr->image);
    cv::waitKey(3);
    //isFirstImg = false;
    //ROS_INFO("ASS"); 
    
  }
  
  //Put your pre-handler shite in here. You can just copy the latest or some damn thing.
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"raptor_generic_ip");
  
  raptor_generic_ip ip_core;
  
  ros::spin();
  
  return EXIT_SUCCESS; 
}
