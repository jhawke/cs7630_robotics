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
#include <sensor_msgs/image_encodings.h>
#include <boost/array.hpp>
#include <std_msgs/Int16.h>

//Put any local defines here.

#define YOUR_MOTHER "town bicycle"
#define MAX_V 255
#define VALUE_LIM 30
#define START_THRESH 15
#define OBSTACLE_THRESHOLD 5
using namespace std;

  //int16_t dark_thresh_val;

raptor_generic_ip::raptor_generic_ip()
{
  threshold = START_THRESH;
  //dark_thresh_val = VALUE_LIM;
  isFirstImg = false;
    //Subscribe to the image producing publisher. (/gscam/image_raw)
    dark_thresh = node.subscribe<std_msgs::Int16>("debug_thresh",1,&raptor_generic_ip::alter_thrsval,this);
    image_subscription = node.subscribe<sensor_msgs::Image>("/gscam/image_raw",1,&raptor_generic_ip::handle_new_image, this);
    //Advertise your service.
     cv::namedWindow("Obstacle_Raw",1);
     cv::namedWindow("Obstacle_InFunction",1);
     cv::namedWindow("Obstacle_Value",1);
     cv::namedWindow("Obstacle_Hue",1);
     cv::namedWindow("Obstacle_Triscale",1);
     cv::namedWindow("Obstacle_Binary",1);
     cv::namedWindow("Obstacle_Detected",1);
     //img=cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,3); 
    vector_gen = node.advertiseService("raptor_generic_ip_srv",&raptor_generic_ip::get_vector_field,this);
    volume = 1;
}
void raptor_generic_ip::alter_thrsval(const std_msgs::Int16::ConstPtr &msg)
{
  //dark_thresh_val = msg->data;
  threshold = msg->data;
}
int HSV_filter(int h, int s, int v, int threshold,int thresholdS, int ref_hue, int ref_sat, int ref_val) {
	int FilteredColor[3] = {ref_hue, ref_sat, ref_val}; 
	int diff = abs((h-FilteredColor[0]));//*(h-FilteredColor[0])+(s-FilteredColor[1])*(s-FilteredColor[1]));
	//int diff = abs(FilteredColor[0]-h);
	
	//ROS_INFO("FC[2] is %d",FilteredColor[2]);
	if(diff < threshold){return 1;}
	else if(s<thresholdS){return 2;}//abs(diff-threshold); /** If here, it has passed! */
	return 0; /** With 0 this is discarded */
}
double *findObstacle(IplImage *l_img, int hue,int sat,int val,int threshold, double blobLowLimit,double blobHighLimit){
	// Input HSV value of color blob your seeking, acceptable threshold of that color, and Min and Max blob sizes beeing sought out. 
	//Ouput: pointer to data array, size[#ofblobs*3+1]; Format data=[Number of Blobs, Area1,X of center1, y of center1, Area2,X of center2,y of center2,...,areaN,X of centerN, Y of centerN];
    

    

	// Image variables
	IplImage* imageSmooth = cvCreateImage( cvGetSize(l_img),8,3);//Gausian Filtered image
	IplImage* imageHSV = cvCreateImage( cvGetSize(l_img),8,3); //HSV image
	IplImage* i1 = cvCreateImage( cvGetSize(l_img),8,1);//desired color filtered image
	IplImage* i_ts = cvCreateImage( cvGetSize(l_img),8,1);//desired color filtered image
	IplImage* planeH = cvCreateImage(cvGetSize(l_img),8,1); //Hue
	IplImage* planeS = cvCreateImage(cvGetSize(l_img),8,1); //Saturation
	IplImage* planeV = cvCreateImage(cvGetSize(l_img),8,1); //Brightness
	//Blob variables
	CBlobResult blobs;
	CBlob blob;
	CBlobGetXCenter getXCenter;
	CBlobGetYCenter getYCenter;
	//Output Variable
	//Gausian Filter
	cvSmooth(l_img,imageSmooth,CV_GAUSSIAN,13,13,0,0);
	//cvShowImage("View2a",imageSmooth);
	
	//Covert RGB to HSV
	cvCvtColor(imageSmooth,imageHSV,CV_BGR2HSV);
	cvCvtPixToPlane(imageHSV, planeH,planeS,planeV,0);//Extract the 3 color components
	cvShowImage("Obstacle_Value",planeV);
	cvShowImage("Obstacle_Hue",planeH);
	cv::waitKey(3);
	//Filter image for desired Color, output image with only desired color highlighted remaining
	for( int y = 0; y < planeH->height; y++ ){
		unsigned char* h = &CV_IMAGE_ELEM( planeH, unsigned char, y, 0 );
		unsigned char* s = &CV_IMAGE_ELEM( planeS, unsigned char, y, 0 );
		unsigned char* v = &CV_IMAGE_ELEM( planeV, unsigned char, y, 0 );
		for( int x = 0; x < planeH->width*planeH->nChannels; x += planeH->nChannels ){
		  //if(x<5){ROS_INFO("hsv[x] is %d,%d,%d",h[x],v[x],x]);}
			 int f= HSV_filter(h[x],s[x],v[x],threshold,VALUE_LIM,hue,sat,val);
			 if(f==1)
			 {
				 ((uchar *)(i1->imageData + y*i1->widthStep))[x]=0;
				 ((uchar *)(i_ts->imageData + y*i_ts->widthStep))[x]=0;
			 }
			 else if(f==2)
			 {
				((uchar *)(i1->imageData + y*i1->widthStep))[x]=0;
				((uchar *)(i_ts->imageData + y*i_ts->widthStep))[x]=127;
			 }
			 else if(f==0)
			 {
				 ((uchar *)(i_ts->imageData + y*i_ts->widthStep))[x]=255;
				 ((uchar *)(i1->imageData + y*i1->widthStep))[x]=255;
			 }
		}
	}//debug

	cvShowImage("Obstacle_Binary",i1);
	cvShowImage("Obstacle_Triscale",i_ts);
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
		data[i*4+2]= blob.MinX(); //X min
		data[i*4+3]= blob.MaxX(); //X max
		data[i*4+4]= blob.MaxY(); //Y max

		//debug
		blob.FillBlob(imageSmooth, cvScalar(255, 0, 0)); // This line will give you a visual marker on image for the blob if you want it for testing or something
    }
    
   //
   
   cvShowImage("Obstacle_Detected",imageSmooth);
    //cv::imshow("View",cv_ptr->image);
    cv::waitKey(3);
    
    
      cvReleaseImage(&imageSmooth);
      cvReleaseImage(&imageHSV);
      cvReleaseImage(&i1);
      cvReleaseImage(&i_ts);
      cvReleaseImage(&planeH);
      cvReleaseImage(&planeS);
      cvReleaseImage(&planeV);
	return data; //return pointer to data array
}

raptor_generic_ip::~raptor_generic_ip()
{
  cvDestroyAllWindows();
  //cvReleaseImage(&img);
}

bool raptor_generic_ip::get_vector_field(raptor::obstacle_histogram::Request &req, raptor::obstacle_histogram::Response &res)
{
  if(isFirstImg)
  {
    IplImage* image_local;
    image_local = cvCloneImage(img);
    cvShowImage("Obstacle_InFunction",image_local);
    cv::waitKey(3);
	double* Out = new double[360];
	for(int i=0;i<360;i++)
		Out[i]=0;
	int viewFieldMin = 159;// view range of robot out of 360
	int viewFieldMax = 200;
	int triangleSlope[1] = {2};//{0,0,0,0,3,2,1,1}; // triangle slope for fuzzyfilter
	int depthIntensity[1] = {10};//{0,0,0,0,5,5,10,15};
	//
	int H= 60;// H value of all searched colors
	int S= 120;//s val
	int V= 219;//V val
	//int threshold = 5;// color search threshold
	ROS_DEBUG("OBST_FIND:GET_VECTOR_FIELD:Threshold is %d",threshold);
	double blobLowLimit = 500;// blob size limits
	double blobHighLimit = 1000000/8;
	int heightInt= (int) image_local->height/8; //(sizeof(depthIntensity)/sizeof(depthIntensity[0])); // pixel width of grid
	for(int i=0;i<(sizeof(depthIntensity)/sizeof(depthIntensity[0]));i++){
		cvSetImageROI(image_local,cvRect(0,(image_local->height)-heightInt,(image_local->width)-1,heightInt-1));
		double* blob= findObstacle(image_local,H,S,V,threshold,blobLowLimit,blobHighLimit);
		ROS_DEBUG("OBST_FIND:GET_VECTOR_FIELD:There are %g blobs.",blob[0]);
		for(int j=0; j<((int)blob[0]);j++){// make fuzzy
			int blobXmin= blob[j*4+2];
			int blobXmax= blob[j*4+3];
			int blobY= blob[j*4+4];
			ROS_DEBUG("Blob y-centroid is %d",blobY);
			int maxV=0;
			int index = i;//(int)blobY/(image_local->height/(sizeof(depthIntensity)/sizeof(depthIntensity[0])));
			maxV=depthIntensity[index];
			ROS_DEBUG("OBST_FIND:GET_VECTOR_FIELD:Depth multiplier is %d",maxV);
			for(int k=0;k<360;k++){
				int val;
				//debug
				if(k<((int)((double)blobXmin/(double)image_local->width*(double)(viewFieldMax-viewFieldMin))+viewFieldMin))
					val = maxV-triangleSlope[index]*abs(k-((int)((double)blobXmin/(double)image_local->width*(double)(viewFieldMax-viewFieldMin))+viewFieldMin));
				else if(k>((int)((double)blobXmax/(double)image_local->width*(double)(viewFieldMax-viewFieldMin))+viewFieldMin))
					val = maxV-triangleSlope[index]*abs(k-((int)((double)blobXmax/(double)image_local->width*(double)(viewFieldMax-viewFieldMin))+viewFieldMin));
				else
					val= maxV;
				if(val<0) val=0;
				Out[k]+=val;
			}
		}
	}// UNFUZZY
	cvResetImageROI(image_local);
	//ROS_DEBUG("weeeeeeeeeeeee");
	for(int k=viewFieldMin;k<viewFieldMax;k++){
		if(Out[k]<OBSTACLE_THRESHOLD){Out[k]=1;}
		else {Out[k]=0;}
	}

	int r=0;
	for(int i=viewFieldMin;i<viewFieldMax;i++)
	{
		res.hist[r]=(int)(Out[i]*volume);
		r++;
	}
	
	cvReleaseImage(&image_local);
  }
  else
  {
  //Finally, fill up the array res.hist
    boost::array<int16_t,41> t = {0};
    res.hist = t;
  }
  return true;
}

void raptor_generic_ip::handle_new_image(const sensor_msgs::Image::ConstPtr& msg)
{
    string cv_encoding="passthrough";

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
    cvShowImage("Obstacle_Raw",img);
    cv::waitKey(3);
  isFirstImg = true;
// your pre-handler shite in here. You can just copy the latest or some damn thing.
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"raptor_generic_ip");
  
  raptor_generic_ip ip_core;
  
  ros::spin();
  
  return EXIT_SUCCESS; 
}

