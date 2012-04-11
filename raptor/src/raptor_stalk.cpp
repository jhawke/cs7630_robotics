/*!
 * \file raptor_stalk.cpp
 * \brief Image processing framework.
 *
 * This file provides hooks to image subscriptions and sets up the output server for the mcom node.
 *
 * \author omernick
 * \date March 22,2012
 */

#include <raptor/raptor_stalk.h>

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

raptor_stalk::raptor_stalk()
{
  isFirstImg = true;
    //Subscribe to the image producing publisher. (/gscam/image_raw)
    image_subscription = node.subscribe<sensor_msgs::Image>("/gscam/image_raw",1,&raptor_stalk::handle_new_image, this);
    //Advertise your service.
     cv::namedWindow("View",1);
    vector_gen = node.advertiseService("raptor_stalk_srv",&raptor_stalk::get_vector_field,this);
    volume = 1;
}
int HSV_filter(int h, int s, int v, int threshold, int hue, int sat, int val) {
	int FilteredColor[3] = {hue, sat, val}; 
	int diff =  (FilteredColor[0]-h)*(FilteredColor[0]-h) +
				(FilteredColor[1]-s)*(FilteredColor[1]-s);
	
	if(diff < threshold) return abs(diff-threshold); /** If here, it has passed! */
	return 0; /** With 0 this is discarded */
}
double *findBlob(IplImage *img, int hue,int sat,int val,int threshold, double blobLowLimit,double blobHighLimit){
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
				 ((uchar *)(i1->imageData + y*i1->widthStep))[x]=255;
			 }else{
				 ((uchar *)(i1->imageData + y*i1->widthStep))[x]=0;
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
	double *data= new double[blobs.GetNumBlobs()*3+1];
	data[0]=blobs.GetNumBlobs();// Set first data value to total number of blobs
	//cout<<data[0]<<"  ";
	for (int i = 0; i < blobs.GetNumBlobs(); i++ ){ // Get Blob Data 
	    blob = blobs.GetBlob(i);//cycle through each blob
		//data[i*3+1]=blob.area;//blob area
		//cout<<blob.area<<"   ";
		data[i*3+2]= getXCenter(blob); //X of centroid
		data[i*3+3]= getYCenter(blob); //Y of centroid

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

raptor_stalk::~raptor_stalk()
{
  
}

bool raptor_stalk::get_vector_field(raptor::polar_histogram::Request &req, raptor::polar_histogram::Response &res)
{
  double* Out = new double[360];
	for(int i=0;i<360;i++)
		Out[i]=0;
	int H[1]= {67};// H value of all searched colors
	int S[1]= {205};//s val
	int V[1]= {219};//V val
	int I[1]= {1};// Taxis intensity of color (ie towards/away etc)
	int threshold = 5000;// color search threshold
	double blobLowLimit = 500;// blob size limits
	double blobHighLimit = 1000000;
	int viewFieldMin = 159;// view range of robot out of 360
	int viewFieldMax = 199;
	int triangleSlope = 10; // triangle slope for fuzzyfilter
	int depthIntensity[5] = {100,100,100,75,50};
	//

	for(int n=0; n<sizeof(H)/sizeof(H[0]); n++){
		double* blob= findBlob(img,H[n],S[n],V[n],threshold,blobLowLimit,blobHighLimit);
		for(int j=0; j<blob[0];j++){
			int blobX= blob[j*3+2];
			int blobY= blob[j*3+3];
			int maxV=0;
			if (I[n]>0)
				maxV=depthIntensity[(int)blobY/(img->height/(sizeof(depthIntensity)/sizeof(depthIntensity[0])))];// if its positive taxis, then it is more attracted to it farther away it is. 
			else
				maxV=depthIntensity[sizeof(depthIntensity)-1-(int)blobY/(img->height/(sizeof(depthIntensity)/sizeof(depthIntensity[0])))];// if its a negative taxis, then it is more scared of it closer it is
			for(int k=0;k<360;k++){
				int val = maxV-triangleSlope*abs(k-((int)((double)blobX/(double)img->width*(double)(viewFieldMax-viewFieldMin))+viewFieldMin));
				if(val<0) val=0;
				val*=I[n];
				Out[k]+=val;
			}
		}
	}

	for(int i=0;i<360;i++)
	{
	  if(Out[i]>100){Out[i]=100;}
	  if(Out[i]<-100){Out[i]=-100;}
		res.hist[i]=(int)(Out[i]*volume);
	}
  //Finally, fill up the array res.hist
  //res.hist = WHATEVER;
  return true;
}

void raptor_stalk::handle_new_image(const sensor_msgs::Image::ConstPtr& msg)
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
  ros::init(argc,argv,"raptor_stalk");
  
  raptor_stalk ip_core;
  
  ros::spin();
  
  return EXIT_SUCCESS; 
}
