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
#include <blob/blob.h>
#include <blob/BlobResult.h>

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
         cvNamedWindow("imSmooth",1);
	 cvNamedWindow("arg",1);
	 cv::namedWindow("View",1);
    vector_gen = node.advertiseService("raptor_find_dark_srv",&raptor_find_dark::get_vector_field,this);
    volume = 1;
}
int HSV_filter(int h, int s, int v, int threshold, int hue, int sat, int val) {
	int FilteredColor[3] = {hue, sat, val}; 
	int diff =  (FilteredColor[0]-h)*(FilteredColor[0]-h);
	
	if(diff < threshold) return abs(diff-threshold); /** If here, it has passed! */
	return 0; /** With 0 this is discarded */
}
int V_filter(int v, int threshold,int val) {
	int diff =  (val-v)*(val-v);
	
	if(diff < threshold) return abs(diff-threshold); /** If here, it has passed! */
	return 0; /** With 0 this is discarded */
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

double *findShadow(IplImage *img, int hue1,int sat1,int val1,int val2,int threshold, double blobLowLimit,double blobHighLimit){
	// Input HSV value of color blob your seeking, acceptable threshold of that color, and Min and Max blob sizes beeing sought out. 
	//Ouput: pointer to data array, size[#ofblobs*3+1]; Format data=[Number of Blobs, Area1,X of center1, y of center1, Area2,X of center2,y of center2,...,areaN,X of centerN, Y of centerN];
    //cv::imshow("View",cv_ptr->image);
    cv::waitKey(3);
    

	// Image variables
	IplImage* imageSmooth = cvCreateImage( cvGetSize(img),8,3);//Gausian Filtered image
	IplImage* imageHSV = cvCreateImage( cvGetSize(img),8,3); //HSV image
	IplImage* i1 = cvCreateImage( cvGetSize(img),8,1);//filtered out for hsv 
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
	cvSmooth(img,imageSmooth,CV_GAUSSIAN,15,9,0,0);
	//Covert RGB to HSV
	cvCvtColor(imageSmooth,imageHSV,CV_BGR2HSV);
	cvCvtPixToPlane(imageHSV, planeH,planeS,planeV,0);//Extract the 3 color components
	//Filter image for desired Color, output image with only desired color highlighted remaining
	for( int y = 0; y < planeH->height; y++ ){
		unsigned char* h = &CV_IMAGE_ELEM( planeH, unsigned char, y, 0 );
		unsigned char* s = &CV_IMAGE_ELEM( planeS, unsigned char, y, 0 );
		unsigned char* v = &CV_IMAGE_ELEM( planeV, unsigned char, y, 0 );
		for( int x = 0; x < planeH->width*planeH->nChannels; x += planeH->nChannels ){
			 int f= HSV_filter(h[x],s[x],v[x],threshold,hue1,sat1,val1);
			 int g= V_filter(v[x],threshold,val2);
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
		data[i*3+1]=blob.Area();//blob area
		//cout<<blob.area<<"   ";
		data[i*3+2]= getXCenter(blob); //X of centroid
		data[i*3+3]= getYCenter(blob); //Y of centroid

		//debug
		blob.FillBlob(imageSmooth, cvScalar(255, 0, 0)); // This line will give you a visual marker on image for the blob if you want it for testing or something
    }

    cvShowImage("imSmooth",img);
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
	int H= {29};// H val of floor
	int S= {0};//s val
	int V= {0};//V val 
	int V2[2]= {30,230};
	int I[2]= {1, -1};// Taxis intensity of color (ie towards/away etc)
	int threshold = 225;// color search threshold
	double blobLowLimit = 100;// blob size limits
	double blobHighLimit = 1000000;
	int viewFieldMin = 159;// view range of robot out of 360
	int viewFieldMax = 199;
	int triangleSlope = 50; // triangle slope for fuzzyfilter
	int depthIntensity[5] = {100,100,100,75,50};
	
	int blobX;
	int blobY;
	//////////////////////////////////////////
	//double* darkVal = findDark(1,8,img); // Jeffs Number
	//int closeDark = darkVal[7];
	//double* lightVal = findDark(1,1,img);
	//int totalDark = lightVal[0]; // jefff other number
	//////////////////////////////////////////
	double areaScale = .001;
	double maxOut = 0;
	double minOut= 255;
	//cout<<"size:"<<sizeof(V2)/sizeof(V2[0]);
	for(int n=0; n<sizeof(V2)/sizeof(V2[0]); n++){
	  
	  cvShowImage("arg",img);
	  cvWaitKey(3);	
		double* blob= findShadow(img,H,S,V,V2[n],threshold,blobLowLimit,blobHighLimit);
		if(blob[0]>0){
		//	cout<<"blobs#:"<<blob[0]<<"  ";
			for (int j = 0; j < blob[0]; j++){
				
				blobX= blob[j*3+2];
				blobY= blob[j*3+3];
		//	cout<<"   "<< blob[j*3+1];
				int maxV=0;
				if (I[n]>0)
					maxV=blob[j*3+1]*areaScale+depthIntensity[(int)blobY/(img->height/(sizeof(depthIntensity)/sizeof(depthIntensity[0])))];// if its positive taxis, then it is more attracted to it farther away it is. 
				else{
					maxV=blob[j*3+1]*areaScale+depthIntensity[sizeof(depthIntensity)/sizeof(depthIntensity[0])-1-(int)blobY/(img->height/(sizeof(depthIntensity)/sizeof(depthIntensity[0])))];// if its a negative taxis, then it is more scared of it closer it is
				}
				for(int k=0;k<360;k++){
					int val = maxV-triangleSlope*abs(k-((int)((double)blobX/(double)img->width*(double)(viewFieldMax-viewFieldMin))+viewFieldMin));
					if(val>0){
						val*=I[n];
						//cout<<I[n];
						Out[k]+=val;
					}
					if(Out[k]<minOut)
						minOut=Out[k];
					if(Out[k]>maxOut)
						maxOut=Out[k];
				}
			}
		}
	}
	double range = maxOut-minOut;

	
	for(int i=0;i<360;i++){
	  Out[i]+= minOut +range/2;
	  Out[i]*= (200/range);
	  res.hist[i]=(int)(Out[i]*volume);}
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
    //cv::waitKey(3);
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

