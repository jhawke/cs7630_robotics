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
#include <boost/array.hpp>
#include <raptor/light_level_srv.h>
#include <std_msgs/Float32.h>

//Put any local defines here.

#define MAX_V 255
#define VALUE_LOW_LIM 30
#define START_THRESH 15
#define OBSTACLE_THRESHOLD 5
#define MIN_BRIGHT 10
#define MIN_SAT 10
#define HIGH_PERCENT .20
#define LOW_PERCENT .20
#define MASK_MIN_BLOB 200
#define LARGE_MIN_BRIGHT 20

using namespace std;
int* xCent;
int* yCent;
int* valCent;

raptor_find_dark::raptor_find_dark()
{
  threshold = START_THRESH;
  //dark_thresh_val = VALUE_LIM;
  isFirstImg = false;
    //Subscribe to the image producing publisher. (/gscam/image_raw)
    dark_thresh = node.subscribe<std_msgs::Int16>("fd_debug_thresh",1,&raptor_find_dark::alter_thrsval,this);
    image_subscription = node.subscribe<sensor_msgs::Image>("/gscam/image_raw",1,&raptor_find_dark::handle_new_image, this);
    near_light_sense = node.advertise<std_msgs::Float32>("DARKNESS_COEFF",2);
    //Advertise your service.
     cv::namedWindow("Dark_Raw",1);
     cv::namedWindow("Dark_Sat",1);
     cv::namedWindow("Dark_Value",1);
     cv::namedWindow("Dark_Hue",1);
     cv::namedWindow("Dark_Triscale",1);
     cv::namedWindow("Dark_ObsDet",1);
     cv::namedWindow("Dark_ObsDetPre",1);
     cv::namedWindow("Dark_Detected",1);
     //img=cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,3); 
    fd_volume = node.subscribe<std_msgs::Float32>("FIND_DARK_GAIN",1,&raptor_find_dark::adjust_gain,this);
    vector_gen = node.advertiseService("raptor_find_dark_srv",&raptor_find_dark::get_vector_field,this);
    light_sense = node.advertiseService("raptor_full_bright_srv",&raptor_find_dark::get_darkness,this);
    volume = 0;
}

void raptor_find_dark::adjust_gain(const std_msgs::Float32::ConstPtr& msg)
{
   volume = msg->data;
   ROS_DEBUG("FD volume updated to %f",volume);
}

int HSV_filter(int h, int s, int v, int m, int threshold_hue,int threshold_low ,int threshold_high,int threshold_mid,int ref_hue, int ref_sat, int ref_val) {
	int FilteredColor[3] = {ref_hue, ref_sat, ref_val}; 
	int diff = abs((h-FilteredColor[0]));//*(h-FilteredColor[0])+(s-FilteredColor[1])*(s-FilteredColor[1]));
	//int diff = abs(FilteredColor[0]-h);
	
	//ROS_INFO("FC[2] is %d",FilteredColor[2]);
	if(m==0||((diff>threshold_hue)&&(!(v<LARGE_MIN_BRIGHT)))){return 0;}//don't care
	if((v<threshold_low)){return 1;}//abs(diff-threshold); /** If here, it has passed! */
	else if(v>(threshold_high)){return 4;}
	else if(v>threshold_mid){return 3;}
	return 2; /** With 3 this is discarded */
}
void raptor_find_dark::alter_thrsval(const std_msgs::Int16::ConstPtr &msg)
{
  //dark_thresh_val = msg->data;
  threshold = msg->data;
}
double findDark(int x, int y,int width,int height, IplImage* img){
	// To convert from ros to iplimage
	//img = bridge.imgmsg_to_cv(image_message, desired_encoding="passthrough");


	/*
	[wxl] s=[(0,0),(0,1),...,(0,l),(1,0),...,(1,l),...,(w,0),...,(w,l)]
     l=image height; w= image width;

	 function outputs the avg brightness of grid of set resolution. The higher the returned value for the grid the brighter the grid location is. 
	*/
		cvSetImageROI(img,cvRect(x,y,width,height));
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

		double dark= cvMean(planeV);
		
		ROS_DEBUG("prerelease");
		cvReleaseImage(&planeH);
		cvReleaseImage(&planeS);
		cvReleaseImage(&planeV);
		cvReleaseImage(&imageHSV);
		ROS_DEBUG("done");
		cvResetImageROI(img);
		//Output
		return dark;

}

double findShadow(IplImage *l_img, int hue,int sat,int val,int threshold, double blobLowLimit,double blobHighLimit){
	// Input HSV value of color blob your seeking, acceptable threshold of that color, and Min and Max blob sizes beeing sought out. 
// Input HSV value of color blob your seeking, acceptable threshold of that color, and Min and Max blob sizes beeing sought out. 
	//Ouput: pointer to data array, size[#ofblobs*3+1]; Format data=[Number of Blobs, Area1,X of center1, y of center1, Area2,X of center2,y of center2,...,areaN,X of centerN, Y of centerN];
    

    

	// Image variables
	IplImage* local_copy = cvCloneImage(l_img);
	IplImage* imageSmooth = cvCreateImage( cvGetSize(l_img),8,3);//Gausian Filtered image
	IplImage* imageSuperSmooth = cvCreateImage( cvGetSize(l_img),8,3);//Gausian Filtered image
	IplImage* imageHSV = cvCreateImage( cvGetSize(l_img),8,3); //HSV image
	IplImage* i1 = cvCreateImage( cvGetSize(l_img),8,1);//desired color filtered image
	IplImage* i2 = cvCreateImage( cvGetSize(l_img),8,1);//desired color filtered image
	IplImage* i_ts = cvCreateImage( cvGetSize(l_img),8,1);//desired color filtered image
	IplImage* planeH = cvCreateImage(cvGetSize(l_img),8,1); //Hue
	IplImage* planeS = cvCreateImage(cvGetSize(l_img),8,1); //Saturation
	IplImage* planeV = cvCreateImage(cvGetSize(l_img),8,1); //Brightness
	IplImage* planeSmoothV = cvCreateImage(cvGetSize(l_img),8,1); //Brightness
	IplImage* imageSmoothHSV = cvCreateImage( cvGetSize(l_img),8,3); //HSV image
	IplImage* obsdetmask = cvCreateImage( cvGetSize(l_img),8,1); //Obs det mask
	IplImage* obsdetmask_dil = cvCreateImage( cvGetSize(l_img),8,1); //Obs det mask
	IplImage* obsdetmask_b = cvCreateImage( cvGetSize(l_img),8,1); //Obs det mask
	IplImage* obsdetmask_bdil = cvCreateImage( cvGetSize(l_img),8,1); //Obs det mask
	//Blob variables
	CBlobResult mask_bls;
	CBlob	mask_bl;
	CBlobResult blobs;
	CBlob blob;
	CBlobResult blobs1;
	CBlob blob1;
	CBlobGetXCenter getXCenter;
	CBlobGetYCenter getYCenter;
	//Output Variable
	//Gausian Filter
	cvSmooth(l_img,imageSmooth,CV_GAUSSIAN,13,13,0,0);
	cvSmooth(l_img,imageSuperSmooth,CV_GAUSSIAN,41,41,0,0);
	//cvShowImage("View2a",imageSmooth);
	
	
	
	//Covert RGB to HSV
	cvCvtColor(imageSmooth,imageHSV,CV_BGR2HSV);
	cvCvtColor(imageSuperSmooth,imageSmoothHSV,CV_BGR2HSV);
	cvCvtPixToPlane(imageSuperSmooth,NULL,NULL,planeSmoothV,0);
	cvCvtPixToPlane(imageHSV, planeH,planeS,planeV,0);//Extract the 3 color components
	cvSetImageROI(imageHSV,cvRect(0,imageHSV->height/3,imageHSV->width,imageHSV->height*2/3));
	IplImage* planeH1 = cvCreateImage(cvGetSize(imageHSV),8,1); //Hue
	IplImage* planeS1 = cvCreateImage(cvGetSize(imageHSV),8,1); //Saturation
	IplImage* planeV1 = cvCreateImage(cvGetSize(imageHSV),8,1); //Brightness
	cvCvtPixToPlane(imageHSV, planeH1,planeS1,planeV1,0);//Extract the 3 color components
	cvResetImageROI(imageHSV);
	
	
	cvShowImage("Dark_Value",planeV);
	cvShowImage("Dark_Sat",planeS);
	cvShowImage("Dark_Hue",planeH);
	cvSet(obsdetmask, cvScalar(0,0,0));
	cv::waitKey(3);
	
	
	int maxDark = 0;
	int minDark = 255;
	int minDarknessValue=0;
	int maxDarknessValue = 0;
	int midDarknessValue = 0;
	//Filter image for desired Color, output image with only desired color highlighted remaining
	for( int y = 0; y < planeH1->height; y++ ){
		unsigned char* h = &CV_IMAGE_ELEM( planeH1, unsigned char, y, 0 );
		unsigned char* s = &CV_IMAGE_ELEM( planeS1, unsigned char, y, 0 );
		unsigned char* v = &CV_IMAGE_ELEM( planeV1, unsigned char, y, 0 );
		for( int x = 0; x < planeH1->width*planeH1->nChannels; x += planeH1->nChannels ){
		  //if(x<5){ROS_INFO("hsv[x] is %d,%d,%d",h[x],v[x],x]);}
			//int f= HSV_filter(h[x],s[x],v[x],threshold,minDarknessValue,maxDarknessValue,midDarknessValue,hue,sat,val);
			int diff = abs((h[x]-hue));
			if(((diff < threshold)||(v[x]<MIN_BRIGHT)||(s[x]<MIN_SAT)))
			{ 
			  ((uchar *)(obsdetmask->imageData + (y+planeH->height-planeH1->height)*obsdetmask->widthStep))[x]=255;
			   if(v[x]<minDark)
			   {minDark=v[x];}
			    if(v[x]>maxDark)
			    {maxDark=v[x];}
			}
			else
			{
			  ((uchar *)(obsdetmask->imageData + (y+planeH->height-planeH1->height)*obsdetmask->widthStep))[x]=0;
			}
		}
	}//debug
	cvDilate(obsdetmask,obsdetmask_dil,NULL,1);
	cvShowImage("Dark_ObsDetPre",obsdetmask_dil);
	mask_bls = CBlobResult(obsdetmask_dil,NULL,0);
	mask_bls.Filter(mask_bls,B_EXCLUDE,CBlobGetArea(),B_LESS,MASK_MIN_BLOB); // Filter Blobs with min and max size
	mask_bls.GetNthBlob( CBlobGetArea(), 0, mask_bl );
	cvSet(obsdetmask_b, cvScalar(0,0,0));
	mask_bl.FillBlob(obsdetmask_b,CV_RGB(255,255,255));
	cvDilate(obsdetmask_b,obsdetmask_bdil,NULL,5);
	cvShowImage("Dark_ObsDet",obsdetmask_bdil);
	cvWaitKey(3);
	minDarknessValue=((maxDark-minDark)*LOW_PERCENT)+minDark;
	if(minDarknessValue<VALUE_LOW_LIM){minDarknessValue=VALUE_LOW_LIM;}
	maxDarknessValue=(maxDark)-((maxDark-minDark)*HIGH_PERCENT);
	midDarknessValue = .5*(minDarknessValue+maxDarknessValue);
	ROS_INFO("minDark = %d, maxDark = %d, minDV = %d, maxDV = %d",minDark,maxDark,minDarknessValue,maxDarknessValue);
	for( int y = 0; y < planeH->height; y++ ){
		unsigned char* h = &CV_IMAGE_ELEM( planeH, unsigned char, y, 0 );
		unsigned char* s = &CV_IMAGE_ELEM( planeS, unsigned char, y, 0 );
		unsigned char* v = &CV_IMAGE_ELEM( planeV, unsigned char, y, 0 );
		unsigned char* m = &CV_IMAGE_ELEM( obsdetmask_bdil, unsigned char, y, 0 );
		for( int x = 0; x < planeH->width*planeH->nChannels; x += planeH->nChannels ){
		  //if(x<5){ROS_INFO("hsv[x] is %d,%d,%d",h[x],v[x],x]);}
			 int f = HSV_filter(h[x],s[x],v[x],m[x],threshold,minDarknessValue,maxDarknessValue,midDarknessValue,hue,sat,val);
			 if((f==0))//Non-floor
			 {
				 ((uchar *)(i1->imageData + y*i1->widthStep))[x]=0;
				 ((uchar *)(i_ts->imageData + y*i_ts->widthStep))[x]=0;
				 ((uchar *)(i2->imageData + y*i2->widthStep))[x]=0;
			 }
			 else if(f==1)	//dark
			 {
				((uchar *)(i1->imageData + y*i1->widthStep))[x]=255;
				((uchar *)(i_ts->imageData + y*i_ts->widthStep))[x]=64;
				((uchar *)(i2->imageData + y*i2->widthStep))[x]=0;
			 }
			 else if(f==2)
			 {
				((uchar *)(i_ts->imageData + y*i_ts->widthStep))[x]=128;
				((uchar *)(i1->imageData + y*i1->widthStep))[x]=0;
				((uchar *)(i2->imageData + y*i2->widthStep))[x]=0;
			 }
			 else if(f==3)
			 {
				((uchar *)(i_ts->imageData + y*i_ts->widthStep))[x]=196;
				((uchar *)(i1->imageData + y*i1->widthStep))[x]=0;
				((uchar *)(i2->imageData + y*i2->widthStep))[x]=0;
			   
			 }
			 else if(f==4)	//bright
			 {
				 ((uchar *)(i_ts->imageData + y*i_ts->widthStep))[x]=255;
				 ((uchar *)(i1->imageData + y*i1->widthStep))[x]=0;
				 ((uchar *)(i2->imageData + y*i2->widthStep))[x]=255;
			 }else{	
			   
			 }
		}
	}

	
	cvShowImage("Dark_Triscale",i_ts);
	cvWaitKey(3);
	//Blob stuff
	blobs = CBlobResult(i1,NULL,0);   //Get blobs of image
	blobs1 =CBlobResult(i2,NULL,0);
	blobs.Filter(blobs,B_INCLUDE,CBlobGetArea(),B_INSIDE,blobLowLimit,blobHighLimit);  // Filter Blobs with min and max size
	blobs1.Filter(blobs1,B_INCLUDE,CBlobGetArea(),B_INSIDE,blobLowLimit,blobHighLimit);
	//Set up data array
	xCent = new int[blobs.GetNumBlobs()+blobs1.GetNumBlobs()];
	yCent = new int[blobs.GetNumBlobs()+blobs1.GetNumBlobs()];
	valCent = new int[blobs.GetNumBlobs()+blobs1.GetNumBlobs()];
	
	ROS_INFO("size:%d  ",blobs.GetNumBlobs()+blobs1.GetNumBlobs());
	double data;
	if(maxDark>190)
	{
	 data=blobs.GetNumBlobs()+blobs1.GetNumBlobs();// Set first data value to total number of blobs
	//cout<<data[0]<<"  ";
	int k=0;
	//ROS_INFO("Blobs gotten.");
	cvWaitKey(3);
	for (int i = 0; i < blobs.GetNumBlobs(); i++ )
	{ // Get Blob Data 
	    blob = blobs.GetBlob(i);//cycle through each blob
		//data[i*3+1]=blob.area;//blob areaEFF
		xCent[i]= getXCenter(blob); //X min
		yCent[i]= getYCenter(blob); //X max	
		valCent[i]= 1; //Y max 
		//debug
		blob.FillBlob(local_copy, cvScalar(255, 0, 0)); // This line will give you a visual marker on image for the blob if you want it for testing or something
      }    
      //ROS_INFO("loop 1 done.");
      cvWaitKey(3);
      for (int i = 0; i < blobs1.GetNumBlobs(); i++ )
      { // Get Blob Data 
	      blob = blobs1.GetBlob(i);//cycle through each blob
		  //data[i*3+1]=blob.area;//blob area
		xCent[blobs.GetNumBlobs()+i]= getXCenter(blob); //X min
		yCent[blobs.GetNumBlobs()+i]= getYCenter(blob); //X max
		valCent[blobs.GetNumBlobs()+i]= -1;

		  //debug
		  blob.FillBlob(local_copy, cvScalar(0, 255, 0)); // This line will give you a visual marker on image for the blob if you want it for testing or something
      }    
	  
	}else{
    //
	data=blobs.GetNumBlobs();// Set first data value to total number of blobs
	//cout<<data[0]<<"  ";
	int k=0;
	//ROS_INFO("Blobs gotten.");
	cvWaitKey(3);
	for (int i = 0; i < blobs.GetNumBlobs(); i++ )
	{ // Get Blob Data 
	    blob = blobs.GetBlob(i);//cycle through each blob
		//data[i*3+1]=blob.area;//blob areaEFF
		xCent[i]= getXCenter(blob); //X min
		yCent[i]= getYCenter(blob); //X max
		valCent[i]= 1; //Y max 
		//debug
		blob.FillBlob(local_copy, cvScalar(255, 0, 0)); // This line will give you a visual marker on image for the blob if you want it for testing or something
      }    
   
   
      }
   cvShowImage("Dark_Detected",local_copy);
    //cv::imshow("View",cv_ptr->image);
    cv::waitKey(3);
    
    
      cvReleaseImage(&local_copy);
      cvReleaseImage(&imageSmooth);
      cvReleaseImage(&imageSuperSmooth);
      cvReleaseImage(&imageHSV);
      cvReleaseImage(&i1);
      cvReleaseImage(&i2);
      cvReleaseImage(&planeSmoothV);
      cvReleaseImage(&imageSmoothHSV);
      cvReleaseImage(&i_ts);
      cvReleaseImage(&planeH);
      cvReleaseImage(&planeS);
      cvReleaseImage(&planeV);
      cvReleaseImage(&planeH1);
      cvReleaseImage(&planeS1);
      cvReleaseImage(&planeV1);
      cvReleaseImage(&obsdetmask);
      cvReleaseImage(&obsdetmask_dil);
      cvReleaseImage(&obsdetmask_b);
      cvReleaseImage(&obsdetmask_bdil);
      return data; //return pointer to data array
}

raptor_find_dark::~raptor_find_dark()
{
  cvDestroyAllWindows();
}

bool raptor_find_dark::get_darkness(raptor::light_level_srv::Request &req, raptor::light_level_srv::Response &res)
{
  double darkVal = findDark(0,0,img->width,img->height,img);
  ROS_INFO("COWS. FI: %f",darkVal);
  res.light_level = (int32_t)darkVal;
  return true;
}

bool raptor_find_dark::get_vector_field(raptor::polar_histogram::Request &req, raptor::polar_histogram::Response &res)
{
  if(isFirstImg)
  {
    ROS_DEBUG("Function 'name' called.");
 
    double* Out = new double[360];
	  for(int i=0;i<360;i++)
		  Out[i]=0;
	  int H= 75;// H val of floor
	  int S= 0;//s val
	  int V= 0;//V val 
	  int V2[1]= {30};
	  //int I[1]= {1};// Taxis intensity of color (ie towards/away etc)
	  int threshold = 22;// color search threshold
	  double blobLowLimit = 100;// blob size limits
	  double blobHighLimit = 1000000;
	  int viewFieldMin = 159;// view range of robot out of 360
	  int viewFieldMax = 199;
	  int triangleSlope = 50; // triangle slope for fuzzyfilter
	  int depthIntensity[5] = {0,0,100,75,50};
	  int intensity;
	  int blobX;
	  int blobY;
	  //////////////////////////////////////////
	  double closeDarkVal = findDark(0,img->height/3,img->width,img->height*2/3,img);
	  std_msgs::Float32 cdv_msg;
	  cdv_msg.data = closeDarkVal;
	  near_light_sense.publish(cdv_msg);
	  //////////////////////////////////////////
	  double areaScale = .001;
	  double maxOut = 0;
	  double minOut= 255;
	  //cout<<"size:"<<sizeof(V2)/sizeof(V2[0]);
	  for(int n=0; n<sizeof(V2)/sizeof(V2[0]); n++){
	    
	    //cvShowImage("arg",img);
	    //cvWaitKey(3);	
		  double blob= findShadow(img,H,S,V,threshold,blobLowLimit,blobHighLimit);
		  //ROS_INFO("passed func");
		  if(blob>0){
		  //	cout<<"blobs#:"<<blob[0]<<"  ";
			  for (int j = 0; j < blob; j++){
				    //ROS_INFO("j: %d ",j);
				  blobX= xCent[j];
				  blobY= yCent[j];
				  intensity = valCent[j];
		  //	cout<<"   "<< blob[j*3+1];
				  int maxV=0;
				  if (intensity>0){
					  int index = (int)blobY/(img->height/(sizeof(depthIntensity)/sizeof(depthIntensity[0])));
					//ROS_INFO("index: %d ", index);
					  maxV=depthIntensity[index];// if its positive taxis, then it is more attracted to it farther away it is. 
				  } else{
					  maxV=depthIntensity[sizeof(depthIntensity)/sizeof(depthIntensity[0])-1-(int)blobY/(img->height/(sizeof(depthIntensity)/sizeof(depthIntensity[0])))];// if its a negative taxis, then it is more scared of it closer it is
	
					//ROS_INFO("else");
				  }
				  for(int k=0;k<360;k++){
					  int val = maxV-triangleSlope*abs(k-((int)((double)blobX/(double)img->width*(double)(viewFieldMax-viewFieldMin))+viewFieldMin));
					  if(val>0){
						  val*=intensity;
						  //cout<<I[n];
						  Out[k]+=val;
					  }
					  if(Out[k]<minOut)
						  minOut=Out[k];
					  if(Out[k]>maxOut)
						  maxOut=Out[k];
				  }
				  //ROS_INFO("loop int done.");
			  }
		  }
	  }
	  double range = maxOut-minOut;

	  
	  for(int i=0;i<360;i++){
	   /* Out[i]+= minOut +range/2;
	    Out[i]*= (200/range);
	 */   res.hist[i]=(int)(Out[i]*volume);
	    
	  }
  }
    else
  {
  //Finally, fill up the array res.hist
    boost::array<int16_t,360> t = {0};
    res.hist = t;
  }
    //Finally, fill up the array res.hist
  //res.hist = WHATEVER;
  return true;
}

void raptor_find_dark::handle_new_image(const sensor_msgs::Image::ConstPtr& msg)
{
  isFirstImg = true; 
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
  donkey_kong=(IplImage)cv_ptr->image; 
  img = &donkey_kong;
  //img= cvGetImage(&(cv_ptr->image),&stub);
  //img = bridge.imgmsg_to_cv(msg, desired_encoding="passthrough");
  if(isFirstImg)
  {
    
    
    cvShowImage("View",img);
    cv::waitKey(3);

  }
  

}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"raptor_find_dark");
  
  raptor_find_dark ip_core;
  
  ros::spin();
  
  return EXIT_SUCCESS; 
}

