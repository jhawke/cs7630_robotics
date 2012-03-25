#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <blob.h>
#include <BlobResult.h>
#include "stdafx.h"
#include <cxcore.h>

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
	double *data;
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
			 }
		}
	}
	//Blob stuff
	blobs = CBlobResult(i1,NULL,0,true);   //Get blobs of image
	blobs.Filter(blobs,B_INCLUDE,CBlobGetArea(),B_INSIDE,blobLowLimit,blobHighLimit);  // Filter Blobs with min and max size
	//Set up data array
	double *data= new double[blobs.GetNumBlobs()*3+1];
	data[0]=blobs.GetNumBlobs();// Set first data value to total number of blobs
	for (int i = 1; i <= blobs.GetNumBlobs(); i+=3 ){ // Get Blob Data 
	    blob = blobs.GetBlob(i);//cycle through each blob
		data[i]=blob.area;//blob area
		data[i+1]= getXCenter(blob); //X of centroid
		data[i+2]= getYCenter(blob); //Y of centroid
		//blob.FillBlob(imageSmooth, cvScalar(255, 0, 0));  This line will give you a visual marker on image for the blob if you want it for testing or something
    }
	return data; //return pointer to data array
}