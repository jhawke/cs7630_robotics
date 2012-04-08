
#include <cxcore.h>
#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <blob.h>
#include <BlobResult.h>
#include <math.h>


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
				// ((uchar *)(i1->imageData + y*i1->widthStep))[x]=0;
			 }
		}
	}//debug
	//cvNamedWindow("i1",1);
	//cvShowImage("i1",i1);
	//cvWaitKey(0);
	//Blob stuff
	blobs = CBlobResult(i1,NULL,0,true);   //Get blobs of image
	blobs.Filter(blobs,B_INCLUDE,CBlobGetArea(),B_INSIDE,blobLowLimit,blobHighLimit);  // Filter Blobs with min and max size
	//Set up data array
	double *data= new double[blobs.GetNumBlobs()*3+1];
	data[0]=blobs.GetNumBlobs();// Set first data value to total number of blobs
	//cout<<data[0]<<"  ";
	for (int i = 0; i < blobs.GetNumBlobs(); i++ ){ // Get Blob Data 
	    blob = blobs.GetBlob(i);//cycle through each blob
		data[i*3+1]=blob.area;//blob area
		//cout<<blob.area<<"   ";
		data[i*3+2]= getXCenter(blob); //X of centroid
		data[i*3+3]= getYCenter(blob); //Y of centroid

		//debug
		//blob.FillBlob(imageSmooth, cvScalar(255, 0, 0)); // This line will give you a visual marker on image for the blob if you want it for testing or something
    }
	return data; //return pointer to data array
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
				s[w*lengthRes+l]=0;
				cvSetImageROI(planeV,cvRect(widthint*w,lengthint*l,widthint,lengthint));
				cvCopy(planeV,grid);
				cvResetImageROI(planeV);
				n=0;
				for( int y = 1; y <= grid->height; y++ ){
				   unsigned char* row = &CV_IMAGE_ELEM( grid, unsigned char, y, 0 );
				   for( int x = 1; x <= grid->width*grid->nChannels; x += grid->nChannels ){
					   s[w*lengthRes+l]+=row[x]; // access the pixel in the first channel 
					   n++;
					   }
				}

				s[w*lengthRes+l]/=n;
				//cout<<w*lengthRes+l<<"_";
				//cout<<s[w*2+1]<<"  ";
			}
		}

		//Output
		return s;

}

double* stalkPrey(IplImage* img){
	double* Out = new double[360];
	for(int i=0;i<360;i++)
		Out[i]=0;
	int H[1]= {67};// H value of all searched colors
	int S[1]= {205};//s val
	int V[1]= {219};//V val
	int I[1]= {1};// Taxis intensity of color (ie towards/away etc)
	int threshold = 100;// color search threshold
	double blobLowLimit = 500;// blob size limits
	double blobHighLimit = 1000000;
	int viewFieldMin = 159;// view range of robot out of 360
	int viewFieldMax = 199;
	int triangleSlope = 1; // triangle slope for fuzzyfilter
	int depthIntensity[5] = {5,10,15,15,15};
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

	return Out;
}

double* findObstacle(IplImage* img){
	cvSmooth(img,img,CV_GAUSSIAN,7,9,0,0);
	//Full HSV color Image
	IplImage* imageHSV = cvCreateImage( cvGetSize(img),8,3); 
	cvCvtColor(img,imageHSV,CV_BGR2HSV);
	//get separate planes
	IplImage* planeH = cvCreateImage(cvGetSize(img),8,1); //Hue
	IplImage* planeS = cvCreateImage(cvGetSize(img),8,1); //Saturation
	IplImage* planeV = cvCreateImage(cvGetSize(img),8,1); //Brightness
	cvCvtPixToPlane(imageHSV, planeH,planeS,planeV,0);//Extract the 3 color components


	//Finding mean RGB value then converting to HSV? Attempt in avoiding Hue mean complexities
	cvSetImageROI(img,cvRect(0,img->height/2,img->width,img->height/2)); // Only want mean of bottom half of image, (floor is background not wall)
	cvSetImageCOI(img,1);  // finnd avg RGB values of image
	double avgR= cvMean(img);
	cvSetImageCOI(img,2);
	double avgB= cvMean(img);
	cvSetImageCOI(img,3);
	double avgG= cvMean(img);
	// convert to HSV
	double avgH,avgS,avgV;
	avgR/=255;
	avgG/=255;
	avgB/=255;
	double minRGB= MIN(avgR,MIN(avgG,avgB));
	double maxRGB= MAX(avgR,MAX(avgG,avgB));
	if(avgR=maxRGB){
		avgH=avgG-avgB;
		avgH/=(maxRGB-minRGB);
	}else if(avgG= maxRGB){
		avgH=2+(avgB-avgR);
		avgH/=(maxRGB-minRGB);
	}
	else{
		avgH=4+(avgR-avgG);
		avgH/=(maxRGB-minRGB);
	}
	avgH*=60;
	if(avgH<0)
		avgH+=360;
	avgV=maxRGB;
	avgS=(maxRGB-minRGB)/maxRGB;
	cvResetImageROI(img);
	//Filter out average
	IplImage* i1 = cvCreateImage( cvGetSize(img),8,1);//desired background filtered image
	double threshold = 15000;
	for( int y = 0; y < planeH->height; y++ ){
		unsigned char* h = &CV_IMAGE_ELEM( planeH, unsigned char, y, 0 );
		unsigned char* s = &CV_IMAGE_ELEM( planeS, unsigned char, y, 0 );
		unsigned char* v = &CV_IMAGE_ELEM( planeV, unsigned char, y, 0 );
		for( int x = 0; x < planeH->width*planeH->nChannels; x += planeH->nChannels ){
			 int f= HSV_filter(h[x],s[x],v[x],threshold,avgH,avgS,avgV);
			 if(f){
				 ((uchar *)(i1->imageData + y*i1->widthStep))[x]=0;
			 }else{
				 ((uchar *)(i1->imageData + y*i1->widthStep))[x]=255;
			 }
		}
	}

	CBlobResult blobs;
	CBlob blob;
    CBlobGetXCenter getXCenter;
	CBlobGetYCenter getYCenter;
	//Blob stuff
	blobs = CBlobResult(i1,NULL,0,true);   //Get blobs of image
	blobs.Filter(blobs,B_INCLUDE,CBlobGetArea(),B_INSIDE,750,1000000);  // Filter Blobs with min and max size
	//Set up data array
	double *data= new double[blobs.GetNumBlobs()*4+1];
	data[0]=blobs.GetNumBlobs();// Set first data value to total number of blobs
	for (int i = 0; i < blobs.GetNumBlobs(); i++ ){ // Get Blob Data 
	    blob = blobs.GetBlob(i);//cycle through each blob
		data[i*4+1]=blob.area;//blob area
		data[i*4+2]= blob.minx; //X min
		data[i*4+3]= blob.maxx; //X max
		data[i*4+4]= getYCenter(blob); //Y of centroid
		//blob.FillBlob(img, cvScalar(255, 0, 0)); // This line will give you a visual marker on image for the blob if you want it for testing or something
    }
	return data;
}

double* detectObstacle(IplImage* img){
	//fuzzifies then defuzzifies Obst. So basiclygets you 0 for not obstacles 1 for obstacle. Closer obstacles and cluttered obstacles will have more of a fringe around them to avoid (ie length extended via fuzzy filter).


	double* Out = new double[360];
	for(int i=0;i<360;i++)
		Out[i]=0;
	int viewFieldMin = 159;// view range of robot out of 360
	int viewFieldMax = 199;
	int triangleSlope = 1; // triangle slope for fuzzyfilter
	int depthIntensity[5] = {15,10,5,0,0};
	//


	double* blob= findObstacle(img);
	for(int j=0; j<blob[0];j++){// make fuzzy
		int blobXmin= blob[j*4+2];
		int blobXmax= blob[j*4+3];
		int blobY= blob[j*4+4];
		int maxV=0;
		maxV=depthIntensity[(int)blobY/(img->height/(sizeof(depthIntensity)/sizeof(depthIntensity[0])))];
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
	return Out;
}


double* detectDark(IplImage* img){
	double* Out = new double[360];
	for(int i=0;i<360;i++)
		Out[i]=0;

	int viewFieldMin = 159;// view range of robot out of 360
	int viewFieldMax = 199;
	double depthIntensity[5] = {.05,.10,.05,0,0};
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
	return Out;
}


int main()
{
	IplImage *img = cvLoadImage("photo3.jpg", 1);
	cvNamedWindow("image",1);
	cvShowImage("image",img);

	double* data = detectDark(img);
	
	for(int i=0;i<360;i++)
		cout<<data[i]<< "  ";



	cvWaitKey(0);
}

