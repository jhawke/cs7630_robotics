// ColorWheelHSV v1.0: Display an interactive HSV color wheel using OpenCV's HSV color converter. by Shervin Emami (shervin.emami@gmail.com), 6th Nov 2009.
// Note that OpenCV's HSV color model is different to the more common HSV color models in graphics software.
// In OpenCV, Hue varies between 0 to 179 (not 0 to 359), and a Saturation of 255 is a bright color, whereas it typically white in other software.
//

#define WIN32_LEAN_AND_MEAN // Exclude rarely-used stuff from Windows headers
//#include <stdio.h>
//#include <tchar.h>

#include <cstdio>	// Used for "printf"
#include <string>	// Used for C++ strings
#include <iostream>	// Used for C++ cout print statements
//#include <cmath>	// Used to calculate square-root for statistics

// Include OpenCV libraries
#include <cv.h>
#include <cvaux.h>
#include <cxcore.h>
#include <highgui.h>

#include "ImageUtils.h"		// easy image cropping, resizing, rotating, etc

using namespace std;

const int HUE_RANGE = 180;	// OpenCV just uses Hues between 0 to 179!
//const int HUE_RANGE = 256;	// OpenCV just uses Hues between 0 to 179!

const int WIDTH = 361;		// Window size
const int HEIGHT = 306;		//		"
const int HUE_HEIGHT = 25;		// thickness of Hue chart
const int WHEEL_TOP = HUE_HEIGHT + 20;		// y position for top of color wheel (= Hue height + gap)
const int WHEEL_BOTTOM = WHEEL_TOP + 255;	// y position for bottom of color wheel
const int TILE_LEFT = 280;	// Position of small tile showing highlighted color
const int TILE_TOP = 140;	//		"
const int TILE_W = 60;		//		"
const int TILE_H = 60;		//		"

char *windowMain = "HSV Color Wheel. Click a color, or press ESC to quit";	// title of the window

int hue = 90;			// This variable is adjusted by the user's trackbar at runtime.
int saturation = 240;	//		"
int brightness = 200;	//		"

int mouseX = -1;	// Position in the window that a user clicked the mouse button.
int mouseY = -1;	//		"


void displayColorWheelHSV(void)
{
	IplImage *imageHSV = cvCreateImage(cvSize(WIDTH, HEIGHT), 8, 3);
	int h = imageHSV->height;			// Pixel height
	int w = imageHSV->width;			// Pixel width
	int rowSize = imageHSV->widthStep;	// Size of row in bytes, including extra padding
	char *imOfs = imageHSV->imageData;	// Pointer to the start of the image HSV pixels.

	// Clear the image to grey (Saturation=0)
	cvSet(imageHSV, cvScalar(0,0,210, 0));

	// Draw the hue chart on the top, at double width.
	for (int y=0; y<HUE_HEIGHT; y++) {
		for (int x=0; x<HUE_RANGE; x++) {
			uchar h = x;		// Hue (0 - 179)
			uchar s = 255;		// max Saturation => most colorful
			uchar v = 255;		// max Value => brightest color
			// Highlight the current value
			if ((h == hue-2 || h == hue+2) && (y < HUE_HEIGHT/2)) {
				s = 0;	// make it white instead of the color
			}
			// Set the HSV pixel components
			*(uchar*)(imOfs + y*rowSize + (x*2+0)*3 + 0) = h;
			*(uchar*)(imOfs + y*rowSize + (x*2+0)*3 + 1) = s;
			*(uchar*)(imOfs + y*rowSize + (x*2+0)*3 + 2) = v;
			*(uchar*)(imOfs + y*rowSize + (x*2+1)*3 + 0) = h;
			*(uchar*)(imOfs + y*rowSize + (x*2+1)*3 + 1) = s;
			*(uchar*)(imOfs + y*rowSize + (x*2+1)*3 + 2) = v;
		}
	}

	// Draw the color wheel: Saturation on the x-axis and Value (brightness) on the y-axis.
	for (int y=0; y<255; y++) {
		for (int x=0; x<255; x++) {
			uchar h = hue;		// Hue (0 - 179)
			uchar s = x;		// Saturation (0 - 255)
			uchar v = (255-y);	// Value (Brightness) (0 - 255)
			// Highlight the current value
			if ((s == saturation-2 || s == saturation-3 || s == saturation+2 || s == saturation+3) && (v == brightness-2 || v == brightness-3 || v == brightness+2 || v == brightness+3)) {
				s = 0;	// make it white instead of the color
				v = 0;	// bright white
			}
			// Set the HSV pixel components
			*(uchar*)(imOfs + (y+WHEEL_TOP)*rowSize + x*3 + 0) = h;
			*(uchar*)(imOfs + (y+WHEEL_TOP)*rowSize + x*3 + 1) = s;
			*(uchar*)(imOfs + (y+WHEEL_TOP)*rowSize + x*3 + 2) = v;
		}
	}

	// Draw a small tile of the highlighted color.
	for (int y=0; y<TILE_H; y++) {
		for (int x=0; x<TILE_W; x++) {
			// Set the HSV pixel components
			*(uchar*)(imOfs + (y+TILE_TOP)*rowSize + (x+TILE_LEFT)*3 + 0) = hue;
			*(uchar*)(imOfs + (y+TILE_TOP)*rowSize + (x+TILE_LEFT)*3 + 1) = saturation;
			*(uchar*)(imOfs + (y+TILE_TOP)*rowSize + (x+TILE_LEFT)*3 + 2) = brightness;
		}
	}

	// Convert the HSV image to RGB (BGR) for displaying
	IplImage *imageRGB = cvCreateImage(cvSize(imageHSV->width, imageHSV->height), 8, 3);
	cvCvtColor(imageHSV, imageRGB, CV_HSV2BGR);	// (note that OpenCV stores RGB images in B,G,R order.

	// Get the highlighted color's RGB (BGR) values
	h = imageRGB->height;			// Pixel height
	w = imageRGB->width;			// Pixel width
	rowSize = imageRGB->widthStep;	// Size of row in bytes, including extra padding
	imOfs = imageRGB->imageData;	// Pointer to the start of the image HSV pixels.
	uchar R = *(uchar*)(imOfs + (255-brightness + WHEEL_TOP)*rowSize + (saturation)*3 + 2);	// Red
	uchar G = *(uchar*)(imOfs + (255-brightness + WHEEL_TOP)*rowSize + (saturation)*3 + 1);	// Green
	uchar B = *(uchar*)(imOfs + (255-brightness + WHEEL_TOP)*rowSize + (saturation)*3 + 0);	// Blue
	cout << "H:" << hue << ", S:" << saturation << ", V:" << brightness << "  ->  R:" << (int)R << ", G:" << (int)G << ", B:" << (int)B << endl;

	// Display the RGB image
	cvShowImage(windowMain, imageRGB);
	cvReleaseImage( &imageRGB );
	cvReleaseImage( &imageHSV );
}

// This function is automatically called whenever the user changes the trackbar value.
void hue_trackbarWasChanged(int)
{
	displayColorWheelHSV();
}

// This function is automatically called whenever the user clicks the mouse in the window.
void mouseEvent( int ievent, int x, int y, int flags, void* param )
{
	// Check if they clicked or dragged a mouse button or not.
	if (flags & CV_EVENT_FLAG_LBUTTON) {
		mouseX = x;
		mouseY = y;
		//cout << mouseX << "," << mouseY << endl;

		// If they clicked on the Hue chart, select the new hue.
		if (mouseY < HUE_HEIGHT) {
			if (mouseX/2 < HUE_RANGE) {	// Make sure its a valid Hue
				hue = mouseX/2;
				cvSetTrackbarPos("Hue", windowMain, hue);	// update the GUI Trackbar
				// Note that "cvSetTrackbarPos()" will implicitly call "displayColorWheelHSV()" for a changed hue.
				//displayColorWheelHSV();
			}
		}
		// If they clicked on the Color wheel, select the new value.
		else if (mouseY >= WHEEL_TOP && mouseY <= WHEEL_BOTTOM) {
			if (mouseX < 256) {	// Make sure its a valid Saturation & Value
				saturation = mouseX;
				brightness = 255 - (mouseY - WHEEL_TOP);
				cvSetTrackbarPos("Saturation", windowMain, saturation);	// update the GUI Trackbar
				cvSetTrackbarPos("Brightness", windowMain, brightness);	// update the GUI Trackbar
				// Note that "cvSetTrackbarPos()" will implicitly call "displayColorWheelHSV()" for saturation or brightness.
				//displayColorWheelHSV();
			}
		}
	}
}


// C++ entry point
int main(int argc, char **argv)
{
	cout << "HSV Color Wheel, by Shervin Emami (shervin.emami@gmail.com), 6th Nov 2009.\n";
	cout << "Click on the top Hue map, or the bottom Color graph to change values.\n";
	cout << "Hi the Escape key in the image window to quit.\n";
	cout << endl << "Red, Green, Blue values of clicked colors:" << endl;

	// Create a GUI window
	cvNamedWindow(windowMain, 1);
	// Allow the user to change the Hue value upto 179, since OpenCV uses Hues upto 180.
	cvCreateTrackbar( "Hue", windowMain, &hue, HUE_RANGE-1, &hue_trackbarWasChanged );
	cvCreateTrackbar( "Saturation", windowMain, &saturation, 255, &hue_trackbarWasChanged );
	cvCreateTrackbar( "Brightness", windowMain, &brightness, 255, &hue_trackbarWasChanged );
	// Allow the user to click on Hue chart to change the hue, or click on the color wheel to see a value.
    cvSetMouseCallback( windowMain, &mouseEvent, 0 );

	displayColorWheelHSV();

//	cvShowImage(windowMain, imageIn);
    cvWaitKey();
    cvDestroyWindow(windowMain);
	
	return 0;
}