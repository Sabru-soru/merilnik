#include <stdio.h>
#include <string>
#include <opencv\cv.h>
#include <opencv\highgui.h>
#include "function.h"

using namespace std;
using namespace cv;


int DetectProfile(Mat &source_image, float* profile, int filter_size){

	//declare derivative
	Mat derivative;
	//declare and calculate sigma
	double sigma = 0.3*((filter_size-1)*0.5 - 1) + 0.8;
	//filter image using GaussianBlur(), Size()
	
	//calculate derivatione using Sobel()
	
	//declare pointer to derivative
	float *pDrv;
	//declare pointer to next element of derivative
	float *pDrv_next;
	//declare pointer to current intensity
	float *pInty;
	//init maximal intensity
	float max_inty;

	//"for each row"
	for(int v = 0; v <		;v++){
		//set "pointer to derivative" to first elemetnt in curr row
		pDrv = 
		//set "pointer to next element derivative" to second elemetnt in curr row
		pDrv_next = 
		//set "pointer to curr intensity" to first element in curr row
		pInty = 
		//set maimal intensity to 0
		max_inty = 
		//for each column/element in the row
		for(int u = 0; u<		-1;u++, pDrv++, pDrv_next++, pInty++){
			//if "pointer to derivative" is larger or equal than 0 AND "pointer to next derivative" is smaller than 0
			if(		){
				//if curr. intensity is larger than maximal intensity
				if(		){
					//store location of the "shoulder" to apropriate place
					profile[ ] = 
					//set maximal intensity to curr intensity
					max_inty =
				}
			}
		}
	}

	return 0;
}

void ReconstructSurface(cv::Mat *profiles, TransformationParameters *TP ,Mat *x ,Mat *y,Mat *z){
	 //length of profile
	long LofPrf = 
	//number of profiles
	long NofPrf = 

	//declare U_N, V_N
	float U_N,V_N;
	//declare pointers for x,y,z
	float *pX,*pY,*pZ;
	//declare pointer for profile
	float* pProf;

	//for each profile
	for(int v= 0;v<		;v++){
		//set pointers to the begining of coresponding row of x,y and z
		pZ = 
		pY = 
		pX = 
		//set pointers to the begining of coresponding row of profiles
		pProf = 

		//for each element in profile
		for(int u= 0;u<		;u++,pX++,pY++,pZ++,pProf++){
			//calculate U_N	(see eq. 1)
			U_N = 
			//calculate V_N (see eq. 2)
			V_N = 
			//calculate Z, use pointer (see eq. 3)
			*pZ = 
			//calculate Y, use pointer (see eq. 4)
			*pY = 
			//calculate X, use pointer
			*pX = 
		}
	}
}

int main(){
	//set transformation parametres
	TransformationParameters TP = TransformationParameters(,	//alpha; triangulation angle
		,		//Py; distance between projector and camera in y direction
		,		//focal length of the camera lens
		,	//center of the image in U direction
		,	//center of the image in V direction
		,	//size of sensor element in U direction
		,	//size of sensor element in V direction
		);	//step between two consequentional images

	//declare source image
	Mat img_original;
	//declare and set path name
	string path = 
	//declare and set number of files
	int NofImages = 
	int startImageIndex = 
	//declare and allocate memory for matrix of profiles (length of profile is 640)
	Mat profiles = 
	//declare x,y,z
	Mat x,y,z;

	//loop
	for(int i = 0; i < ; i++){
		printf("%d\n",i);
		//load GRAYSCALE image
		= imread(path + to_string(static_cast<long long>(		)) + ".bmp", CV_LOAD_IMAGE_GRAYSCALE);
		//transpose image
		
		//convert image to apropriate datatype
		
		//DetectLine
		DetectProfile(img_original,		,	);
	}
	
	//allocate memory for x,y,z
	z = 
	y = 
	x = 

	//ReconstructSurface
	ReconstructSurface(	);
	//Save .VRML file
	SaveMXYZfile(	,&x,&y,&z);

	return 0;
}