#include "IDPExpressUtil.h"
#include "IDPExpressConfig.h"
#include "iocsv.hpp"
#include "timer.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>

#ifdef _DEBUG
#pragma comment(lib, "opencv_core247d.lib")
#pragma comment(lib, "opencv_imgproc247d.lib")
#pragma comment(lib, "opencv_highgui247d.lib")
#else
#pragma comment(lib, "opencv_core247.lib")
#pragma comment(lib, "opencv_imgproc247.lib")
#pragma comment(lib, "opencv_highgui247.lib")
#endif
#pragma comment(lib, "PDCLIB.lib")

#define FPS			2000
#define SHUTTER		2800
#define IMG_WIDTH	512
#define IMG_HEIGHT	512
#define RESULT_WIDTH 256
#define RESULT_HEIGHT 256

//#define OPT_SAVE 
#ifdef OPT_SAVE
#define FRAMENUM_MAX 400
#else
#define FRAMENUM_MAX 120000
#endif

using namespace IDPExpress;
using namespace mytimer;
using namespace std;
using namespace cv;

IDPExpressConfig idpConf(1);

// mess these three
#define ADDR_MOD_CAM1		0x00
#define ADDR_THRE_HSV_CAM1	0x04
#define ADDR_THRE_CELL_CAM1	0x08
// don't mess there onwards
#define ADDR_DIFF_CAM1		0x0C
#define ADDR_ROI1_CAM1		0x10
#define ADDR_ROI2_CAM1		0x14

#define ADDR_MOD_CAM2		0x20
#define ADDR_THRE_HSV_CAM2	0x24
#define ADDR_THRE_CELL_CAM2	0x28
#define ADDR_DIFF_CAM2		0x2C
#define ADDR_ROI1_CAM2		0x30
#define ADDR_ROI2_CAM2		0x34

// matrices
Mat	imgHead, imgResult;
Mat *img;

//logs
ofstream logfile;
ofstream logfile2;
ofstream logintensity;
char filename[20];

#define LUMI_INIT 200
unsigned char value_temp[2], value_prev[2], value_prev2[2], value_current[2];
unsigned char lumi_temp[2], lumi_max[2], lumi_min[2], lumi_led[9], lumi_threshold=LUMI_INIT;
// self-tuning position
int axisX[9]={263, 266, 272, 275, 280, 283, 289, 292, 301};
int axisY[9]={227, 222, 227, 221, 227, 220, 226, 221, 220};
bool state[8];
int n; // led iter

// for PLL
#define MARKER_BLOCK 100
#define DELAY_STEP 8000 // from 0 to 50000-ish
#define DELAY_AMOUNT 5000

// image reconst
int x, y, i, j;
unsigned char temp, prev;
char done;

void check_lumi0(){
	if (lumi_temp[0] > lumi_max[0]) lumi_max[0]= lumi_temp[0];
	else if (lumi_temp[0] < lumi_min[0]) lumi_min[0]= lumi_temp[0];
}

int main(){
	if (idpConf.init()									== PDC_FAILED) return 1;
	if (idpConf.setRecordRate(FPS)						== PDC_FAILED) return 1;
	if (idpConf.setShutterSpeed(SHUTTER)				== PDC_FAILED) return 1;
	if (idpConf.setResolution(IMG_WIDTH, IMG_HEIGHT)	== PDC_FAILED) return 1;
	
	unsigned long errorcode=0;
	unsigned long modenum, modelist;

	//PDC_GetTriggerModeList(1, &modenum, &modelist, &errorcode);
	//PDC_SetExternalInMode(0, 0, PDC_EXT_IN_TRIGGER_POSI, &errorcode); 
 	//if(idpConf.setExternalInMode(1)						== PDC_FAILED) return 14; // external sync
	
	if (idpConf.setPixelGainMode()						== PDC_FAILED) return 1;
	if (idpConf.setLiveStatus()							== PDC_FAILED) return 1;
	if (idpConf.getRemainBlocks()						== PDC_FAILED) return 1;	
	if (idpConf.setRecordingBlocks(1)					== PDC_FAILED) return 1;
	if (idpConf.getMaxFrames()							== PDC_FAILED) return 1;
	if (idpConf.setRecReady()							== PDC_FAILED) return 1;
	if (idpConf.setTriggerMode(  0, PDC_TRIGGER_MANUAL,
		idpConf.getNumFrames(),
		idpConf.getNumFrames(),
		1)						== PDC_FAILED) return 1;
	//if (idpConf.setTriggerMode(  1, PDC_TRIGGER_MANUAL,
	//	idpConf.getNumFrames(),
	//	idpConf.getNumFrames(),
	//	1)						== PDC_FAILED) return 1;
	if (idpConf.startEndlessRecording()					== PDC_FAILED) return 1;
	
	IDPExpressUtil idpUtil(idpConf.getHeadNumber(0), IMG_WIDTH, IMG_HEIGHT);
	int colortype_cam1 = idpConf.getCameraType(0,0);
	int colortype_cam2 = idpConf.getCameraType(0,1);

	// color mode
	// cam1_height | cam1_color_mode | cam1_output_mode
	// 16 bit | 8 bit | 8 bit
	idpConf.writeRegister(0, ADDR_MOD_CAM1, ((IMG_HEIGHT-1)<<16) | (colortype_cam1<<8) | 0x00);
	idpConf.writeRegister(0, ADDR_MOD_CAM2, ((IMG_HEIGHT-1)<<16) | (colortype_cam1<<8) | 0x00);

	if(colortype_cam1 == 1) idpConf.writeRegister(0, ADDR_THRE_HSV_CAM1, 0x00ff0000);
	else idpConf.writeRegister(0, ADDR_THRE_HSV_CAM1, 0x00ff);
	if(colortype_cam2 == 1) idpConf.writeRegister(0, ADDR_THRE_HSV_CAM2, 0x00ff0000);
	else idpConf.writeRegister(0, ADDR_THRE_HSV_CAM2, 0x00ff);
	
	// mess start here
	unsigned int framenum;
	imgHead			= Mat(IMG_HEIGHT, IMG_WIDTH, CV_8UC1);
	imgResult	= Mat(RESULT_HEIGHT, RESULT_WIDTH, CV_8UC1);
	// buffered images
#ifdef OPT_SAVE
	img = new cv::Mat [FRAMENUM_MAX];
	for(framenum=0; framenum<FRAMENUM_MAX; framenum++){
		imgHead = Mat(IMG_HEIGHT, IMG_WIDTH, CV_8UC1);
	}
#endif

	unsigned long	nFrameNo, oldFrameNo = 0;
	void			*pBaseAddress;
	
	logintensity.open("log.txt");
	logintensity << "start " << GetTickCount() << endl;
	idpConf.writeRegister(0, 0xb4, 0); // just to be safe
	idpConf.writeRegister(0, 0xb4, 0);

	// saving
	vector < int   >compression_params;
	    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
	    compression_params.push_back(9);

	//while(1){
	for (framenum=0; framenum<FRAMENUM_MAX; framenum++){
	waitframe:
		if (idpConf.getLiveFrameAddress(0, &nFrameNo, &pBaseAddress) == PDC_FAILED ) break;
		if (nFrameNo == oldFrameNo) goto waitframe;
		oldFrameNo = nFrameNo;
		//cout << framenum << "--" << nFrameNo << endl;
		//idpUtil.setBase((UINT8 *)pBaseAddress +8); // head0
		idpUtil.setBase((UINT8 *)pBaseAddress -8); // head1
		//idpUtil.getHeadData(imgHead.data, 0); // head0
		idpUtil.getHeadData(imgHead.data, 0); // head1
		
#ifdef OPT_SAVE
		//for later saving
		imgHead.copyTo(img[framenum]);
#endif
		// for source1
		// current character being read
		value_temp[0]= 0;
		lumi_max[0]= 0;
		lumi_min[0]= 255;
		
		for (n=0; n<9; n++){
			state[n] = FALSE;
			lumi_led[n]= 0;
			// selection isn't else'd so execution is rather constant
			lumi_temp[0]= imgHead.data[(axisY[n]*IMG_WIDTH + axisX[n])];
			if (lumi_temp[0] > lumi_threshold) state[n] = TRUE;
			check_lumi0();
			if (lumi_temp[0] > lumi_led[n]) lumi_led[n] = lumi_temp[0];
			lumi_temp[0]= imgHead.data[(axisY[n]*IMG_WIDTH + axisX[n]+1)];
			if (lumi_temp[0] > lumi_threshold) state[n] = TRUE;
			if (lumi_temp[0] > lumi_led[n]) lumi_led[n] = lumi_temp[0];
			check_lumi0();
			lumi_temp[0]= imgHead.data[(axisY[n]*IMG_WIDTH + axisX[n]-1)];
			if (lumi_temp[0] > lumi_threshold) state[n] = TRUE;
			if (lumi_temp[0] > lumi_led[n]) lumi_led[n] = lumi_temp[0];
			check_lumi0();
			lumi_temp[0]= imgHead.data[((axisY[n]+1)*IMG_WIDTH + axisX[n])];
			if (lumi_temp[0] > lumi_threshold) state[n] = TRUE;
			if (lumi_temp[0] > lumi_led[n]) lumi_led[n] = lumi_temp[0];
			check_lumi0();
			lumi_temp[0]= imgHead.data[((axisY[n]+1)*IMG_WIDTH + axisX[n]+1)];
			if (lumi_temp[0] > lumi_threshold) state[n] = TRUE;
			if (lumi_temp[0] > lumi_led[n]) lumi_led[n] = lumi_temp[0];
			check_lumi0();
			lumi_temp[0]= imgHead.data[((axisY[n]+1)*IMG_WIDTH + axisX[n]-1)];
			if (lumi_temp[0] > lumi_threshold) state[n] = TRUE;
			if (lumi_temp[0] > lumi_led[n]) lumi_led[n] = lumi_temp[0];
			check_lumi0();
			lumi_temp[0]= imgHead.data[((axisY[n]-1)*IMG_WIDTH + axisX[n])];
			if (lumi_temp[0] > lumi_threshold) state[n] = TRUE;
			if (lumi_temp[0] > lumi_led[n]) lumi_led[n] = lumi_temp[0];
			check_lumi0();
			lumi_temp[0]= imgHead.data[((axisY[n]-1)*IMG_WIDTH + axisX[n]+1)];
			if (lumi_temp[0] > lumi_threshold) state[n] = TRUE;
			if (lumi_temp[0] > lumi_led[n]) lumi_led[n] = lumi_temp[0];
			check_lumi0();
			lumi_temp[0]= imgHead.data[((axisY[n]-1)*IMG_WIDTH + axisX[n]-1)];
			if (lumi_temp[0] > lumi_threshold) state[n] = TRUE;
			if (lumi_temp[0] > lumi_led[n]) lumi_led[n] = lumi_temp[0];
			check_lumi0();
		}

		// reconstuct value
		if (state[0]) value_temp[0] |= 1;
		if (state[1]) value_temp[0] |= 2;
		if (state[2]) value_temp[0] |= 4;
		if (state[3]) value_temp[0] |= 8;
		if (state[4]) value_temp[0] |= 16;
		if (state[5]) value_temp[0] |= 32;
		if (state[6]) value_temp[0] |= 64;
		if (state[7]) value_temp[0] |= 128;
				
		value_prev[0]= value_current[0];
		value_current[0] = value_temp[0];
		
		// image reconst
if (value_temp[0] == 0x3) {
    if (value_prev[0] != 0x3) {
	i = 0;
	j = 0;
	cout << "image start" << endl;
	if (done) {
	    // save image
	    cout << "saving" << endl;
	    sprintf(filename,"%d.png",GetTickCount());
		imwrite(filename, imgResult, compression_params);	// PNG
		// imshow( "result", imgResult);
	} else
	    done = 1;
    }
} else if (value_temp[0] == 0xc) {
    if (value_prev[0] != 0xc) {
	j++;
	i = 0;
	cout << "row: " << j << endl;
    }
} else if (i < RESULT_WIDTH) {
    imgResult.data[j * RESULT_WIDTH + i] = value_temp[0];
    i++;
}


		logintensity << oldFrameNo << ';' << short int(lumi_min[0]) << ';' << short int(lumi_max[0])\
			<< ';' << short int (lumi_led[0]) << ';' << short int (lumi_led[1]) << ';' << short int (lumi_led[2]) \
			<< ';' << short int (lumi_led[3]) << ';' << short int (lumi_led[4]) << ';' << short int (lumi_led[5]) \
			<< ';' << short int (lumi_led[6]) << ';' << short int (lumi_led[7]) << ';' << short int (lumi_led[8]) \
			<< ';' << unsigned short int (value_temp[0]) << endl;
		
		// pll iseng
	//	if (lumi_led[8] > 40) idpConf.writeRegister(0,0xb4,5000);
	//	else idpConf.writeRegister(0,0xb4,0);

	} // framenum

	logintensity << "stop  " << GetTickCount() << endl;
	logintensity << "------------" << endl;
	logintensity.close();
	
	//cout << "saving" << endl;
	//imwrite("final.png", imgResult, compression_params);	// PNG
	    
#ifdef OPT_SAVE
	cout << "start saving" << endl;
	vector<int> compression_params;
	compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
	compression_params.push_back(9);

	for(framenum=0; framenum<FRAMENUM_MAX; framenum++){
		try {
			sprintf(filename,"%4.4d.jpg",framenum);
			// for (int i=0; i<IMG_WIDTH; i++) imgHead.data[axisY[i]*IMG_WIDTH + i] = 200;
			// imwrite(filename, imgHead, compression_params);  // PNG
			imwrite(filename, img[framenum]); // another
		}
		catch (runtime_error& ex) {
			fprintf(stderr, "Exception converting image to JPG format: %s\n", ex.what());
			return 1;
		}
	}
#endif

	if (idpConf.closeDevice() == PDC_FAILED) return 1;
	destroyAllWindows();
	return(0);
	}