#include "IDPExpressUtil.h"
#include "IDPExpressConfig.h"
#include "iocsv.hpp"
//#include "timer.h"
#include "windows.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <ctime>

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
#define SHUTTER		2000
#define IMG_WIDTH	512
#define IMG_HEIGHT	512

#define OPT_SAVE 
#ifdef OPT_SAVE
#define FRAMENUM_MAX 800
#else
#define FRAMENUM_MAX 4000
#endif

using namespace IDPExpress;
//using namespace mytimer;
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

#define THRESHOLD_BACKGROUND 30
#define THRESHOLD_LOGIC 90
bool background, background_prev;

Mat	imgHead, imgResult;
Mat *img;

char buffer[256];
int nframenumber[FRAMENUM_MAX];
int index_char= 0;
std::string str;

//logs
ofstream logintensity;
char filename[20];

//timer
LARGE_INTEGER StartingTime, EndingTime, ElapsedMicroseconds;
LARGE_INTEGER Frequency;

//DLP stuff
#define SEQUENCE_MAX 33
unsigned short int sequence;
unsigned short int \
 bitplane[SEQUENCE_MAX]={7, 6, 4, 4, 3, 2, 1, 0,\
						 0, 0, 0, 0, 0, 0, 0, 0,\
						 0, 0, 0, 0, 0, 0, 0, 0,\
						 0, 0, 0, 0, 0, 0, 0, 0, 0};



int main(){
	QueryPerformanceFrequency(&Frequency); 
	//saving as PNG
	vector<int> compression_params;
	compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
	compression_params.push_back(9);

	if (idpConf.init()									== PDC_FAILED) return 1;
	if (idpConf.setRecordRate(FPS)						== PDC_FAILED) return 1;
	if (idpConf.setShutterSpeed(SHUTTER)				== PDC_FAILED) return 1;
	if (idpConf.setResolution(IMG_WIDTH, IMG_HEIGHT)	== PDC_FAILED) return 1;
	
	//unsigned long errorcode=0;
	//unsigned long modenum, modelist;
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
	imgResult		= Mat(IMG_HEIGHT, IMG_WIDTH, CV_8UC1);
	
#ifdef OPT_SAVE
	img = new cv::Mat [FRAMENUM_MAX];
	for(framenum=0; framenum<FRAMENUM_MAX; framenum++){
		img[framenum] = Mat(IMG_HEIGHT, IMG_WIDTH, CV_8UC1);
	}
#endif

	unsigned long	nFrameNo, oldFrameNo = 0;
	void			*pBaseAddress;
	
	idpConf.writeRegister(0, 0xb4, 0); // just to be safe
	logintensity.open("log.txt");
	logintensity << "start " << GetTickCount() << endl;
	
	
	//while(1){
	for (framenum=0; framenum<FRAMENUM_MAX; framenum++){
	waitframe:
		if (idpConf.getLiveFrameAddress(0, &nFrameNo, &pBaseAddress) == PDC_FAILED ) break;
		if (nFrameNo == oldFrameNo) goto waitframe;
		oldFrameNo = nFrameNo;
		idpUtil.setBase((UINT8 *)pBaseAddress); // head0
		//idpUtil.setBase((UINT8 *)pBaseAddress -8); // head1
		//idpUtil.getHeadData(imgHead.data, 0);
		
		QueryPerformanceCounter(&StartingTime);
		idpUtil.getHeadData(imgHead.data, 0);
		
#ifdef OPT_SAVE
		//for later saving
		imgHead.copyTo(img[framenum]);
#endif
		// THE ROUTINE
		background_prev= background;
		background= TRUE;
		for (int j=0; j<IMG_HEIGHT; j++){
		for (int i=0; i<IMG_WIDTH; i++){
			if (imgHead.data[j*IMG_WIDTH+i] > THRESHOLD_BACKGROUND){
				background= FALSE;
				if (imgHead.data[j*IMG_WIDTH+i] > THRESHOLD_LOGIC){
					if (sequence<SEQUENCE_MAX) imgResult.data[j*IMG_WIDTH+i] |= (1<<(bitplane[sequence]));
					}
				}
			}
		}

	QueryPerformanceCounter(&EndingTime);
	ElapsedMicroseconds.QuadPart = EndingTime.QuadPart - StartingTime.QuadPart;
	ElapsedMicroseconds.QuadPart *= 1000000;
	ElapsedMicroseconds.QuadPart /= Frequency.QuadPart;
	logintensity << ElapsedMicroseconds.QuadPart << endl;
	
	// reset sequence if all-background frame is found
		if (background==TRUE && background_prev==FALSE){
		sequence= 0;
		// render the resulting image
		imshow("result", imgResult);
		}

	// END ROUTINE
	
	} // framenum

	logintensity << "stop  " << GetTickCount() << endl;
	logintensity.close();
	
#ifdef OPT_SAVE
	cout << "start saving" << endl;
	
	for(framenum=0; framenum<FRAMENUM_MAX; framenum++){
		try {
			sprintf(filename,"%4.4d.bmp",framenum);
			// for (int i=0; i<IMG_WIDTH; i++) imgHead.data[axisY[i]*IMG_WIDTH + i] = 200;
			imwrite(filename, imgHead, compression_params);  // PNG
			imwrite(filename, img[framenum]); // another
		}
		catch (runtime_error& ex) {
			fprintf(stderr, "Exception converting image to PNG format: %s\n", ex.what());
			return 1;
		}
	}
#endif

	if (idpConf.closeDevice() == PDC_FAILED) return 1;
	destroyAllWindows();
	return(0);
	}