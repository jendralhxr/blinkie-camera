#include "IDPExpressUtil.h"
#include "IDPExpressConfig.h"
#include "iocsv.hpp"
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

#define FPS			1000
#define SHUTTER		1000
#define IMG_WIDTH	512
#define IMG_HEIGHT	512

//#define OPT_SAVE 
#ifdef OPT_SAVE
#define FRAMENUM_MAX 1000
#else
#define FRAMENUM_MAX 5000
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

#define THRESHOLD_INIT 100
#define THRESHOLD_LOW 20
#define BLANKING_OUT_FRAMES 2
#define GRAY_CODED_PROJECTION

bool background=FALSE, background_prev=FALSE;
bool skipped;

//Mat	imgHead, imgResult, imgHigh, imgThreshold, imgOutput;
Mat *img, *imgOut, *imgTmp;
Mat imagein;

char buffer[256];
unsigned int nframenumber[FRAMENUM_MAX];
char nbitplane[FRAMENUM_MAX],  nblanking[FRAMENUM_MAX];
bool nbackground[FRAMENUM_MAX];
long long int nelapsedmicro[FRAMENUM_MAX];

int index_char= 0;
std::string str;

//logs
ofstream logfile;
char filename[20];

//timer
LARGE_INTEGER StartingTime, EndingTime, ElapsedMicroseconds;
LARGE_INTEGER Frequency;

//DLP stuff
#define BITPLANE_SEQUENCE_MAX 27 // 0 is stitching, -1 is waiting
unsigned char shift[BITPLANE_SEQUENCE_MAX]={0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 5, 6, 6, 6, 7, 7, 7, 0, 0, 0};
//unsigned char shift[BITPLANE_SEQUENCE_MAX]={7, 7, 7, 6, 6, 6, 5, 5, 5, 4, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0};
char bitplane_sequence, blanking_sequence;
int i, j, offset;

// Gray-code 8-bit value lookup table
unsigned char lookup[256]={\
	0, 1, 3, 2, 7, 6, 4, 5, 15, 14, 12, 13, 8, 9, 11, 10, \
	31, 30, 28, 29, 24, 25, 27, 26, 16, 17, 19, 18, 23, 22, 20, 21, \
	63, 62, 60, 61, 56, 57, 59, 58, 48, 49, 51, 50, 55, 54, 52, 53, \
	32, 33, 35, 34, 39, 38, 36, 37, 47, 46, 44, 45, 40, 41, 43, 42, \
	127, 126, 124, 125, 120, 121, 123, 122, 112, 113, 115, 114, 119, 118, 116, 117, \
	96, 97, 99, 98, 103, 102, 100, 101, 111, 110, 108, 109, 104, 105, 107, 106, \
	64, 65, 67, 66, 71, 70, 68, 69, 79, 78, 76, 77, 72, 73, 75, 74, \
	95, 94, 92, 93, 88, 89, 91, 90, 80, 81, 83, 82, 87, 86, 84, 85, \
	255, 254, 252, 253, 248, 249, 251, 250, 240, 241, 243, 242, 247, 246, 244, 245, \
	224, 225, 227, 226, 231, 230, 228, 229, 239, 238, 236, 237, 232, 233, 235, 234, \
	192, 193, 195, 194, 199, 198, 196, 197, 207, 206, 204, 205, 200, 201, 203, 202, \
	223, 222, 220, 221, 216, 217, 219, 218, 208, 209, 211, 210, 215, 214, 212, 213, \
	128, 129, 131, 130, 135, 134, 132, 133, 143, 142, 140, 141, 136, 137, 139, 138, \
	159, 158, 156, 157, 152, 153, 155, 154, 144, 145, 147, 146, 151, 150, 148, 149, \
	191, 190, 188, 189, 184, 185, 187, 186, 176, 177, 179, 178, 183, 182, 180, 181, \
	160, 161, 163, 162, 167, 166, 164, 165, 175, 174, 172, 173, 168, 169, 171, 170 \
	};

unsigned int grayToBinary_t(unsigned int val){
	return(lookup[val]);
}

	// mess start here
	unsigned int framenum;
	Mat imgHead		= Mat(IMG_HEIGHT, IMG_WIDTH, CV_8UC1);
	Mat imgHigh	= Mat(IMG_HEIGHT, IMG_WIDTH, CV_8UC1); // high-level threshold map
	Mat imgThreshold		= Mat(IMG_HEIGHT, IMG_WIDTH, CV_8UC1); // binarization threshold
	Mat imgResult	= Mat(IMG_HEIGHT, IMG_WIDTH, CV_8UC1); // grayscale reconstructed frame
	//Mat imgResult		= Mat(IMG_HEIGHT, IMG_WIDTH, CV_8UC3); // color reconstructed frame
	Mat imgOutput	= Mat(IMG_HEIGHT, IMG_WIDTH, CV_8UC1); // rendered frame

int main(){
	QueryPerformanceFrequency(&Frequency); 
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
	
#ifdef OPT_SAVE
	img = new cv::Mat [FRAMENUM_MAX]; // capture 
	imgOut = new cv::Mat [FRAMENUM_MAX]; // output
	imgTmp = new cv::Mat [FRAMENUM_MAX]; // threshold
	for(framenum=0; framenum<FRAMENUM_MAX; framenum++){
		img[framenum] = Mat(IMG_HEIGHT, IMG_WIDTH, CV_8UC1);
		imgOut[framenum] = Mat(IMG_HEIGHT, IMG_WIDTH, CV_8UC1);
		imgTmp[framenum] = Mat(IMG_HEIGHT, IMG_WIDTH, CV_8UC1);
	}
#endif

#ifndef OPT_SAVE
// video output
	sprintf(filename, "%d.avi", GetTickCount());
	//cv::VideoWriter videofile(filename, -1, 30, cv::Size(512,512), false);
#endif

	unsigned long	nFrameNo, oldFrameNo = 0;
	void			*pBaseAddress;
	
	idpConf.writeRegister(0, 0xb4, 0); // just to be safe
	logfile.open("log.txt");
	logfile << "start " << GetTickCount() << endl;

	// initialize mats
	for (j=0; j<IMG_HEIGHT; j++){
		for (i=0; i<IMG_WIDTH; i++){
			offset= j*IMG_WIDTH+i;
			imgThreshold.data[offset]= THRESHOLD_INIT;
			imgHigh.data[offset]= THRESHOLD_LOW;
			imgResult.data[offset]= 0; // or 0, if that matters
			imgOutput.data[offset]= 0;
		}
	}
	
	for (framenum=0; framenum<FRAMENUM_MAX; framenum++){
	
//	while(1){
		waitframe:
		if (idpConf.getLiveFrameAddress(0, &nFrameNo, &pBaseAddress) == PDC_FAILED ) break;
		if (nFrameNo == oldFrameNo) goto waitframe;
		oldFrameNo = nFrameNo;
		idpUtil.setBase((UINT8 *)pBaseAddress+8); // head0
		//idpUtil.setBase((UINT8 *)pBaseAddress -8); // head1

		QueryPerformanceCounter(&StartingTime);
		background_prev= background;
		background= TRUE;

		idpUtil.getHeadData(imgHead.data, 0);
#ifdef OPT_SAVE
		imgHead.copyTo(img[framenum]); //for later saving
		imgThreshold.copyTo(imgTmp[framenum]);
#endif
		
		//check for one skipped frame
		skipped= FALSE;
		if (framenum!=0 && nFrameNo!=nframenumber[framenum-1]+1 && nframenumber[framenum-1]-191!=nFrameNo){
			skipped= TRUE;
		}

		if (bitplane_sequence>0) {
		if (skipped){
		if (nframenumber[framenum-1] > nFrameNo) bitplane_sequence -= nFrameNo-(nframenumber[framenum-1]-192);
		else bitplane_sequence -= nFrameNo-nframenumber[framenum-1];
		}
		else bitplane_sequence--; // next bitplane sequence
	}
		
		// THE ROUTINE
		// update high threshold map for bitplane, ok
		// update bitplane sequence, ok
		// stitch bit-planes if not background, ok
		offset=IMG_HEIGHT*IMG_WIDTH-1;
bitplane:
	// not background frame i.e. greater than threshold map, stich 1, else keep 0
	//if (imgHead.data[offset] > 120){ 
	if (imgHead.data[offset] > imgThreshold.data[offset] ){ 
		if (background==TRUE){
			background= FALSE;
			if (blanking_sequence>BLANKING_OUT_FRAMES){
				bitplane_sequence= BITPLANE_SEQUENCE_MAX; // new bitplane sequence
				blanking_sequence= 0;
			}
		}
		// upper threshold
		if ((bitplane_sequence==27 || bitplane_sequence==26) && imgHead.data[offset] > imgHigh.data[offset]){
			imgHigh.data[offset]= imgHead.data[offset];
			imgThreshold.data[offset]= (imgHigh.data[offset]>>1);
		}
		// stitching
		if (bitplane_sequence>=0){
		if (skipped && (bitplane_sequence==22 || bitplane_sequence==19 || bitplane_sequence==16 || bitplane_sequence==13 || \
			bitplane_sequence==10 || bitplane_sequence==7 || bitplane_sequence==4 || bitplane_sequence==1)) \
			imgResult.data[offset] |= (1<<shift[bitplane_sequence]);
		else if (bitplane_sequence==23 || bitplane_sequence==20 || bitplane_sequence==17 || bitplane_sequence==14 || \
			bitplane_sequence==11 || bitplane_sequence==8 || bitplane_sequence==5 || bitplane_sequence==2 ) \
			imgResult.data[offset] |= (1<<shift[bitplane_sequence-1]);
		}
		}

	// background frame, blanking sequence
	if ((background==TRUE) && (background_prev==TRUE)){
		if (skipped) {
			if (nframenumber[framenum-1] > nFrameNo) blanking_sequence= nblanking[framenum-1] + (nFrameNo-(nframenumber[framenum-1]-192));
			else blanking_sequence= nblanking[framenum-1] +(nFrameNo-nframenumber[framenum-1]);
		}
		else blanking_sequence= nblanking[framenum-1]+1; 
	}
	nblanking[framenum] = blanking_sequence;
	
	offset--;
	if (offset!=-1) goto bitplane;

// bitplane stitching progress
#ifdef OPT_SAVE
	//imgOut[framenum]= imgResult.clone();
#endif

	// if background-only frame found after sequence ends
	// reconvert Gray-coded projection, ok
	// calculate new threshold map (half of maximum), k
	if (bitplane_sequence==0) bitplane_sequence= -1;
	
	if (blanking_sequence==BLANKING_OUT_FRAMES){
		offset=IMG_HEIGHT*IMG_WIDTH-1;
thresheval:	
#ifdef GRAY_CODED_PROJECTION
		imgOutput.data[offset] = grayToBinary_t(imgResult.data[offset]);
#else
		imgOutput.data[offset] = imgResult.data[offset];
#endif
		imgHigh.data[offset]= THRESHOLD_LOW; // reset threshold buffer
		imgResult.data[offset]= 0; // reset the output buffer
		offset--;
		if (offset!=-1) goto thresheval;
	
#ifndef OPT_SAVE
		//videofile.write(imgOutput);
		sprintf_s(filename,"%d.bmp",GetTickCount());
		imwrite(filename, imgOutput); // another
#endif
	}

#ifdef OPT_SAVE
	imgOutput.copyTo(imgOut[framenum]);
#endif


	QueryPerformanceCounter(&EndingTime);
	ElapsedMicroseconds.QuadPart = EndingTime.QuadPart - StartingTime.QuadPart;
	ElapsedMicroseconds.QuadPart *= 1000000;
	ElapsedMicroseconds.QuadPart /= Frequency.QuadPart;


	nframenumber[framenum] = nFrameNo;
	nbitplane[framenum] = bitplane_sequence;
	nbackground[framenum] = background;
	nelapsedmicro[framenum] = ElapsedMicroseconds.QuadPart;
	nblanking[framenum]= blanking_sequence;

	} // framenum end

	for (framenum=0; framenum<FRAMENUM_MAX; framenum++){
		logfile << framenum << ';' << nframenumber[framenum] << ';' << nbackground[framenum] << ';' << (short int) nblanking[framenum] << ';' \
			<< (short int) nbitplane[framenum] << ';' << nelapsedmicro[framenum] << endl; // add some info here
	}

	//end of log
	logfile << "stop  " << GetTickCount() << endl;
	logfile.close();	

#ifdef OPT_SAVE
	cout << "start saving" << endl;
	
	for(framenum=0; framenum<FRAMENUM_MAX; framenum++){
		try {
			cout << framenum << endl;
			sprintf_s(filename,"c%4.4d.bmp",framenum);
			imwrite(filename, img[framenum]); // another
			sprintf_s(filename,"t%4.4d.bmp",framenum);
			imwrite(filename, imgTmp[framenum]); // another
			sprintf_s(filename,"r%4.4d.bmp",framenum);
			imwrite(filename, imgOut[framenum]); // another
			
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