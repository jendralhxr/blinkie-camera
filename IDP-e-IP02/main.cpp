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
#define SHUTTER		4000
#define IMG_WIDTH	512
#define IMG_HEIGHT	512
#define FRAMENUM_MAX 1000
#define LUMINANCE_THRESHOLD	75

#define DELAY_PHASE 4000 // 1=9.9ns
#define DELAY_FREQ 200
#define DELAY_BLOCK 100 // frames
#define DELAY_MAX 50505 // maximum delay unit on 2k fps is 50505

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

Mat	imgHead;
Mat *img;

char buffer[256];
int nframenumber[FRAMENUM_MAX];

// char state; // 0 waiting; 1 read
int index_char= 0;
std::string str;

//logs
ofstream logfile;
ofstream logfile2;
ofstream logintensity;
char filename[20];
int blocknum;
bool dropped;
unsigned int delay;

// self tuning
int axisY=277;
int axisX[8]={181, 214, 248, 281, 313, 346, 379, 411};
bool state[8];

int axisY2=398;
int axisX2[8]={374, 343, 312, 280, 250, 219, 187, 157};
bool state2[8];

char value_current, value_temp, value_prev;
char value_current2, value_temp2, value_prev2;
unsigned char lumi_temp, lumi_max, lumi_min;
unsigned char lumi_temp2, lumi_max2, lumi_min2;

unsigned char max_global, min_global, max_current, min_current, clk_intensity;
bool state_rise, state_fall;

int main(){

	if (idpConf.init()									== PDC_FAILED) return 1;
	if (idpConf.setRecordRate(FPS)						== PDC_FAILED) return 1;
	if (idpConf.setShutterSpeed(SHUTTER)				== PDC_FAILED) return 1;
	if (idpConf.setResolution(IMG_WIDTH, IMG_HEIGHT)	== PDC_FAILED) return 1;
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
	int framenum;
	imgHead			= Mat(IMG_HEIGHT, IMG_WIDTH, CV_8UC1);
	img = new cv::Mat [FRAMENUM_MAX];
	for(framenum=0; framenum<FRAMENUM_MAX; framenum++){
		img[framenum] = Mat(IMG_HEIGHT, IMG_WIDTH, CV_8UC1);
	}
	
	unsigned long	nFrameNo, oldFrameNo = 0;
	void			*pBaseAddress;
	
	logfile.open("log-source1.txt", ios::app);
	logfile << "start " << DELAY_FREQ << " " << GetTickCount() <<endl;
	logfile2.open("log-source2.txt", ios::app);
	logfile2 << "start " << DELAY_FREQ << " " << GetTickCount() << endl;
	logintensity.open("log-intensity.txt", ios::app);
	logintensity << "start " << DELAY_FREQ << " " << GetTickCount() << endl;
	idpConf.writeRegister(0, 0xb4, 0, 0);
	
	// state=0;
	//while(1){
	for (framenum=0; framenum<FRAMENUM_MAX; framenum++){
	waitframe:
		if (idpConf.getLiveFrameAddress(0, &nFrameNo, &pBaseAddress) == PDC_FAILED ) break;
		if (nFrameNo == oldFrameNo) goto waitframe;
	oldFrameNo = nFrameNo;
		//logfile << framenum << "--" << nFrameNo << endl;
		
		idpUtil.setBase((UINT8 *)pBaseAddress -8);
		//idpUtil.getHeadData(imgHead.data, 1);
		idpUtil.getHeadData(img[framenum].data, 0);
		
		// for source1
		// current character being read
		// (even) parity check
		value_temp= 0;
		lumi_max= 0;
		lumi_min= 255;
		
		for (int i=0; i<8; i++){
			state[i] = FALSE;
			lumi_temp= img[framenum].data[(axisY*IMG_WIDTH + axisX[i])];
			if (lumi_temp > LUMINANCE_THRESHOLD) state[i] = TRUE;
			if (lumi_temp > lumi_max) lumi_max= lumi_temp;
			else if (lumi_temp < lumi_min) lumi_min= lumi_temp;
			lumi_temp= img[framenum].data[(axisY*IMG_WIDTH + axisX[i+1])];
			if (lumi_temp > LUMINANCE_THRESHOLD) state[i] = TRUE;
			if (lumi_temp > lumi_max) lumi_max= lumi_temp;
			else if (lumi_temp < lumi_min) lumi_min= lumi_temp;
			lumi_temp= img[framenum].data[(axisY*IMG_WIDTH + axisX[i-1])];
			if (lumi_temp > LUMINANCE_THRESHOLD) state[i] = TRUE;
			if (lumi_temp > lumi_max) lumi_max= lumi_temp;
			else if (lumi_temp < lumi_min) lumi_min= lumi_temp;
			lumi_temp= img[framenum].data[((axisY+1)*IMG_WIDTH + axisX[i])];
			if (lumi_temp > LUMINANCE_THRESHOLD) state[i] = TRUE;
			if (lumi_temp > lumi_max) lumi_max= lumi_temp;
			else if (lumi_temp < lumi_min) lumi_min= lumi_temp;
			lumi_temp= img[framenum].data[((axisY+1)*IMG_WIDTH + axisX[i+1])];
			if (lumi_temp > LUMINANCE_THRESHOLD) state[i] = TRUE;
			if (lumi_temp > lumi_max) lumi_max= lumi_temp;
			else if (lumi_temp < lumi_min) lumi_min= lumi_temp;
			lumi_temp= img[framenum].data[((axisY+1)*IMG_WIDTH + axisX[i-1])];
			if (lumi_temp > LUMINANCE_THRESHOLD) state[i] = TRUE;
			if (lumi_temp > lumi_max) lumi_max= lumi_temp;
			else if (lumi_temp < lumi_min) lumi_min= lumi_temp;
			lumi_temp= img[framenum].data[((axisY-1)*IMG_WIDTH + axisX[i])];
			if (lumi_temp > LUMINANCE_THRESHOLD) state[i] = TRUE;
			if (lumi_temp > lumi_max) lumi_max= lumi_temp;
			else if (lumi_temp < lumi_min) lumi_min= lumi_temp;
			lumi_temp= img[framenum].data[((axisY-1)*IMG_WIDTH + axisX[i+1])];
			if (lumi_temp > LUMINANCE_THRESHOLD) state[i] = TRUE;
			if (lumi_temp > lumi_max) lumi_max= lumi_temp;
			else if (lumi_temp < lumi_min) lumi_min= lumi_temp;
			lumi_temp= img[framenum].data[((axisY-1)*IMG_WIDTH + axisX[i-1])];
			if (lumi_temp > LUMINANCE_THRESHOLD) state[i] = TRUE;
			if (lumi_temp > lumi_max) lumi_max= lumi_temp;
			else if (lumi_temp < lumi_min) lumi_min= lumi_temp;
 		}

		// reconstuct value
		if (state[0]) value_temp |= 1;
		if (state[1]) value_temp |= 2;
		if (state[2]) value_temp |= 4;
		if (state[3]) value_temp |= 8;
		if (state[4]) value_temp |= 16;
		if (state[5]) value_temp |= 32;
		if (state[6]) value_temp |= 64;
		
		if (state[0]^state[1]^state[2]^state[3]^state[4]^state[5]^state[6]^state[7]^TRUE){
			if (value_temp!=value_prev){
				value_prev = value_temp;
				logfile <<value_temp;
			}
		}
		//else logfile << 'X' << blocknum;

		// for source2
		// current character being read
		// (even) parity check
		value_temp2= 0;
		lumi_max2= 0;
		lumi_min2= 255;

		for (int i=0; i<8; i++){
			state2[i] = FALSE;
			lumi_temp2= img[framenum].data[(axisY2*IMG_WIDTH + axisX2[i])];
			if (lumi_temp2 > LUMINANCE_THRESHOLD) state2[i] = TRUE;
			if (lumi_temp2 > lumi_max2) lumi_max2= lumi_temp2;
			else if (lumi_temp2 < lumi_min2) lumi_min2= lumi_temp2;
			lumi_temp2= img[framenum].data[(axisY2*IMG_WIDTH + axisX2[i+1])];
			if (lumi_temp2 > LUMINANCE_THRESHOLD) state2[i] = TRUE;
			if (lumi_temp2 > lumi_max2) lumi_max2= lumi_temp2;
			else if (lumi_temp2 < lumi_min2) lumi_min2= lumi_temp2;
			lumi_temp2= img[framenum].data[(axisY2*IMG_WIDTH + axisX2[i-1])];
			if (lumi_temp2 > LUMINANCE_THRESHOLD) state2[i] = TRUE;
			if (lumi_temp2 > lumi_max2) lumi_max2= lumi_temp2;
			else if (lumi_temp2 < lumi_min2) lumi_min2= lumi_temp2;
			lumi_temp2= img[framenum].data[((axisY2+1)*IMG_WIDTH + axisX2[i])];
			if (lumi_temp2 > LUMINANCE_THRESHOLD) state2[i] = TRUE;
			if (lumi_temp2 > lumi_max2) lumi_max2= lumi_temp2;
			else if (lumi_temp2 < lumi_min2) lumi_min2= lumi_temp2;
			lumi_temp2= img[framenum].data[((axisY2+1)*IMG_WIDTH + axisX2[i+1])];
			if (lumi_temp2 > LUMINANCE_THRESHOLD) state2[i] = TRUE;
			if (lumi_temp2 > lumi_max2) lumi_max2= lumi_temp2;
			else if (lumi_temp2 < lumi_min2) lumi_min2= lumi_temp2;
			lumi_temp2= img[framenum].data[((axisY2+1)*IMG_WIDTH + axisX2[i-1])];
			if (lumi_temp2 > LUMINANCE_THRESHOLD) state2[i] = TRUE;
			if (lumi_temp2 > lumi_max2) lumi_max2= lumi_temp2;
			else if (lumi_temp2 < lumi_min2) lumi_min2= lumi_temp2;
			lumi_temp2= img[framenum].data[((axisY2-1)*IMG_WIDTH + axisX2[i])];
			if (lumi_temp2 > LUMINANCE_THRESHOLD) state2[i] = TRUE;
			if (lumi_temp2 > lumi_max2) lumi_max2= lumi_temp2;
			else if (lumi_temp2 < lumi_min2) lumi_min2= lumi_temp2;
			lumi_temp2= img[framenum].data[((axisY2-1)*IMG_WIDTH + axisX2[i+1])];
			if (lumi_temp2 > LUMINANCE_THRESHOLD) state2[i] = TRUE;
			if (lumi_temp2 > lumi_max2) lumi_max2= lumi_temp2;
			else if (lumi_temp2 < lumi_min2) lumi_min2= lumi_temp2;
			lumi_temp2= img[framenum].data[((axisY2-1)*IMG_WIDTH + axisX2[i-1])];
			if (lumi_temp2 > LUMINANCE_THRESHOLD) state2[i] = TRUE;
			if (lumi_temp2 > lumi_max2) lumi_max2= lumi_temp2;
			else if (lumi_temp2 < lumi_min2) lumi_min2= lumi_temp2;
		}
		
		// reconstruct value
		if (state2[0]) value_temp2 |= 1;
		if (state2[1]) value_temp2 |= 2;
		if (state2[2]) value_temp2 |= 4;
		if (state2[3]) value_temp2 |= 8;
		if (state2[4]) value_temp2 |= 16;
		if (state2[5]) value_temp2 |= 32;
		if (state2[6]) value_temp2 |= 64;
		
		if (state2[0]^state2[1]^state2[2]^state2[3]^state2[4]^state2[5]^state2[6]^state2[7]^TRUE){
			if (value_temp2!=value_prev2){
				value_prev2 = value_temp2;
				logfile2 << value_temp2;
			}
		}
		//else logfile2 << 'X' << blocknum;

		// intensity logging
		logintensity << blocknum << ';' << int(lumi_min) << ';' << int(lumi_max)	<< ';' << int(lumi_min2) << ';' << int(lumi_max2) << endl;

		// variable frequency using frame throttling, with drops
		if (dropped){
			dropped= FALSE;
			logintensity << 'D' << framenum << endl;
		}
		else {
			delay += DELAY_FREQ;
			if (delay >= DELAY_MAX){
				dropped= TRUE;
				delay -= DELAY_MAX;
			}
		}	
		idpConf.writeRegister(0, 0xb4, delay, 0);
				
		// block milestones, introduce phase shift
		if (blocknum== DELAY_BLOCK){ // new block
			std::cout << "mile" <<framenum << endl; 
			blocknum=0;
			logfile << 'B';
			logfile2 << 'B';
			logintensity << 'B' << endl;
		//	idpConf.writeRegister(0, 0xb4, DELAY_PHASE, 0);
		}		
		blocknum++;
		
		/*
		// dummy pll function here
		clk_intensity= imgHead.data[(axisY*IMG_WIDTH + axisX[7])];
		if (clk_intensity > max_current){
			max_current= clk_intensity;
			state_rise= TRUE;
		}
		if (clk_intensity < min_current){
			min_current= clk_intensity;
			state_fall= TRUE;
		}
		if (state_rise && state_fall){
			if ((max_current-min_current) >= (max_global-min_global)){
				max_global= max_current;
				min_global= min_current;
				min_current= 255;
				max_current= 0;
				delay += DELAY_UNIT;
			}
			else {
				delay = delay - (3*DELAY_UNIT);
				if (delay<0) delay=0;
				max_global= 0;
				min_global= 255;
				max_current= 0;
				min_current= 255;
			}
		}
		// adjust freq and phase accordingly
		*/
		
		/*
		// line buffering, 50% duty cycle
		if (state == 0){ // if waiting for char
			state= 1;
			logfile << char(value_temp) << endl;
			// read new char
			// redo buffer from the front on CR, otherwise continue buffer
			if (value_temp==13){
				buffer[index_char]= 0;
				//str.assign (buffer,index_char);
				//logfile << buffer << endl;
				strcpy(buffer,"");
				index_char= 0;
				// read complete line here, flush buffer
			} else if((value_temp != value_current) && (value_temp)){ // new char read
				//logfile << framenum << " newly reads " << char(value_temp) << '(' << int(value_temp) << ')' << endl;
				buffer[index_char] = value_temp;
				value_current = value_temp;
				index_char++;
				
				// cout << framenum << " reads " << value_current << endl;
				
			}
		} else {
			if (value_temp == 0 ) state= 0; // sync zero frame
			else if (value_temp == value_current) state= 0; // repetition frame
			else if(value_temp != value_current){ // read new char, might be unstable here
				buffer[index_char] = value_temp;
				value_current = value_temp;
				index_char++;
				state=0;
			}
			//cout << framenum << " reads zero" << endl;
		}
		*/
	}

	std:cout << "start saving" << endl;

	logfile << endl << "stop  " << GetTickCount() << endl;
	logfile << "------------" << endl;;
	logfile.close();
	logfile2 << endl << "stop  " << GetTickCount() << endl;
	logfile2 << "------------" << endl;;
	logfile2.close();
	logintensity << endl << "stop  " << GetTickCount() << endl;
	logintensity << "------------" << endl;
	logintensity.close();
	
	vector<int> compression_params;
	compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
	compression_params.push_back(9);

	
	for(framenum=0; framenum<FRAMENUM_MAX; framenum++){
		try {
			sprintf(filename,"%4.4d.jpg",framenum);
			// for (int i=0; i<IMG_WIDTH; i++) img[framenum].data[axisY*IMG_WIDTH + i] = 200;
			// imwrite(filename, img[framenum], compression_params);  // PNG
			 imwrite(filename, img[framenum]); // another

		}
		catch (runtime_error& ex) {
			fprintf(stderr, "Exception converting image to JPG format: %s\n", ex.what());
			return 1;
		}
		
		/*
		logfile.open("logdata-fixedpoints.txt", ios::app);
		logfile << int(img[framenum].data[(axisY*IMG_WIDTH + axisX[0])]) << ';' << \
			int(img[framenum].data[axisY*IMG_WIDTH + axisX[1]]) << ';' << \
			int(img[framenum].data[axisY*IMG_WIDTH + axisX[2]]) << ';' << \
			int(img[framenum].data[axisY*IMG_WIDTH + axisX[3]]) << ';' << \
			int(img[framenum].data[axisY*IMG_WIDTH + axisX[4]]) << ';' << \
			int(img[framenum].data[axisY*IMG_WIDTH + axisX[5]]) << ';' << \
			int(img[framenum].data[axisY*IMG_WIDTH + axisX[6]]) << ';' << \
			int(img[framenum].data[axisY*IMG_WIDTH + axisX[7]]) << ';';
		logfile << endl;
		logfile.close();
		*/
	}
	

	
	if (idpConf.closeDevice() == PDC_FAILED) return 1;
	destroyAllWindows();
	return 0;
	}