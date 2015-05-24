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

#define FPS			10000
#define SHUTTER		56000
#define IMG_WIDTH	512
#define IMG_HEIGHT	48
#define FRAMENUM_MAX 200
#define LUMINANCE_THRESHOLD	60
#define DELAY_UNIT 10

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

char value_current, value_temp, value_prev;
char buffer[256];
char state; // 0 waiting; 1 read
int index_char= 0;
std::string str;

//logging
ofstream logfile;
char filename[20];

// self tuning
int axisY=8;
int axisX[8]={329, 343, 359, 374, 389, 404, 419, 433};

unsigned char max_global, min_global, max_current, min_current, clk_intensity;
unsigned int delay;
bool state_rise, state_fall;

int main(){

	if (idpConf.init()									== PDC_FAILED) return 1;
	if (idpConf.setRecordRate(FPS)						== PDC_FAILED) return 1;
	if (idpConf.setShutterSpeed(SHUTTER)				== PDC_FAILED) return 1;
	if (idpConf.setResolution(IMG_WIDTH, IMG_HEIGHT)	== PDC_FAILED) return 1;

	if (idpConf.setPixelGainMode()						== PDC_FAILED) return 1;
	if (idpConf.setLiveStatus()							== PDC_FAILED) return 1;
	if (idpConf.getRemainBlocks()						== PDC_FAILED) return 1;	

	if(idpConf.setRecordingBlocks(1)					== PDC_FAILED) return 1;
	if (idpConf.getMaxFrames()							== PDC_FAILED) return 1;	
	if(idpConf.getMaxFrames()							== PDC_FAILED) return 1;
	if(idpConf.setRecReady()							== PDC_FAILED) return 1;
	if(idpConf.setTriggerMode(  0, PDC_TRIGGER_MANUAL,
		idpConf.getNumFrames(),
		idpConf.getNumFrames(),
		1)						== PDC_FAILED) return 1;
	if(idpConf.setTriggerMode(  1, PDC_TRIGGER_MANUAL,
		idpConf.getNumFrames(),
		idpConf.getNumFrames(),
		1)						== PDC_FAILED) return 1;
	if(idpConf.startEndlessRecording()					== PDC_FAILED) return 1;

	IDPExpressUtil idpUtil(idpConf.getHeadNumber(0), IMG_WIDTH, IMG_HEIGHT);

	int colortype_cam1 = idpConf.getCameraType(0,0);
	colortype_cam1=1;
	// color mode
	// cam1_height | cam1_color_mode | cam1_output_mode
	// 16 bit | 8 bit | 8 bit
	idpConf.writeRegister(0, ADDR_MOD_CAM1, ((IMG_HEIGHT-1)<<16) | (colortype_cam1<<8) | 0x00);

	if(colortype_cam1 == 1){
		idpConf.writeRegister(0, ADDR_THRE_HSV_CAM1, 0x00ff0000);
	} else {
		// neglect 'slightly darker'
		//idpConf.writeRegister(0, ADDR_THRE_HSV_CAM1, 0x00ff)
		idpConf.writeRegister(0, ADDR_THRE_HSV_CAM1, 0x10ff);
	}

	int framenum;
	imgHead			= Mat(IMG_HEIGHT, IMG_WIDTH, CV_8UC1);
	img = new cv::Mat [FRAMENUM_MAX];
	for(framenum=0; framenum<FRAMENUM_MAX; framenum++){
		img[framenum] = Mat(IMG_HEIGHT, IMG_WIDTH, CV_8UC1);
	}
	
	Timer tm;
	tm.init();	

	unsigned long	nFrameNo, oldFrameNo = 0;
	void			*pBaseAddress;

	logfile.open("logtime-fixedpoints.txt", ios::app);
	logfile << "start " << GetTickCount() << endl;;
	//logfile.close();

	state=0;
	while(1){
		if( idpConf.getLiveFrameAddress(0, &nFrameNo, &pBaseAddress) == PDC_FAILED ) break;
		if(oldFrameNo != 0 && nFrameNo == oldFrameNo) continue;
		oldFrameNo = nFrameNo;

		idpUtil.setBase((UINT8 *)pBaseAddress + 8);

		idpUtil.getHeadData(imgHead.data, 1);
		//idpUtil.getHeadData(img[framenum].data, 1);
		//imshow("Result", img[framenum]);
		//imshow("Result", imgHead);

		// read current frame's character value (iterative)
		/*value_temp= 0;
		for (int n=0; n<8; n++){
		if (img[framenum].data[(axisY*IMG_WIDTH + axisX[n])] > LUMINANCE_THRESHOLD){
		value_temp |= (1<<n);
		}
		}*/
		
		value_temp= 0;
		//if (imgHead.data[(axisY*IMG_WIDTH + axisX[7])] > LUMINANCE_THRESHOLD) {
			if (imgHead.data[(axisY*IMG_WIDTH + axisX[0])] > LUMINANCE_THRESHOLD) value_temp |= 1;
			if (imgHead.data[(axisY*IMG_WIDTH + axisX[1])] > LUMINANCE_THRESHOLD) value_temp |= 2;
			if (imgHead.data[(axisY*IMG_WIDTH + axisX[2])] > LUMINANCE_THRESHOLD) value_temp |= 4;
			if (imgHead.data[(axisY*IMG_WIDTH + axisX[3])] > LUMINANCE_THRESHOLD) value_temp |= 8;
			if (imgHead.data[(axisY*IMG_WIDTH + axisX[4])] > LUMINANCE_THRESHOLD) value_temp |= 16;
			if (imgHead.data[(axisY*IMG_WIDTH + axisX[5])] > LUMINANCE_THRESHOLD) value_temp |= 32;
			if (imgHead.data[(axisY*IMG_WIDTH + axisX[6])] > LUMINANCE_THRESHOLD) value_temp |= 64;
		//}
		//temporal character being read
		// logfile << framenum << " reads " << char(value_temp) << '(' << int(value_temp) << ')' <<  endl;
		if ((value_temp!=value_prev) && value_temp){
			logfile << char(value_temp);
			value_prev= value_temp;
		}
		
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
			if ((max_current-min_current) > (max_global-min_global)){
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
		// shutter speed delay
		idpConf.writeRegister(0, 0xb4, delay, 0);
		

		/*
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
		framenum++;
	}

	//logfile.open("logtime-fixedpoints.txt", ios::app);
	logfile << endl << "stop  " << GetTickCount() << endl;;
	logfile << "------------" << endl;;
	logfile.close();
	
	vector<int> compression_params;
	compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
	compression_params.push_back(9);

	
	/*
	for(framenum=0; framenum<FRAMENUM_MAX; framenum++){
		try {
			sprintf(filename,"%6.6d.jpg",framenum);
			// for (int i=0; i<IMG_WIDTH; i++) img[framenum].data[axisY*IMG_WIDTH + i] = 200;
			// imwrite(filename, img[framenum], compression_params);
			imwrite(filename, img[framenum]);

		}
		catch (runtime_error& ex) {
			fprintf(stderr, "Exception converting image to JPG format: %s\n", ex.what());
			return 1;
		}
		
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
		
	}
	
	*/

	if (idpConf.closeDevice() == PDC_FAILED) return 1;
	destroyAllWindows();
	return 0;
	}