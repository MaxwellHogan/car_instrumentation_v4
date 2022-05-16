#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

// includes for threading 
#include <chrono>
#include <thread>
#include <memory>
#include <mutex>
#include <condition_variable>

// ouster lidar stuff
#include "ouster/build.h"
#include "ouster/client.h"
#include "ouster/lidar_scan.h"
#include "ouster/types.h"

// includes for realsense and opencv
#include <librealsense2/rs.hpp> 
#include "opencv2/opencv.hpp"  

// magewell frameGrabber includes
#include "MWFOURCC.h"
#include "LibMWCapture/MWCapture.h"

// xsens imu includes 
#include "xsensdeviceapi.h"
#include "xstypes/xstime.h"
#include "xscommon/xsens_mutex.h"

// for reading the config.json
#include <json/json.h>

using namespace ouster;

const size_t UDP_BUF_SIZE = 65536;

void FATAL(const char* msg) {
    std::cerr << msg << std::endl;
    std::exit(EXIT_FAILURE);
}

// object to store json config file data in,
// this file is rami_config and is used to set
// up the sensors without needing to recompile 
// the code.
Json::Value root;

// function defines 

// from realsense example code - convert between 
static cv::Mat frame_to_mat(const rs2::frame& f);
void wait_for_timeout(bool* p_flag, float timeout_seconds);

// class defines

/******************************* DepthCamera class def *******************************/
class DepthCamera
{
private:

    // destination where to save the output files 
    std::string file_base;

    // Create librealsense context for managing devices
    rs2::context ctx;

    // librealsense frames coming from camera
    rs2::pipeline pipe;

    // camera config 
    rs2::config cfg;

    // Keep track of frame number
    uint last_frame_number = 0;

    // ready flag - set after poll thread is initialised
    bool ready = false;
    bool wake = true; 

    // busy flag
    bool busy = false;
    bool capture_trigger = false; 

    cv::Mat color_frame;
    cv::Mat left_ir_frame;
    cv::Mat right_ir_frame;

    // threads running in the back 
    void poll_thread();
    void save_thread();

public:
    DepthCamera(Json::Value root);

    // call this fucntion to start the polling thread
    std::thread spawn_poll_thread();

    // call this fucntion to start the saving thread
    std::thread spawn_save_thread();
    
    // call this function to trigger the save thread
    void set_trigger();
    // returns the current trigger state 
    bool get_trigger();

    // run this after threads are spawned to make sure setup is complete 
    void wait_for_ready();

    // release device 
    void release_device(); 

};

/******************************* DepthCamera class def *******************************/
class ZedCamera
{
private:

    // destination where to save the output files 
    std::string file_base;

    // Keep track of frame number
    uint last_frame_number = 0;

    // ready flag - set after poll thread is initialised
    bool ready = false;

    // busy flag
    bool capture_trigger = false; 

    // required for zed
    cv::VideoCapture cap;
    cv::Size2i image_size;

    cv::Mat frame;

    // threads running in the back 
    void save_thread();

public:
    ZedCamera(Json::Value root);

    // call this fucntion to start the saving thread
    std::thread spawn_save_thread();
    
    // call this function to trigger the save thread
    void set_trigger();
    // returns the current trigger state 
    bool get_trigger();

    // run this after threads are spawned to make sure setup is complete 
    void wait_for_ready();

};

/********************************** Lidar class def **********************************/

class Lidar
{
private:

    // handle for the lidar client
    // user to control the lidar and retrieve scans 
    std::shared_ptr<ouster::sensor::client> handle;

    // sample period - how often to save cloud to drive 
    float samplePeriod;

    // contains metadata about the sensor that is required to build
    // other structures
    sensor::sensor_info info;

    // scan dimensions 
    size_t scan_w;
    size_t scan_h;

    // destination where to save the output files 
    std::string file_base;

    // ready flag - set after poll thread is initialised
    bool ready = false;
    bool wake = true;

    // flag to check whether the buffers are busy
    bool busy = false;

    // flag to set whether point cloud has saved to drive  
    std::condition_variable capture_cv;
    std::mutex capture_mutex;
    bool capture_trigger = false; 
    
    // set up buffers to store scans in
    // for reading to 
    std::unique_ptr<ouster::LidarScan> ls_poll; 
    // for storing in  
    std::unique_ptr<ouster::LidarScan> ls_swap;
    // for writing to disk 
    std::unique_ptr<ouster::LidarScan> ls_save;

    // store file count 
    uint file_ind = 0;

    // look up table to calc point cloud 
    ouster::XYZLut lut;
    ouster::LidarScan::Points cloud;

    void poll_thread();
    void save_thread();
    
public:
    Lidar(Json::Value root);
    
    // call this fucntion to start the polling thread
    std::thread spawn_poll_thread();

    // call this fucntion to start the saving thread
    std::thread spawn_save_thread();
    
    // call this function to trigger the save thread
    void set_trigger();
    // returns the current trigger state 
    bool get_trigger();

    // run this after threads are spawned to make sure setup is complete 
    void wait_for_ready();

    void release_device();

};

/********************************** Xsens class def **********************************/

class CallbackHandler : public XsCallback
{
public:
	CallbackHandler(size_t maxBufferSize = 5)
		: m_maxNumberOfPacketsInBuffer(maxBufferSize)
		, m_numberOfPacketsInBuffer(0)
	{
	}

	virtual ~CallbackHandler() throw()
	{
	}

	bool packetAvailable() const
	{
		xsens::Lock locky(&m_mutex);
		return m_numberOfPacketsInBuffer > 0;
	}

	XsDataPacket getNextPacket()
	{
		assert(packetAvailable());
		xsens::Lock locky(&m_mutex);
		XsDataPacket oldestPacket(m_packetBuffer.front());
		m_packetBuffer.pop_front();
		--m_numberOfPacketsInBuffer;
		return oldestPacket;
	}

protected:
	virtual void onLiveDataAvailable(XsDevice*, const XsDataPacket* packet)
	{
		xsens::Lock locky(&m_mutex);
		assert(packet != nullptr);
		while (m_numberOfPacketsInBuffer >= m_maxNumberOfPacketsInBuffer)
			(void)getNextPacket();

		m_packetBuffer.push_back(*packet);
		++m_numberOfPacketsInBuffer;
		assert(m_numberOfPacketsInBuffer <= m_maxNumberOfPacketsInBuffer);
	}
private:
	mutable xsens::Mutex m_mutex;

	size_t m_maxNumberOfPacketsInBuffer;
	size_t m_numberOfPacketsInBuffer;
	std::list<XsDataPacket> m_packetBuffer;
};

class Xsense_sensor
{
private:
    // destination where to save the output files 
    std::string file_base;

    // Keep track of frame number
    uint last_frame_number = 0;

    // ready flag - set after poll thread is initialised
    bool ready = false;

    // busy flag
    bool busy = false;
    bool capture_trigger = false; 
    bool wake = true; 

    // objects interacting with the library 
    XsControl* control;
    XsDevice* device;
	XsPortInfo mtPort;

    // callback handler from example code 
    CallbackHandler callback;

    // thread running in the back 
    void save_thread();

public:
    Xsense_sensor(Json::Value root);

    // call this fucntion to start the saving thread
    std::thread spawn_save_thread();
    
    // call this function to trigger the save thread
    void set_trigger();
    // returns the current trigger state 
    bool get_trigger();

    // run this after threads are spawned to make sure setup is complete 
    void wait_for_ready();

    // release device 
    void release_device(); 

};

/******************************* FrameGrabber class def ******************************/
/*
    the thermal cameras use the magewell framegrabber, this library works differently 
    to the other two sensors. The SDK allows us to implement a callback function that 
    will be triggered every time a new frame is available. In this fuction we convert 
    to opencv mat and then store in a buffer. 
*/

// Custom structure to be passed to MW callback function
typedef struct
{
   	cv::Mat frame;
    cv::Rect roi;

    // busy is handled here instead
    bool busy = false;

    int width;
    int height;

    int capture_height;
    int capture_width;

} MW_FRAME;

// declare class
class FrameGrabberInstance
{
private:
    MW_FRAME MW_frame;
    HANDLE hVideo;
    HCHANNEL hChannel;

    uint last_frame_number = 0;

    // destination where to save the output files 
    std::string file_base;

    // ready flag - set after poll thread is initialised
    bool ready = false;
    bool wake = true;

    // busy flag
    bool capture_trigger = false; 

    // thread running in the back
    void save_thread();

public:
    FrameGrabberInstance(Json::Value root, uint8_t t_nIndex = 0);
    
    // gets the most recent frame and place in buffer
    uint8_t getframe(cv::Mat* pbuffer);
    
    // call this fucntion to start the saving thread
    std::thread spawn_save_thread();

    // call this function to trigger the save thread
    void set_trigger();
    // returns the current trigger state 
    bool get_trigger();

    // run this after threads are spawned to make sure setup is complete 
    void wait_for_ready();

    // release device - set wake to false
    void release_device();

};

/********************************** Main Loop Start **********************************/
int main(int argc, char* argv[]){

    std::cout << "Hello Bronte." << std::endl;
    
    std::ifstream config_doc("rami_config.json", std::ifstream::binary);
    config_doc >> root;

    float samplePeriod = root["SamplePeriod"].asFloat();

    // define lidar object and start its threads.
    Lidar lidar_obj = Lidar(root);
    std::thread lidar_poll_thread = lidar_obj.spawn_poll_thread();
    std::thread lidar_save_thread = lidar_obj.spawn_save_thread();
    lidar_obj.wait_for_ready();
    std::cout << "INFO: Lidar Ready\n";

    // define frame grabber object and start its threads
    // Initializes MWCapture
    MWCaptureInitInstance();

    // look up how many framegrabbers are meant to be connected 
    int MW_count = root["FrameGrabber_config"]["count"].asInt();

    // // // check that the correct amount of cameras are plugged in
    uint8_t nCount = MWGetChannelCount();
    if(nCount != MW_count){
        std::cerr << "ERROR!" << nCount << "grabbers detected\n";
        return 0;
    } // if none found return 

    // the software is written to work with exactly two framegabbers 
    FrameGrabberInstance frameGrabber_0(root, 0);
    std::thread frameGrabber_0_save_thread = frameGrabber_0.spawn_save_thread();
    frameGrabber_0.wait_for_ready();
    std::cout << "INFO: frameGrabber_0 Ready\n";

    FrameGrabberInstance frameGrabber_1(root, 1);
    std::thread frameGrabber_1_save_thread = frameGrabber_1.spawn_save_thread();
    frameGrabber_1.wait_for_ready();
    std::cout << "INFO: frameGrabber_1 Ready\n";

    // define the ZED camera instance - this is opencv based
    // ZedCamera zedCamera(root);
    // std::thread zed_save_thread = zedCamera.spawn_save_thread();
    // zedCamera.wait_for_ready(); 
    // std::cout << "INFO: Zed Ready\n";

    // define depth camera object and start its threads
    DepthCamera depthCamera_obj = DepthCamera(root);
    std::thread depthCamera_poll_thread = depthCamera_obj.spawn_poll_thread();
    std::thread depthCamera_save_thread = depthCamera_obj.spawn_save_thread();
    depthCamera_obj.wait_for_ready();
    std::cout << "INFO: depthCamera Ready\n";

    // define xsens imu object and start its threads
    Xsense_sensor xsense_sensor(root);
    std::thread xsense_sensor_save_thread = xsense_sensor.spawn_save_thread();

    // main loop
    bool wake = true;

    // define start of new frame 
    auto startNewFrame = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> lastFrame;
    std::cout << "INFO: Triggering has started\n";

    std::thread trigger_thread([&](){
    while (wake)
    {
    // update new frame start time
        lastFrame = std::chrono::high_resolution_clock::now() - startNewFrame;

        if(lastFrame.count() >= samplePeriod){

            startNewFrame = std::chrono::high_resolution_clock::now();

            // set triggers to begin capture 
            lidar_obj.set_trigger();
            depthCamera_obj.set_trigger();
            frameGrabber_0.set_trigger();
            frameGrabber_1.set_trigger();
            // zedCamera.set_trigger();
            xsense_sensor.set_trigger();
            
            // wait until all devices have captured 
            while(
                lidar_obj.get_trigger()
                || depthCamera_obj.get_trigger()
                || frameGrabber_0.get_trigger() 
                || frameGrabber_1.get_trigger()
                // || zedCamera.get_trigger()
                || xsense_sensor.get_trigger()
            ){};

            // uncomment below to print the time it takes to record a frame to the disk 
            std::chrono::duration<float> ts = std::chrono::high_resolution_clock::now() - startNewFrame;
            std::cout << "sample time: " << ts.count() << std::endl; 

        }
    }
    });

    // when the user hits enter the program will end 
	std::cout << "Press [ENTER] to stop." << std::endl;
	std::cin.get();
	wake = false;

    // release the devices
	xsense_sensor.release_device();
    depthCamera_obj.release_device();
    lidar_obj.release_device();
    frameGrabber_0.release_device();
    frameGrabber_1.release_device();
	
    // join all the threads to make sure the end correctly 
	trigger_thread.join();

    // lidar
    lidar_poll_thread.join();
    lidar_save_thread.join();

    // realsense 
    depthCamera_poll_thread.join();
    depthCamera_save_thread.join();

    // IMU 
	xsense_sensor_save_thread.join();

    // framegrabbers
    frameGrabber_0_save_thread.join();
    frameGrabber_1_save_thread.join();

    std::cout << "END OF PROGRAM" << std::endl;

    return EXIT_SUCCESS;
}

/*********************************** Main Loop End ***********************************/

/********************************** Xsens functions **********************************/

Xsense_sensor::Xsense_sensor(Json::Value root){
    
    // setup filebase
    file_base = root["output_path"].asString() + "IMU_txt/frame_";

    // Creating XsControl object
	control = XsControl::construct();
	assert(control != nullptr);

	XsVersion version;
	xdaVersion(&version);
	std::cout << "INFO: Using XDA version: " << version.toString().toStdString() << std::endl;

	// Scanning for devices
	XsPortInfoArray portInfoArray = XsScanner::scanPorts();

	// Find an MTi device
	for (auto const &portInfo : portInfoArray)
	{
		if (portInfo.deviceId().isMti() || portInfo.deviceId().isMtig())
		{
			mtPort = portInfo;
			break;
		}
	}

	// Opening port
    control->openPort(mtPort.portName().toStdString(), mtPort.baudrate());

	// Get the device object
	device = control->device(mtPort.deviceId());
    // make sure the device is found 
	assert(device != nullptr);

	// Create and attach callback handler to device
	device->addCallbackHandler(&callback);

	// Put the device into configuration mode before configuring the device
	device->gotoConfig();
	
	// Configuring the device...
	XsOutputConfigurationArray configArray;
	configArray.push_back(XsOutputConfiguration(XDI_PacketCounter, 0));
	configArray.push_back(XsOutputConfiguration(XDI_SampleTimeFine, 0));

    configArray.push_back(XsOutputConfiguration(XDI_Acceleration, 100));
    configArray.push_back(XsOutputConfiguration(XDI_RateOfTurn, 100));
    configArray.push_back(XsOutputConfiguration(XDI_MagneticField, 100));
    
    device->setOutputConfiguration(configArray);

	// Putting device into measurement mode
	device->gotoMeasurement();

}

void Xsense_sensor::save_thread(){

    // orientation buffers
    XsQuaternion quaternion;
    XsEuler euler;

    // CalibratedData buffers 
    XsVector acc;
    XsVector gyr;
    XsVector mag;

    XsDataPacket packet;

    bool CalibratedData_ready = false;
    bool Orientation_ready = false;

	while (wake)
	{
        
        if (callback.packetAvailable())
		{
			std::cout << std::setw(5) << std::fixed << std::setprecision(5);

			// Retrieve a packet
			packet = callback.getNextPacket();

			if (packet.containsOrientation())
			{
				quaternion = packet.orientationQuaternion();
                euler = packet.orientationEuler();

                Orientation_ready = true;
			}

            if (packet.containsCalibratedData())
			{
				acc = packet.calibratedAcceleration();
				gyr = packet.calibratedGyroscopeData();
				mag = packet.calibratedMagneticField();

                CalibratedData_ready = true;
			}


            ready = Orientation_ready*CalibratedData_ready;
		}

        if(capture_trigger && ready){

            // create filename 
            std::stringstream fname;
            fname << file_base << std::to_string(last_frame_number) << ".txt";

            std::ofstream out(fname.str());
            out << std::fixed << std::setprecision(4);

            out << "q0:" << quaternion.w()
                << ", q1:" << quaternion.x()
                << ", q2:" << quaternion.y()
                << ", q3:" << quaternion.z();

            out << ", Roll:" << euler.roll()
                << ", Pitch:" << euler.pitch()
                << ", Yaw:" << euler.yaw();

            out << ", Acc X:" << acc[0]
                << ", Acc Y:" << acc[1]
                << ", Acc Z:" << acc[2];

            out << ", Gyr X:" << gyr[0]
                << ", Gyr Y:" << gyr[1]
                << ", Gyr Z:" << gyr[2];

            out << ", Mag X:" << mag[0]
                << ", Mag Y:" << mag[1]
                << ", Mag Z:" << mag[2];


            out.close();

            last_frame_number++;

            capture_trigger = false;
        }
	}
}

std::thread Xsense_sensor::spawn_save_thread(){
    return std::thread( [this] { save_thread(); } );
}

void Xsense_sensor::set_trigger(){
    capture_trigger = true;
}

bool Xsense_sensor::get_trigger(){
    return capture_trigger;
}

void Xsense_sensor::release_device(){

    ready = false;
    // set wake flag to false to put the threads to sleep 
    wake = false;

    // Closing port;
	control->closePort(mtPort.portName().toStdString());

	// Freeing XsControl object 
	control->destruct();

}

void Xsense_sensor::wait_for_ready(){
    std::cout << "INFO: Waiting for Xsense Camera" << std::endl;
    wait_for_timeout(&ready, 10.0);
}

/******************************* FrameGrabber functions ******************************/

/**
    Callback function which is passed to MWCreateVideoCapture from the magewell library 
    this will be called when a new frame is available from the camera to update the 
    frame stored in the CAMERA_FRAME type - this will always be the most recent frame 
    from the IR camera, or whatever you have plugged into the framegrabber.

    @param pBuffer pointer to the buffer where the frame is stored.
    @param BufferLen this is the frame size (MW_CAPTURE_HEIGHT*MW_CAPTURE_WIDTH*[number of channels])
    @param pParam pointer to the parameter passed to this function - this is the CAMERA_FRAME
                  type which stores data for the magewell framegrabber, this need to be 
                  passed to MWCreateVideoCapture function from the magewell library - see main
*/
void mw_CaptureCallback(BYTE *pBuffer, long BufferLen, void* pParam)
{
    MW_FRAME *pMW_FRAME = (MW_FRAME*)pParam;

    // image from frame grabber has two channels YUY2 format 
    // cv::Mat temp1 = cv::Mat(MW_CAPTURE_HEIGHT,MW_CAPTURE_WIDTH, CV_8UC2, ( unsigned char *)pBuffer);
    cv::Mat temp1 = cv::Mat(pMW_FRAME->capture_height,pMW_FRAME->capture_width, CV_8UC2, ( unsigned char *)pBuffer);

    cv::Mat temp2 = temp1(pMW_FRAME->roi); // crop image
    cv::resize(temp2,temp2,{pMW_FRAME->width,pMW_FRAME->height},0,0); // resize image 

    while(pMW_FRAME->busy){/* wait while buffers are busy */}
    pMW_FRAME->busy = true;
    cv::cvtColor(temp2, pMW_FRAME->frame, cv::COLOR_YUV2GRAY_YUY2);
    pMW_FRAME->busy = false;
}

// constructor for my custom class
FrameGrabberInstance::FrameGrabberInstance(Json::Value root, uint8_t t_nIndex){
    
    MWCAP_CHANNEL_INFO t_info;
    if(MWGetChannelInfoByIndex(t_nIndex, &t_info)==MW_SUCCEEDED){
        std::cout << std::to_string(t_nIndex) << " : " << t_info.szProductName << std::endl;
    }
    else{
        std::cout << std::to_string(t_nIndex) << " : Failed to grab info." << std::endl;
    }

    // define files base - this is where the frames will be dumped 
    file_base = root["output_path"].asString() + "framegrabber_" + std::to_string(t_nIndex) + "/frame_";

    // for checking file base 
    // std::cout << file_base << std::endl;

    // get realsense config 
    auto frameGrabber_config = root["FrameGrabber_config"];

    int MW_CAPTURE_WIDTH = frameGrabber_config["capture_width"].asInt();
    int MW_CAPTURE_HEIGHT = frameGrabber_config["capture_height"].asInt();
    int CAMERA_FRAMERATE = frameGrabber_config["frame_rate"].asInt();

    // init the frame object for capture 
    // there is a black boarder around the image so 
    // roi is used to crop the capture. 
    int lBorder = ceil(0.125*MW_CAPTURE_WIDTH);
    int rBorder = lBorder;
    int uBorder = lBorder;
    int dBorder = ceil(1.175*lBorder);

    MW_frame.roi.x = lBorder;
    MW_frame.roi.y = uBorder;
    MW_frame.roi.width = MW_CAPTURE_WIDTH - lBorder - rBorder;
    MW_frame.roi.height = MW_CAPTURE_HEIGHT - uBorder - dBorder;

    MW_frame.capture_width = MW_CAPTURE_WIDTH;
    MW_frame.capture_height = MW_CAPTURE_HEIGHT;

    // define frame actual width and height, create buffer
    int width = frameGrabber_config["width"].asInt();
    int height = frameGrabber_config["height"].asInt();
    MW_frame.width = width;
    MW_frame.height = height;
    MW_frame.frame = cv::Mat(width, height, CV_8U);

    // access device by instance
    char wPath[256] = {0};
    MWGetDevicePath(t_nIndex, wPath);

    hChannel = MWOpenChannelByPath(wPath);

    MWSetVideoInputAspectRatio(hChannel,0,0);// default is 0,0

    /******************** Start Capture *******************/
 
    // init handle for video capture - this will start capture
    hVideo = MWCreateVideoCapture(
                                        hChannel,
                                        MW_CAPTURE_WIDTH,
                                        MW_CAPTURE_HEIGHT,
                                        MWFOURCC_YUY2,
                                        CAMERA_FRAMERATE,
                                        mw_CaptureCallback,
                                        &MW_frame // to be passed to callback function
                                        );
    
}

uint8_t FrameGrabberInstance::getframe(cv::Mat* buffer){

    if (MW_frame.frame.empty()) 
    {
        return 1;
    }

    while(MW_frame.busy){/* wait while buffers are busy */}
    MW_frame.busy = true;
    MW_frame.frame.copyTo(*buffer);
    MW_frame.busy = false;
    
    return 0;
}

void FrameGrabberInstance::save_thread()
{
    // create buffer to for frame which will be saved in 
    cv::Mat cv_frame = cv::Mat(MW_frame.width, MW_frame.height, CV_8U);
    
    while(!MW_frame.frame.empty()){/*wait for first frame to be captured*/}
    
    while (wake) {

        while(!capture_trigger && wake){/*waiting for capture to trigger*/}
        if(!wake){break;} // check if thread should be alive 

        // create the filename
        std::stringstream fname;
        fname << file_base << std::to_string(last_frame_number) << ".bmp";

        // grab frame 
        getframe(&cv_frame);

        // save file 
        cv::imwrite(fname.str(), cv_frame);

        last_frame_number++;
        capture_trigger = false;
        // std::cout << "frame captured" << std::endl;
    }       
}

std::thread FrameGrabberInstance::spawn_save_thread(){
    return std::thread( [this] { save_thread(); } );
}

void FrameGrabberInstance::set_trigger(){
    capture_trigger = true;
}

bool FrameGrabberInstance::get_trigger(){
    return capture_trigger;
}

void FrameGrabberInstance::wait_for_ready(){
    std::cout << "INFO: Waiting for FrameGrabber" << std::endl;
    // set up timeout and wait for first frame to come in 
    auto t_start = std::chrono::high_resolution_clock::now();
    while (MW_frame.frame.empty())
    {
        std::chrono::duration<float> t_waiting = std::chrono::high_resolution_clock::now() - t_start;
        if(t_waiting.count() >= 10)
            throw std::runtime_error("Frame never arrived");
    }
}

void FrameGrabberInstance::release_device(){
    MWDestoryVideoCapture(hVideo);
    MWCloseChannel(hChannel);
    ready = false;
    wake = false;
}

/****************************** Depth Camera Definitions ******************************/

DepthCamera::DepthCamera(Json::Value root)
{
    file_base = root["output_path"].asString();

    // get realsense config 
    auto RealSense_config = root["RealSense_config"];

    int fps = RealSense_config["frame_rate"].asInt();
    int color_width = RealSense_config["color_width"].asInt();
    int color_height = RealSense_config["color_height"].asInt();
    int nir_width = RealSense_config["nir_width"].asInt();
    int nir_height = RealSense_config["nir_height"].asInt();

    // define buffers 
    color_frame = cv::Mat(
        color_height,
        color_width, 
        CV_8UC3
    );
    left_ir_frame = cv::Mat(
        nir_height,
        nir_width, 
        CV_8UC1
    );
    right_ir_frame= cv::Mat(
        nir_height,
        nir_width,
        CV_8UC1
    );

    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    cfg.enable_stream(
        RS2_STREAM_COLOR, 
        color_width, 
        color_height, //RealSense_config["color_height"].asInt(),
        RS2_FORMAT_BGR8, 
        fps
    );

    cfg.enable_stream(
        RS2_STREAM_INFRARED, 
        1, 
        nir_width,
        nir_height,
        RS2_FORMAT_Y8, 
        fps
    );

    cfg.enable_stream(
        RS2_STREAM_INFRARED, 
        2, 
        nir_width, 
        nir_height,
        RS2_FORMAT_Y8, 
        fps
    );

    // start pipeline
    pipe = rs2::pipeline(ctx);
    rs2::pipeline_profile selection = pipe.start(cfg);

    // additional device config
    rs2::device selected_device = selection.get_device();

    auto depth_sensor = selected_device.first<rs2::depth_sensor>();

    if (RealSense_config["enable_emmiter"].asBool()){
        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f); // Enable emitter
    }
    else{
        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f); // Disable emitter
    }
}

void DepthCamera::poll_thread(){

    cv::Mat color_frame_poll;
    cv::Mat left_ir_frame_poll;
    cv::Mat right_ir_frame_poll;

    rs2::frameset frames;
    rs2::frame colored_frame;
    rs2::frame ir_frame_left;
    rs2::frame ir_frame_right;

    while(wake){
        if(pipe.poll_for_frames(&frames)){

            colored_frame = frames.get_color_frame();
            ir_frame_left = frames.get_infrared_frame(1);
            ir_frame_right = frames.get_infrared_frame(2);

            color_frame_poll = frame_to_mat(colored_frame);
            left_ir_frame_poll = frame_to_mat(ir_frame_left);
            right_ir_frame_poll = frame_to_mat(ir_frame_right);

            while(busy){/* wait while buffers are busy */}

            // convert to opencv and copy to frame buffers
            busy = true;
            std::swap(color_frame, color_frame_poll);
            std::swap(left_ir_frame, left_ir_frame_poll);
            std::swap(right_ir_frame, right_ir_frame_poll);
            busy = false;

            ready = true;
        }
    }
}

void DepthCamera::save_thread()
{
    cv::Mat color_frame_save;
    cv::Mat left_ir_frame_save;
    cv::Mat right_ir_frame_save;

    while(!ready){/*wait for first frame to be captured*/}
    
    while (1) {

        while(!capture_trigger && wake){/*waiting for capture to trigger*/}
        if(!wake){break;} // check if thread should be alive 

        while(busy){/*scan is being wrote to*/}

        // grab frames from buffer
        busy = true;
        std::swap(color_frame, color_frame_save);
        std::swap(left_ir_frame, left_ir_frame_save);
        std::swap(right_ir_frame, right_ir_frame_save);
        busy = false;

        std::stringstream fname_rgb;
        std::stringstream fname_nir_l;
        std::stringstream fname_nir_r;

        fname_rgb    << file_base << "/rgb/frame_"   << std::to_string(last_frame_number) << ".bmp";
        fname_nir_l  << file_base << "/nir_l/frame_" << std::to_string(last_frame_number) << ".bmp";
        fname_nir_r  << file_base << "/nir_r/frame_" << std::to_string(last_frame_number) << ".bmp";

        cv::imwrite(fname_rgb.str(), color_frame_save);
        cv::imwrite(fname_nir_l.str(), left_ir_frame_save);
        cv::imwrite(fname_nir_r.str(), right_ir_frame_save);

        last_frame_number++;
        capture_trigger = false;
        // std::cout << "frame captured" << std::endl;
    }       
}

std::thread DepthCamera::spawn_poll_thread(){
    return std::thread( [this] { poll_thread(); } );
}

std::thread DepthCamera::spawn_save_thread(){
    return std::thread( [this] { save_thread(); } );
}

void DepthCamera::set_trigger(){
    capture_trigger = true;
}

bool DepthCamera::get_trigger(){
    return capture_trigger;
}

void DepthCamera::wait_for_ready(){
    std::cout << "INFO: Waiting for Depth Camera" << std::endl;
    wait_for_timeout(&ready, 10.0);
}

void DepthCamera::release_device(){
    wake = false;
    pipe.stop();
}

/****************************** Depth Camera Definitions ******************************/

ZedCamera::ZedCamera(Json::Value root)
{
    // where the images will be saved
    file_base = root["output_path"].asString() + "zed_rgb/";

    // get realsense config 
    auto zed_opencv_config = root["Zed_opencv_config"];

    int width = zed_opencv_config["width"].asInt();
    int height = zed_opencv_config["height"].asInt();

    image_size = cv::Size2i(width, height);

    // open capture 
    int cap_no = zed_opencv_config["cap_no"].asInt();
    cap = cv::VideoCapture("v4l2src device=/dev/video0 ! video/x-raw, format=YUY2 ! videoconvert ! video/x-raw, format=BGR ! appsink drop=1",
                            cv::CAP_GSTREAMER);

    cap.grab();
    // Set the video resolution (2*Width * Height)
    cap.set(cv::CAP_PROP_FRAME_WIDTH, image_size.width * 2);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, image_size.height);
    cap.grab();
    
    if (cap.isOpened())
        ready = true;

}

void ZedCamera::save_thread()
{
    while(!ready){/*wait for first frame to be captured*/}
    
    while (1) {

        while(!capture_trigger){/*waiting for capture to trigger*/}

        // Get a new frame from camera
        cap >> frame;

        // create filename 
        std::stringstream fname;
        fname << file_base << std::to_string(last_frame_number) << ".bmp";

        // save the file 
        cv::imwrite(fname.str(), frame);

        last_frame_number++;
        capture_trigger = false;
    }       
}

std::thread ZedCamera::spawn_save_thread(){
    return std::thread( [this] { save_thread(); } );
}

void ZedCamera::set_trigger(){
    capture_trigger = true;
}

bool ZedCamera::get_trigger(){
    return capture_trigger;
}

void ZedCamera::wait_for_ready(){
    std::cout << "INFO: Waiting for Zed Camera" << std::endl;
    wait_for_timeout(&ready, 10.0);
}



/********************************* Lidar Definitions *********************************/

Lidar::Lidar(Json::Value root)
{
    
    file_base = root["output_path"].asString() + "bin_files/cloud_";

    auto Ouster_config = root["Ouster_config"];

    const std::string sensor_hostname = Ouster_config["hostname"].asString();
    const std::string udp_dest = Ouster_config["udp_dest"].asString();

    sensor::lidar_mode mode = sensor::lidar_mode_of_string(Ouster_config["lidar_mode"].asString());

    const int lidar_port = Ouster_config["lidar_port"].asInt();;
    const int imu_port = Ouster_config["imu_port"].asInt();

    samplePeriod = root["SamplePeriod"].asFloat();

    std::cerr << "Connecting to \"" << sensor_hostname << "\"... ";

    handle = sensor::init_client(
                                    sensor_hostname, 
                                    udp_dest, mode,
                                    sensor::TIME_FROM_UNSPEC, 
                                    lidar_port, 
                                    imu_port
    );
    
    if (!handle) FATAL("Failed to connect");
    std::cerr << "ok" << std::endl;

    std::cerr << "Gathering metadata..." << std::endl;
    auto metadata = sensor::get_metadata(*handle);

    // Raw metadata can be parsed into a `sensor_info` struct
    info = sensor::parse_metadata(metadata);

    scan_w = info.format.columns_per_frame;
    scan_h = info.format.pixels_per_column;

    std::cerr << "  Firmware version:  " << info.fw_rev
              << "\n  Serial number:     " << info.sn
              << "\n  Product line:      " << info.prod_line
              << "\n  Scan dimensions:   " << scan_w << " x " << scan_h << std::endl;

    // create buffer to store scans as they are passed between threads
    std::unique_ptr<ouster::LidarScan> ls_poll_(
    new ouster::LidarScan{scan_w, scan_h, info.format.udp_profile_lidar});
    ls_poll = std::move(ls_poll_);

    std::unique_ptr<ouster::LidarScan> ls_swap_(
    new ouster::LidarScan{scan_w, scan_h, info.format.udp_profile_lidar});
    ls_swap = std::move(ls_swap_);

    std::unique_ptr<ouster::LidarScan> ls_save_(
    new ouster::LidarScan{scan_w, scan_h, info.format.udp_profile_lidar});
    ls_save = std::move(ls_save_);

    // init look up table 
    lut = ouster::make_xyz_lut(info);

}

void Lidar::poll_thread(){
    // get information on the packet format 
    ouster::sensor::packet_format packet_format = sensor::get_format(info);

    auto batch = ouster::ScanBatcher(scan_w, packet_format);

    // buffer to store raw lidar data in
    std::unique_ptr<uint8_t[]> lidar_buf(
        new uint8_t[packet_format.lidar_packet_size + 1]);
    std::unique_ptr<uint8_t[]> imu_buf(
        new uint8_t[packet_format.imu_packet_size + 1]);

    while(wake)
    {
        // Poll the client for data and add to our lidar scan
        sensor::client_state st = sensor::poll_client(*handle);

        if (st & sensor::client_state::CLIENT_ERROR) {
            std::cerr << "Client returned error state" << std::endl;
            std::exit(EXIT_FAILURE);

        }

        // if sensor returns lidar data read packet and parse the scan
        if (st & sensor::client_state::LIDAR_DATA) {

            if (sensor::read_lidar_packet(*handle, lidar_buf.get(),
                                            packet_format)) {

                if (batch(lidar_buf.get(), *ls_poll)) {

                    while(busy){/*scan is being read*/}
                    busy = true;
                    std::swap(ls_swap, ls_poll);
                    busy = false;

                }
            }
        }

        // do nothing if imu packet received 
        if (st & sensor::client_state::IMU_DATA) {
            sensor::read_imu_packet(*handle, imu_buf.get(), packet_format);
        }

        ready = true;
    }
}

void Lidar::save_thread()
{
    while (1) {

        while(!capture_trigger && wake){/*waiting for capture to trigger*/}
        if(!wake){break;}

        while(busy){/*scan is being wrote to*/}
        busy = true;
        std::swap(ls_swap, ls_save);
        busy = false;

        // convert scan to point cloud 
        cloud = ouster::cartesian(*ls_save, lut);

        // set up file to write to 
        std::stringstream filename;
        filename << file_base << std::to_string(file_ind++) << ".bin";
        std::ofstream out(filename.str(), std::ios::out | std::ios::binary | std::ios::trunc);
        out.write((char*) cloud.data(), scan_h*scan_w*3*sizeof(double));
        out.close();

        capture_trigger = false;
        // std::cout << "frame captured" << std::endl;
    }       
}

std::thread Lidar::spawn_poll_thread(){
    return std::thread( [this] { poll_thread(); } );
}

std::thread Lidar::spawn_save_thread(){
    return std::thread( [this] { save_thread(); } );
}

void Lidar::set_trigger(){
    capture_trigger = true;
}

bool Lidar::get_trigger(){
    return capture_trigger;
}

void Lidar::wait_for_ready(){
    std::cout << "INFO: Waiting for Lidar" << std::endl;
    wait_for_timeout(&ready, 10.0);
}

void Lidar::release_device(){
    ready = false;
    wake = false;
}

/********************************** Helper Functions *********************************/

// custom timeout frunction - waits for flag to be set within timeout
void wait_for_timeout(bool* p_flag, float timeout_seconds){
       // set up time out and wait for first frame to come in 
    auto t_start = std::chrono::high_resolution_clock::now();
    
    while (!*p_flag)
    {
        std::chrono::duration<float> t_waiting = std::chrono::high_resolution_clock::now() - t_start;
        if(t_waiting.count() >= timeout_seconds)
            throw std::runtime_error("wait for exceeded");
    }
}


// helper function to convert from rs2 frame to cv mat type
// Convert rs2::frame to cv::Mat
static cv::Mat frame_to_mat(const rs2::frame& f)
{

    auto vf = f.as<rs2::video_frame>();
    const int w = vf.get_width();
    const int h = vf.get_height();

    if (f.get_profile().format() == RS2_FORMAT_BGR8)
    {
        return cv::Mat(cv::Size(w, h), CV_8UC3, (void*)f.get_data(), cv::Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_RGB8)
    {
        auto r_rgb = cv::Mat(cv::Size(w, h), CV_8UC3, (void*)f.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat r_bgr;
        cvtColor(r_rgb, r_bgr, cv::COLOR_RGB2BGR);
        return r_bgr;
    }
    else if (f.get_profile().format() == RS2_FORMAT_Z16)
    {

        return cv::Mat(cv::Size(w, h), CV_16UC1, (void*)f.get_data(), cv::Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_Y8)
    {
        return cv::Mat(cv::Size(w, h), CV_8UC1, (void*)f.get_data(), cv::Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_DISPARITY32)
    {
        return cv::Mat(cv::Size(w, h), CV_32FC1, (void*)f.get_data(), cv::Mat::AUTO_STEP);
    } 

    throw std::runtime_error("Frame format is not supported yet!");
}

