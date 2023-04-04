///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2020, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

#ifndef VIDEOCAPTURE_HPP
#define VIDEOCAPTURE_HPP

#include "defines.hpp"
#include <thread>
#include <mutex>
#include <fstream>      // std::ofstream
#include <iomanip>

#define LOG_SEP ","

#ifdef VIDEO_MOD_AVAILABLE

#include "videocapture_def.hpp"

namespace sl_oc {



#ifdef SENSORS_MOD_AVAILABLE
namespace sensors {
class SensorCapture;
}
#endif

namespace video {

/*!
 * \brief The Frame struct containing the acquired video frames
 */
struct SL_OC_EXPORT Frame
{
    uint64_t frame_id = 0;          //!< Increasing index of frames
    uint64_t timestamp = 0;         //!< Timestamp in nanoseconds
    uint8_t* data = nullptr;        //!< Frame data in YUV 4:2:2 format
    uint16_t width = 0;             //!< Frame width
    uint16_t height = 0;            //!< Frame height
    uint8_t channels = 0;           //!< Number of channels per pixel
};

/*!
 * \brief The VideoCapture class provides image grabbing functions and settings control for all the Stereolabs camera models
 */
class SL_OC_EXPORT VideoCapture
{
    ZED_OC_VERSION_ATTRIBUTE;

public:
    /*!
     * \brief The default constructor
     *  * \param params the initialization parameters (see  VideoParams)
     */
    VideoCapture( VideoParams params = VideoParams() );

    /*!
     * \brief The class destructor
     */
    virtual ~VideoCapture();

    /*!
     * \brief Open a ZED camera using the specified ID or searching for the first available
     * \param devId Id of the camera (see `/dev/video*`). Use `-1` to open the first available camera
     * \return returns true if the camera is correctly opened
     */
    bool initializeVideo( int devId=-1 );

    /*!
     * \brief Get the last received camera image
     * \param timeout_msec frame grabbing timeout in millisecond.
     * \return returns a reference to the last received frame as pointer.
     *
     * \note To check if a new frame is available it is possible to compare the timestamp of the returned frame with
     * the timestamp of the previous valid frame
     *
     * \note Frame received will contains the RAW buffer from the camera, in YUV4:2:2 color format and in side by side mode.
     * Images must then be converted to RGB for proper display and will not be rectified.
     */
    const Frame& getLastFrame(uint64_t timeout_msec=100);

    /*!
     * \brief Get the size of the camera frame
     * \param width the frame width
     * \param height the frame height
     */
    inline void getFrameSize( int& width, int& height ){width=mWidth;height=mHeight;}

    // ----> Led Control
    /*!
     * \brief Set the status of the camera led
     * \param status true for "ON", false for "OFF"
     * \return returns a negative value if an error occurred
     */
    int setLEDstatus(bool status);

    /*!
     * \brief Get the status of the camera led
     * \param status returned status: true for "ON", false for "OFF"
     * \return returns a negative value if an error occurred
     */
    int getLEDstatus(bool *status);

    /*!
     * \brief Toggle the status of the camera led
     * \param value returned status: true for "ON", false for "OFF"
     * \return returns a negative value if an error occurred
     */
    int toggleLED(bool *value);
    // <---- Led Control

    // ----> Camera Settings control
    /*!
     * \brief Set the Brightness value
     * \param brightness Brightness value in the range [0,8]
     */
    void setBrightness(int brightness);

    /*!
     * \brief Get the Brightness value
     * \return the current Brightness value
     */
    int getBrightness();

    /*!
     * \brief Reset the Brightness value to default value
     */
    void resetBrightness();

    /*!
     * \brief Set the Sharpness value
     * \param sharpness Sharpness value in the range [0,8]
     */
    void setSharpness(int sharpness);

    /*!
     * \brief Get the Sharpness value
     * \return the current Sharpness value
     */
    int getSharpness();

    /*!
     * \brief Reset the Sharpness value to default value
     */
    void resetSharpness();

    /*!
     * \brief Set the Contrast value
     * \param contrast Contrast value in the range [0,8]
     */
    void setContrast(int contrast);

    /*!
     * \brief Get the Contrast value
     * \return the current Contrast value
     */
    int getContrast();

    /*!
     * \brief Reset the Contrast value to default value
     */
    void resetContrast();

    /*!
     * \brief Set the Hue value
     * \param hue Hue value in the range [0,11]
     */
    void setHue(int hue);

    /*!
     * \brief Get the Hue value
     * \return the current Hue value
     */
    int getHue();

    /*!
     * \brief Reset the Hue value to default value
     */
    void resetHue();

    /*!
     * \brief Set the Saturation value
     * \param saturation Saturation value in the range [0,8]
     */
    void setSaturation(int saturation);

    /*!
     * \brief Get the Saturation value
     * \return the current Saturation value
     */
    int getSaturation();

    /*!
     * \brief Reset the Saturation value to default value
     */
    void resetSaturation();

    /*!
     * \brief Set the White Balance value (disable auto White Balance if active)
     * \param wb White Balance value in the range [2800,6500]
     */
    void setWhiteBalance(int wb);

    /*!
     * \brief Get the White Balance value
     * \return the current White Balance value
     */
    int getWhiteBalance();

    /*!
     * \brief Enable/Disable the automatic White Balance control
     * \param active true to activate automatic White Balance
     */
    void setAutoWhiteBalance(bool active);

    /*!
     * \brief Get the status of the automatic White Balance control
     * \return the status of the automatic White Balance control
     */
    bool getAutoWhiteBalance();

    /*!
     * \brief Reset the automatic White Balance control value to default value
     */
    void resetAutoWhiteBalance();

    /*!
     * \brief Set the Gamma value
     * \param gamma Gamma value in the range [1,9]
     */
    void setGamma(int gamma);

    /*!
     * \brief Get the Gamma value
     * \return the current Gamma value
     */
    int getGamma();

    /*!
     * \brief Reset the Gamma value to default value
     */
    void resetGamma();

    /*!
     * \brief Enable/Disable the automatic Exposure and Gain control
     * \param active true to activate automatic White Balance
     */
    int setAECAGC(bool active);

    /*!
     * \brief Get the status of the automatic Exposure and Gain control
     * \return the status of the automatic Exposure and Gain control
     */
    bool getAECAGC();

    /*!
     * \brief Reset the automatic Exposure and Gain control value to default value
     */
    void resetAECAGC();

    /*!
     * \brief Set Region Of Interest (ROI) for AECAGC control
     * \param side position of the camera sensor (see  CAM_SENS_POS)
     * \param x top left ROI rectangle X coordinate
     * \param y top left ROI rectangle Y coordinate
     * \param w ROI rectangle width
     * \param h ROI rectangle width
     * \return returns true if the ROI has been correctly set
     */
    bool setROIforAECAGC(CAM_SENS_POS side, uint16_t x, uint16_t y, uint16_t w, uint16_t h);

    /*!
     * \brief Reset the ROI for AECAGC control
     * \param side position of the camera sensor (see  CAM_SENS_POS)
     * \return returns true if the ROI has been correctly reset
     */
    bool resetROIforAECAGC(CAM_SENS_POS side);

    /*!
     * \brief Get the coordinates of the current ROI for AECAGC control
     * \param side position of the camera sensor (see  CAM_SENS_POS)
     * \param x top left ROI rectangle X coordinate
     * \param y top left ROI rectangle Y coordinate
     * \param w ROI rectangle width
     * \param h ROI rectangle width
     * \return returns true if no errors occurred
     */
    bool getROIforAECAGC(CAM_SENS_POS side,uint16_t& x, uint16_t& y, uint16_t& w, uint16_t& h);

    /*!
     * \brief Set the Gain value (disable Exposure and Gain control if active)
     * \param cam position of the camera sensor (see  CAM_SENS_POS)
     * \param gain Gain value in the range [0,100]
     */
    void setGain(CAM_SENS_POS cam, int gain);

    /*!
     * \brief Get the current Gain value
     * \param cam position of the camera sensor (see  CAM_SENS_POS)
     * \return the current Gain value
     */
    int getGain(CAM_SENS_POS cam);

    /*!
     * \brief Set the Exposure value (disable Exposure and Gain control if active)
     * \param cam position of the camera sensor (see  CAM_SENS_POS)
     * \param exposure Exposure value in the range [0,100]
     */
    void setExposure(CAM_SENS_POS cam, int exposure);

    /*!
     * \brief Get the current Exposure value
     * \param cam position of the camera sensor (see  CAM_SENS_POS)
     * \return the current Exposure value
     */
    int getExposure(CAM_SENS_POS cam);
    // <---- Camera Settings control

    /*!
     * \brief Retrieve the serial number of the connected camera
     * \return the serial number of the connected camera
     */
    int getSerialNumber();

    /*!
     * \brief Utils fct to set Color Bars on Image
     */
    void setColorBars(int side, bool c);

#ifdef SENSOR_LOG_AVAILABLE
    /*!
     * \brief Start logging to file of AEG/AGC camera registers
     * \param enable set to true to enable logging
     * \param frame_skip number of frames to skip when logging to file
     * \return true if log file can be correctly created/closed
     */
    bool enableAecAgcSensLogging(bool enable, int frame_skip=10);

    /*!
     * \brief Save all ISP camera registers into a file
     * \param filename csv filename
     * \note CSV file will contain Adress , L value, R value
     */
    void saveAllISPRegisters(std::string filename);

    /*!
     * \brief Save all sensors ctrl register
     * \param filename csv filename
     * \note CSV file will contain Adress , L value, R value
     */
    void saveAllSensorsRegisters(std::string filename);
#endif


#ifdef SENSORS_MOD_AVAILABLE
    /*!
     * \brief Enable synchronizations between Camera frame and Sensors timestamps
     * \param sensCap pointer to  SensorCapture object
     * \return true if synchronization has been correctly started
     */
    bool enableSensorSync( sensors::SensorCapture* sensCap=nullptr );

    /*!
     * \brief Indicates that the  SensorCapture object received the HW sync signal and a frame must
     *        be synchronized to the last Sensor Data
     */
    inline void setReadyToSync(){ mSensReadyToSync=true; }
#endif

        bool resetAGCAECregisters();

private:
    void grabThreadFunc();  //!< The frame grabbing thread function

    // ----> Low level functions
    int ll_VendorControl(uint8_t *buf, int len, int readMode, bool safe = false, bool force=false);
    int ll_get_gpio_value(int gpio_number, uint8_t* value);
    int ll_set_gpio_value(int gpio_number, uint8_t value);
    int ll_set_gpio_direction(int gpio_number, int direction);
    int ll_read_system_register(uint64_t address, uint8_t* value);
    int ll_write_system_register(uint64_t address, uint8_t value);
    int ll_read_sensor_register(int side, int sscb_id, uint64_t address, uint8_t *value);
    int ll_write_sensor_register(int side, int sscb_id, uint64_t address, uint8_t value);

    int ll_SPI_FlashProgramRead(uint8_t *pBuf, int Adr, int len, bool force=false);

    int ll_isp_aecagc_enable(int side, bool enable);
    int ll_isp_is_aecagc(int side);

    uint8_t ll_read_reg(uint64_t addr);

    int ll_isp_get_gain(uint8_t *val, uint8_t sensorID);
    int ll_isp_set_gain(unsigned char ucGainH, unsigned char ucGainM, unsigned char ucGainL, int sensorID);
    int ll_isp_get_exposure(unsigned char *val, unsigned char sensorID);
    int ll_isp_set_exposure(unsigned char ucExpH, unsigned char ucExpM, unsigned char ucExpL, int sensorID);

    void ll_activate_sync(); // Activate low level sync signal between Camera and MCU
    // <---- Low level functions

    // ----> Mid level functions
    void setCameraControlSettings(int ctrl_id, int ctrl_val);
    void resetCameraControlSettings(int ctrl_id);
    int getCameraControlSettings(int ctrl_id);

    int setGammaPreset(int side, int value);

    int calcRawGainValue(int gain); // Convert "user gain" to "ISP gain"
    int calcGainValue(int rawGain); // Convert "ISP Gain" to "User gain"
    // <---- Mid level functions

    // ----> Connection control functions
    bool openCamera( uint8_t devId );                           //!< Open camera
    bool startCapture();                                        //!< Start video capture thread
    void reset();                                               //!< Reset camera connection
    inline void stopCapture(){mStopCapture=true;}               //!< Stop video capture thread
    int input_set_framerate(int fps);                           //!< Set UVC framerate
    int xioctl(int fd, uint64_t IOCTL_X, void *arg);            //!< Send ioctl command
    void checkResFps();                                         //!< Check if the Framerate is correct for the selected resolution
    SL_DEVICE getCameraModel(std::string dev_name);     //!< Get the connected camera model
    // <---- Connection control functions

    typedef enum _date_time
    {
        FULL,
        DATE,
        TIME
    } DateTime;

    static inline std::string getCurrentDateTime( DateTime type ){
        time_t now = time(0);
        struct tm  tstruct;
        char  buf[80];
        tstruct = *localtime(&now);
        if(type==FULL)
            strftime(buf, sizeof(buf), "%Y-%m-%d %X", &tstruct);
        else if(type==DATE)
            strftime(buf, sizeof(buf), "%Y-%m-%d", &tstruct);
        else if(type==TIME)
            strftime(buf, sizeof(buf), "%X", &tstruct);
        return std::string(buf);
    }

#ifdef SENSOR_LOG_AVAILABLE
    void saveLogDataLeft();
    void saveLogDataRight();
#endif

private:
    // Flags
    bool mNewFrame=false;               //!< Indicates if a new frame is available
    bool mInitialized=false;            //!< Inficates if the camera has been initialized
    bool mStopCapture=true;             //!< Indicates if the grabbing thread must be stopped
    bool mGrabRunning=false;            //!< Indicates if the grabbing thread is running

    VideoParams mParams;                //!< Grabbing parameters

    int mDevId = 0;                     //!< ID of the camera device
    std::string mDevName;               //!< The file descriptor path name (e.g. /dev/video0)
    int mFileDesc=-1;                   //!< The file descriptor handler

    std::mutex mBufMutex;               //!< Mutex for safe access to data buffer
    std::mutex mComMutex;               //!< Mutex for safe access to UVC communication

    int mWidth = 0;                     //!< Frame width
    int mHeight = 0;                    //!< Frame height
    int mChannels = 0;                  //!< Frame channels
    int mFps=0;                         //!< Frames per seconds

    SL_DEVICE mCameraModel = SL_DEVICE::NONE; //!< The camera model

    Frame mLastFrame;                   //!< Last grabbed frame
    uint8_t mBufCount = 2;              //!< UVC buffer count
    uint8_t mCurrentIndex = 0;          //!< The index of the currect UVC buffer
    struct UVCBuffer *mBuffers = nullptr;  //!< UVC buffers

    uint64_t mStartTs=0;                //!< Initial System Timestamp, to calculate differences [nsec]
    uint64_t mInitTs=0;                 //!< Initial Device Timestamp, to calculate differences [usec]

    int mGainSegMax=0;                  //!< Maximum value of the raw gain to be used for conversion
    int mExpoureRawMax;                 //!< Maximum value of the raw exposure to be used for conversion

    std::thread mGrabThread;            //!< The video grabbing thread

    bool mFirstFrame=true;              //!< Used to initialize the timestamp start point

#ifdef SENSOR_LOG_AVAILABLE
    // ----> Registers logging
    bool mLogEnable=false;
    std::string mLogFilenameLeft;
    std::string mLogFilenameRight;
    std::ofstream mLogFileLeft;
    std::ofstream mLogFileRight;
    int mLogFrameSkip=10;
    // <---- Registers logging
#endif


#ifdef SENSORS_MOD_AVAILABLE
    bool mSyncEnabled=false;            //!< Indicates if a  SensorCapture object is synchronized
    sensors::SensorCapture* mSensPtr;   //!< Pointer to the synchronized  SensorCapture object

    bool mSensReadyToSync=false;        //!< Indicates if the MCU received a HW sync signal
#endif
};

}

}
#endif

/** \example zed_oc_video_example.cpp
 * Example of how to use the VideoCapture class to get raw video frames and show the stream on screen using the
 * OpenCV library
 */

/** \example zed_oc_control_example.cpp
 * Example of how to use the VideoCapture class to get raw video frames and control the camera
 * parameters.
 */

/** \example zed_oc_rectify_example.cpp
 * Example of how to use the VideoCapture class to get and rectify raw video frames downloading
 * calibration parameters from Stereolabs servers
 */

#endif // VIDEOCAPTURE_HPP

