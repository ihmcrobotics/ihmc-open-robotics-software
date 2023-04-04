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

#ifndef SENSORCAPTURE_HPP
#define SENSORCAPTURE_HPP

#include "defines.hpp"

#include <thread>
#include <vector>
#include <map>
#include <mutex>

#ifdef SENSORS_MOD_AVAILABLE

#include "sensorcapture_def.hpp"
#include "hidapi.h"

namespace sl_oc {

#ifdef VIDEO_MOD_AVAILABLE
namespace video {
class VideoCapture;
}
#endif

namespace sensors {

namespace data {

/*!
 * \brief Contains the acquired Imu data
 */
struct SL_OC_EXPORT Imu
{
    // Validity of the magnetometer sensor data
    typedef enum _imu_status {
        NOT_PRESENT = 0,
        OLD_VAL = 1,
        NEW_VAL = 2
    } ImuStatus;

    ImuStatus valid = NOT_PRESENT;     //!< Indicates if IMU data are valid
    uint64_t timestamp = 0; //!< Timestamp in nanoseconds
    float aX;               //!< Acceleration along X axis in m/s²
    float aY;               //!< Acceleration along Y axis in m/s²
    float aZ;               //!< Acceleration along Z axis in m/s²
    float gX;               //!< Angular velocity around X axis in °/s
    float gY;               //!< Angular velocity around Y axis in °/s
    float gZ;               //!< Angular velocity around > axis in °/s
    float temp;             //!< Sensor temperature in °C
    bool sync;              //!< Indicates in IMU data are synchronized with a video frame
};

/*!
 * \brief Contains the acquired Magnetometer data
 */
struct SL_OC_EXPORT Magnetometer
{
    // Validity of the magnetometer sensor data
    typedef enum _mag_status {
        NOT_PRESENT = 0,
        OLD_VAL = 1,
        NEW_VAL = 2
    } MagStatus;

    MagStatus valid = NOT_PRESENT;     //!< Indicates if Magnetometer data are valid
    uint64_t timestamp = 0; //!< Timestamp in nanoseconds
    float mX;               //!< Acceleration along X axis in uT
    float mY;               //!< Acceleration along Y axis in uT
    float mZ;               //!< Acceleration along Z axis in uT
};

/*!
 * \brief Contains the acquired Environment data
 */
struct SL_OC_EXPORT Environment
{
    // Validity of the environmental sensor data
    typedef enum _env_status {
        NOT_PRESENT = 0,
        OLD_VAL = 1,
        NEW_VAL = 2
    } EnvStatus;

    EnvStatus valid = NOT_PRESENT;     //!< Indicates if Environmental data are valid
    uint64_t timestamp = 0; //!< Timestamp in nanoseconds
    float temp;             //!< Sensor temperature in °C
    float press;            //!< Atmospheric pressure in hPa
    float humid;            //!< Humidity in %rH
};

/*!
 * \brief Contains the acquired Camera Temperature data
 */
struct SL_OC_EXPORT Temperature
{
    typedef enum _temp_status {
        NOT_PRESENT = 0,
        OLD_VAL = 1,
        NEW_VAL = 2
    } TempStatus;

    TempStatus valid = NOT_PRESENT;     //!< Indicates if camera temperature data are valid
    uint64_t timestamp = 0; //!< Timestamp in nanoseconds
    float temp_left;        //!< Temperature of the left CMOS camera sensor
    float temp_right;       //!< Temperature of the right CMOS camera sensor
};

}

/*!
 * \brief The SensorCapture class provides sensor grabbing functions for the Stereolabs ZED Mini and ZED2 camera models
 */
class SL_OC_EXPORT SensorCapture
{
    ZED_OC_VERSION_ATTRIBUTE;

public:
    /*!
     * \brief The default constructor
     * \param verbose_lvl enable useful information to debug the class behaviours while running
     */
    SensorCapture( sl_oc::VERBOSITY verbose_lvl=sl_oc::VERBOSITY::ERROR );

    /*!
     * \brief The class destructor
     */
    virtual ~SensorCapture();

    /*!
     * \brief Get the list of the serial number of all the available devices
     * \param refresh if true USB device tree is parsed to search for modifications (new device connected/disconnected)
     * \return a vector containing the serial number of all the available devices
     */
    std::vector<int> getDeviceList(bool refresh=false);

    /*!
     * \brief Open a connection to the MCU of a ZED Mini or a ZED2 camera using the specified serial number or searching
     *        for the first available device
     * \param sn Serial Number of the camera. Use `-1` to open connect to the first available device
     * \return returns true if the connection is correctly estabilished
     */
    bool initializeSensors( int sn=-1 );

    /*!
     * \brief Get the MCU firmware version in form [fw_major].[fw_minor]
     * \param fw_major the major firmware version number
     * \param fw_minor the minor firmware version number
     */
    void getFirmwareVersion( uint16_t& fw_major, uint16_t& fw_minor );

    /*!
     * \brief Retrieve the serial number of the connected camera
     * \return the serial number of the connected camera
     */
    int getSerialNumber();

    /*!
     * \brief Get the last received IMU data
     * \param timeout_usec data grabbing timeout in milliseconds.
     * \return returns a reference to the last received data.
     */
    const data::Imu& getLastIMUData(uint64_t timeout_usec=1500);

    /*!
     * \brief Get the last received Magnetometer data
     * \param timeout_usec data grabbing timeout in milliseconds.
     * \return returns a reference to the last received data.
     */
    const data::Magnetometer& getLastMagnetometerData(uint64_t timeout_usec=100);

    /*!
     * \brief Get the last received Environment data
     * \param timeout_usec data grabbing timeout in milliseconds.
     * \return returns a reference to the last received data.
     */
    const data::Environment& getLastEnvironmentData(uint64_t timeout_usec=100);

    /*!
     * \brief Get the last received camera sensors temperature data
     * \param timeout_usec data grabbing timeout in milliseconds.
     * \return returns a reference to the last received data.
     */
    const data::Temperature& getLastCameraTemperatureData(uint64_t timeout_usec=100);

    /*!
     * \brief Perform a SW reset of the Sensors Module. To be called in case one of the sensors stops to work correctly.
     *
     * \param serial_number The serial number of the device to be reset (0 to reset the first available)
     * \return true if successful
     *
     * \note After the reset a new \ref SensorCapture connection is required
     * \note The Sensors Module reset automatically performs a reset of the Video Module, so a new \ref VideoCapture
     *  connection is required
     */
    static bool resetSensorModule(int serial_number=0);

    /*!
     * \brief Perform a reset of the video module without resetting the sensor module. To be called in case the Video
     * module stops to work correctly.
     *
     * \param \param serial_number The serial number of the device to be reset (0 to reset the first available)
     * \return true if successful
     *
     * \note After the reset a new \ref VideoCapture connection is required
     */
    static bool resetVideoModule(int serial_number=0);

#ifdef VIDEO_MOD_AVAILABLE
    void updateTimestampOffset(uint64_t frame_ts);                                 //!< Called by  VideoCapture to update timestamp offset
    inline void setStartTimestamp(uint64_t start_ts){mStartSysTs=start_ts;}        //!< Called by  VideoCapture to sync timestamps reference point
    inline void setVideoPtr(video::VideoCapture* videoPtr){mVideoPtr=videoPtr;}    //!< Called by  VideoCapture to set the pointer to it
#endif

private:
    static bool searchForConnectedDev(int* serial_number, unsigned short* found_pid); //!< Search for a device and returns its pid and serial number

    void grabThreadFunc();              //!< The sensor data grabbing thread function

    bool startCapture();                //!< Start data capture thread

    bool open(uint16_t pid, int serial_number); //!< Open the USB connection
    void close();                       //!< Close the USB connection

    int enumerateDevices();             //!< Populates the  mSlDevPid map with serial number and PID of the available devices

    // ----> USB commands to MCU
    bool enableDataStream(bool enable); //!< Enable/Disable the data stream
    bool isDataStreamEnabled();         //!< Check if the data stream is enabled
    bool sendPing();                    //!< Send a ping  each second (before 6 seconds) to keep data streaming alive
    // ----> USB commands to MCU

private:
    // Flags
    int mVerbose=0;                //!< Verbose status
    bool mNewIMUData=false;             //!< Indicates if new  IMU data are available
    bool mNewMagData=false;             //!< Indicates if new  MAG data are available
    bool mNewEnvData=false;             //!< Indicates if new  ENV data are available
    bool mNewCamTempData=false;         //!< Indicates if new  CAM_TEMP data are available

    bool mInitialized = false;          //!< Inficates if the MCU has been initialized
    bool mStopCapture = false;          //!< Indicates if the grabbing thread must be stopped
    bool mGrabRunning = false;          //!< Indicates if the grabbing thread is running

    std::map<int,uint16_t> mSlDevPid;   //!< All the available Stereolabs MCU (ZED-M and ZED2) product IDs associated to their serial number
    std::map<int,uint16_t> mSlDevFwVer; //!< All the available Stereolabs MCU (ZED-M and ZED2) product IDs associated to their firmware version

    hid_device* mDevHandle = nullptr;   //!< Hidapi device handler
    int mDevSerial = -1;                //!< Serial number of the connected device
    int mDevFwVer = -1;                 //!< FW version of the connected device
    unsigned short mDevPid = 0;         //!< Product ID of the connected device

    data::Imu mLastIMUData;             //!< Contains the last received IMU data
    data::Magnetometer mLastMagData;    //!< Contains the last received Magnetometer data
    data::Environment mLastEnvData;     //!< Contains the last received Environmental data
    data::Temperature mLastCamTempData; //!< Contains the last received camera sensors temperature data

    std::thread mGrabThread;            //!< The grabbing thread

    std::mutex mIMUMutex;               //!< Mutex for safe access to IMU data buffer
    std::mutex mMagMutex;               //!< Mutex for safe access to MAG data buffer
    std::mutex mEnvMutex;               //!< Mutex for safe access to ENV data buffer
    std::mutex mCamTempMutex;           //!< Mutex for safe access to CAM_TEMP data buffer

    uint64_t mStartSysTs=0;             //!< Initial System Timestamp, to calculate differences [nsec]
    uint64_t mLastMcuTs=0;              //!< MCU Timestamp of the previous data, to calculate relative timestamps [nsec]

    bool mFirstImuData=true;            //!< Used to initialize the sensor timestamp start point

    // ----> Timestamp synchronization
    uint64_t mLastFrameSyncCount=0;     //!< Used to estimate sync signal in case we lost the MCU data containing the sync signal

    std::vector<uint64_t> mMcuTsQueue;  //!< Queue to keep the latest MCU timestamps to be used to calculate the shift scaling factor
    std::vector<uint64_t> mSysTsQueue;  //!< Queue to keep the latest UVC timestamps to be used to calculate the shift scaling factor

    double mNTPTsScaling=1.0;           //!< Timestamp shift scaling factor
    int mNTPAdjustedCount = 0;          //!< Counter for timestamp shift scaling

    int64_t mSyncOffset=0;              //!< Timestamp offset respect to synchronized camera
    // <---- Timestamp synchronization

#ifdef VIDEO_MOD_AVAILABLE
    video::VideoCapture* mVideoPtr=nullptr;    //!< Pointer to the synchronized SensorCapture object
    uint64_t mSyncTs=0;                 //!< Timestamp of the latest received HW sync signal
#endif

};

#endif

/** \example zed_oc_sensors_example.cpp
 * Example of how to use the SensorCapture class to get the raw sensors data at the maximum available
 * frequency.
 */

/** \example zed_oc_sync_example.cpp
 * Example of how to get synchronized video and sensors data from
 * the VideoCapture class and the SensorCapture class.
 */

}
}


#endif // SENSORCAPTURE_HPP
