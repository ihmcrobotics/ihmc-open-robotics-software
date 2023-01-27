///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2021, STEREOLABS.
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

#include "sensorcapture.hpp"

#ifdef VIDEO_MOD_AVAILABLE

#include "videocapture.hpp"
#endif

#include <sstream>
#include <cmath>              // for round
#include <unistd.h>           // for usleep, close

namespace sl_oc {

namespace sensors {

SensorCapture::SensorCapture(VERBOSITY verbose_lvl )
{
    mVerbose = verbose_lvl;

    if( mVerbose )
    {
        std::string ver =
                "ZED Open Capture - Sensors module - Version: "
                + std::to_string(mMajorVer) + "."
                + std::to_string(mMinorVer) + "."
                + std::to_string(mPatchVer);
        INFO_OUT(mVerbose,ver );
    }
}

SensorCapture::~SensorCapture()
{
    close();
}

int SensorCapture::enumerateDevices()
{
    mSlDevPid.clear();
    mSlDevFwVer.clear();

    struct hid_device_info *devs, *cur_dev;

    if (hid_init()==-1)
        return 0;

    devs = hid_enumerate(SL_USB_VENDOR, 0x0);
    cur_dev = devs;
    while (cur_dev) {
        int fw_major = cur_dev->release_number>>8;
        int fw_minor = cur_dev->release_number&0x00FF;
        uint16_t pid = cur_dev->product_id;
        if(!cur_dev->serial_number)
        {
            cur_dev = cur_dev->next;
            continue;
        }
        std::string sn_str = wstr2str( cur_dev->serial_number );
        int sn = std::stoi( sn_str );

        mSlDevPid[sn]=pid;
        mSlDevFwVer[sn]=cur_dev->release_number;

        if(mVerbose)
        {
            std::ostringstream smsg;

            smsg << "Device Found: " << std::endl;
            smsg << "  VID: " << std::hex << cur_dev->vendor_id << " PID: " << std::hex << cur_dev->product_id << std::endl;
            smsg << "  Path: " << cur_dev->path << std::endl;
            smsg << "  Serial_number:   " << sn_str << std::endl;
            smsg << "  Manufacturer:   " << wstr2str(cur_dev->manufacturer_string) << std::endl;
            smsg << "  Product:   " << wstr2str(cur_dev->product_string) << std::endl;
            smsg << "  Release number:   v" << std::dec << fw_major << "." << fw_minor << std::endl;
            smsg << "***" << std::endl;

            INFO_OUT(mVerbose,smsg.str());
        }

        cur_dev = cur_dev->next;
    }

    hid_free_enumeration(devs);

    return mSlDevPid.size();
}

std::vector<int> SensorCapture::getDeviceList(bool refresh)
{
    if(mSlDevPid.size()==0 || refresh)
        enumerateDevices();

    std::vector<int> sn_vec;

    for(std::map<int,uint16_t>::iterator it = mSlDevPid.begin(); it != mSlDevPid.end(); ++it) {
        sn_vec.push_back(it->first);
    }

    return sn_vec;
}

bool SensorCapture::open( uint16_t pid, int serial_number)
{
    std::string sn_str = std::to_string(serial_number);
    std::wstring wide_sn_string = std::wstring(sn_str.begin(), sn_str.end());

    const wchar_t* wsn = wide_sn_string.c_str();

    mDevHandle = hid_open(SL_USB_VENDOR, pid, wsn );

    if(mDevHandle) mDevSerial = serial_number;

    return mDevHandle!=0;
}

bool SensorCapture::initializeSensors( int sn )
{
    if(mSlDevPid.size()==0)
    {
        enumerateDevices();
    }

    std::string sn_str;

    if(sn==-1)
    {
        if(mSlDevPid.size()==0)
        {
            enumerateDevices();
        }

        if(mSlDevPid.size()==0)
        {
            ERROR_OUT(mVerbose,"No available ZED Mini or ZED2 cameras");
            return false;
        }

        sn = mSlDevPid.begin()->first;
    }

    uint16_t pid = mSlDevPid[sn];

    if(!open( pid,sn))
    {
        std::string msg = "Connection to device with sn ";
        msg += std::to_string(sn);
        msg += " failed";

        ERROR_OUT(mVerbose,msg);

        mDevFwVer = -1;
        mDevSerial = -1;

        return false;
    }

    if(mVerbose)
    {
        std::string msg = "Connected to device with sn ";
        msg += std::to_string(sn);

        INFO_OUT(mVerbose,msg);
    }

    mDevFwVer = mSlDevFwVer[sn];
    mDevPid = pid;
    mInitialized = startCapture();

    return true;
}

void SensorCapture::getFirmwareVersion( uint16_t& fw_major, uint16_t& fw_minor )
{
    if(mDevSerial==-1)
        return;

    uint16_t release = mSlDevFwVer[mDevSerial];

    fw_major = release>>8;
    fw_minor = release&0x00FF;
}

int SensorCapture::getSerialNumber()
{
    if(mDevSerial==-1)
        return -1;

    return mDevSerial;
}

bool SensorCapture::enableDataStream(bool enable) {
    if( !mDevHandle )
        return false;
    unsigned char buf[65];
    buf[0] = usb::REP_ID_SENSOR_STREAM_STATUS;
    buf[1] = enable?1:0;

    int res = hid_send_feature_report(mDevHandle, buf, 2);
    if (res < 0) {
        if(mVerbose)
        {
            std::string msg = "Unable to set a feature report [SensStreamStatus] - ";
            msg += wstr2str(hid_error(mDevHandle));

            WARNING_OUT( mVerbose, msg);
        }

        return false;
    }

    return true;
}

bool SensorCapture::isDataStreamEnabled() {
    if( !mDevHandle ) {
        return false;
    }

    unsigned char buf[65];
    buf[0] = usb::REP_ID_SENSOR_STREAM_STATUS;
    int res = hid_get_feature_report(mDevHandle, buf, sizeof(buf));
    if (res < 0)
    {
        std::string msg = "Unable to get a feature report [SensStreamStatus] - ";
        msg += wstr2str(hid_error(mDevHandle));

        WARNING_OUT( mVerbose,msg );

        return false;
    }

    if( res < static_cast<int>(sizeof(usb::StreamStatus)) )
    {
        WARNING_OUT(mVerbose,std::string("SensStreamStatus size mismatch [REP_ID_SENSOR_STREAM_STATUS]"));
        return false;
    }

    if( buf[0] != usb::REP_ID_SENSOR_STREAM_STATUS )
    {
        WARNING_OUT(mVerbose,std::string("SensStreamStatus type mismatch [REP_ID_SENSOR_STREAM_STATUS]") );

        return false;
    }

    bool enabled = (buf[1]==1);

    return enabled;
}

bool SensorCapture::startCapture()
{
    if( !enableDataStream(true) )
    {
        return false;
    }

    mGrabThread = std::thread( &SensorCapture::grabThreadFunc,this );

    return true;
}

void SensorCapture::close()
{
    mStopCapture = true;

    if( mGrabThread.joinable() )
    {
        mGrabThread.join();
    }

    enableDataStream(false);

    if( mDevHandle ) {
        hid_close(mDevHandle);
        mDevHandle = nullptr;
    }

    if( mVerbose && mInitialized)
    {
        std::string msg = "Device closed";
        INFO_OUT(mVerbose,msg );
    }

    mInitialized=false;
}

void SensorCapture::grabThreadFunc()
{
    mStopCapture = false;
    mGrabRunning = false;

    mNewIMUData=false;
    mNewMagData=false;
    mNewEnvData=false;
    mNewCamTempData=false;

    // Read sensor data
    unsigned char usbBuf[65];

    int ping_data_count = 0;

    mFirstImuData = true;

    uint64_t rel_mcu_ts = 0;

    mSysTsQueue.reserve(TS_SHIFT_VAL_COUNT);
    mMcuTsQueue.reserve(TS_SHIFT_VAL_COUNT);

    while (!mStopCapture)
    {
        // ----> Keep data stream alive
        // sending a ping aboutonce per second
        // to keep the streaming alive
        if(ping_data_count>=400) {
            ping_data_count=0;
            sendPing();
        };
        ping_data_count++;
        // <---- Keep data stream alive

        mGrabRunning=true;

        // Sensor data request
        usbBuf[1]=usb::REP_ID_SENSOR_DATA;
        int res = hid_read_timeout( mDevHandle, usbBuf, 64, 2000 );

        // ----> Data received?
        if( res < static_cast<int>(sizeof(usb::RawData)) )  {
            hid_set_nonblocking( mDevHandle, 0 );
            continue;
        }
        // <---- Data received?

        // ----> Received data are correct?
        int target_struct_id = 0;
        if (mDevPid==SL_USB_PROD_MCU_ZED2_REVA || mDevPid==SL_USB_PROD_MCU_ZED2i_REVA)
            target_struct_id = usb::REP_ID_SENSOR_DATA;

        if( usbBuf[0] != target_struct_id)
        {
            if(mVerbose)
            {
                WARNING_OUT(mVerbose,std::string("REP_ID_SENSOR_DATA - Sensor Data type mismatch") );
            }

            hid_set_nonblocking( mDevHandle, 0 );
            continue;
        }
        // <---- Received data are correct?

        // Data structure static conversion
        usb::RawData* data = (usb::RawData*)usbBuf;

        // ----> Timestamp update
        uint64_t mcu_ts_nsec = static_cast<uint64_t>(std::round(static_cast<float>(data->timestamp)*TS_SCALE));

        if(mFirstImuData && data->imu_not_valid!=1)
        {
            mStartSysTs = getWallTimestamp(); // Starting system timestamp
            //std::cout << "SensorCapture: " << mStartSysTs << std::endl;

            mLastMcuTs = mcu_ts_nsec;
            mFirstImuData = false;
            continue;
        }

        uint64_t delta_mcu_ts_raw = mcu_ts_nsec - mLastMcuTs;

        //std::cout << "Internal MCU freq: " << 1e9/delta_mcu_ts_raw << " Hz" << std::endl;

        mLastMcuTs = mcu_ts_nsec;
        // <---- Timestamp update

        // Apply timestamp drift scaling factor
        rel_mcu_ts +=  static_cast<uint64_t>(static_cast<double>(delta_mcu_ts_raw)*mNTPTsScaling);

        // mStartSysTs is synchronized to Video TS when sync is enabled using \ref VideoCapture::enableSensorSync
        uint64_t current_data_ts = (mStartSysTs-mSyncOffset) + rel_mcu_ts;

        // ----> Camera/Sensors Synchronization
        if( data->sync_capabilities != 0 ) // Synchronization active
        {
            if(mLastFrameSyncCount!=0 && (data->frame_sync!=0 || data->frame_sync_count>mLastFrameSyncCount))
            {
#if 0 // Timestamp sync debug info
                std::cout << "MCU sync information: " << std::endl;
                std::cout << " * data->frame_sync: " << (int)data->frame_sync << std::endl;
                std::cout << " * data->frame_sync_count: " << data->frame_sync_count << std::endl;
                std::cout << " * mLastFrameSyncCount: " << mLastFrameSyncCount << std::endl;
                std::cout << " * MCU timestamp scaling: " << mNTPTsScaling << std::endl;
#endif
                mSysTsQueue.push_back( getSteadyTimestamp() );     // Steady host timestamp
                mMcuTsQueue.push_back( current_data_ts );   // MCU timestamp

                // Once we have enough data, calculate the drift scaling factor
                if (mSysTsQueue.size()==TS_SHIFT_VAL_COUNT && mMcuTsQueue.size() == TS_SHIFT_VAL_COUNT)
                {
                    //First and last ts
                    int first_index = 5;
                    if (mNTPAdjustedCount <= NTP_ADJUST_CT) {
                        first_index = TS_SHIFT_VAL_COUNT/2;
                    }

                    uint64_t first_ts_imu = mMcuTsQueue.at(first_index);
                    uint64_t last_ts_imu = mMcuTsQueue.at(mMcuTsQueue.size()-1);
                    uint64_t first_ts_cam = mSysTsQueue.at(first_index);
                    uint64_t last_ts_cam = mSysTsQueue.at(mSysTsQueue.size()-1);
                    double scale = double(last_ts_cam-first_ts_cam) / double(last_ts_imu-first_ts_imu);
                    //CLAMP
                    if (scale > 1.2) scale = 1.2;
                    if (scale < 0.8) scale = 0.8;

                    //Adjust scaling continuoulsy. No jump so that ts(n) - ts(n-1) == 400Hz
                    mNTPTsScaling*=scale;

                    //scale will be applied to the next values, so clear the vector and wait until we have enough data again
                    mMcuTsQueue.clear();
                    mSysTsQueue.clear();

                    // Count the number of completed time shift factor estimations
                    mNTPAdjustedCount++;

#ifdef VIDEO_MOD_AVAILABLE
                    // ----> Signal update offset to VideoCapture
                    if(mVideoPtr)
                    {
                        mSyncTs = current_data_ts;
                        mVideoPtr->setReadyToSync();
                    }
                    // <---- Update offset
#endif //VIDEO_MOD_AVAILABLE
                }
            }
        }
        mLastFrameSyncCount = data->frame_sync_count;
        // <---- Camera/Sensors Synchronization

        // ----> IMU data
        mIMUMutex.lock();
        mLastIMUData.sync = data->frame_sync;
        mLastIMUData.valid = (data->imu_not_valid!=1)?(data::Imu::NEW_VAL):(data::Imu::OLD_VAL);
        mLastIMUData.timestamp = current_data_ts;
        mLastIMUData.aX = data->aX*ACC_SCALE;
        mLastIMUData.aY = data->aY*ACC_SCALE;
        mLastIMUData.aZ = data->aZ*ACC_SCALE;
        mLastIMUData.gX = data->gX*GYRO_SCALE;
        mLastIMUData.gY = data->gY*GYRO_SCALE;
        mLastIMUData.gZ = data->gZ*GYRO_SCALE;
        mLastIMUData.temp = data->imu_temp*TEMP_SCALE;
        mNewIMUData = true;
        mIMUMutex.unlock();

        //std::string msg = std::to_string(mLastMAGData.timestamp);
        //INFO_OUT(msg);
        // <---- IMU data

        // ----> Magnetometer data
        if(data->mag_valid == data::Magnetometer::NEW_VAL)
        {
            mMagMutex.lock();
            mLastMagData.valid = data::Magnetometer::NEW_VAL;
            mLastMagData.timestamp = current_data_ts;
            mLastMagData.mY = data->mY*MAG_SCALE;
            mLastMagData.mZ = data->mZ*MAG_SCALE;
            mLastMagData.mX = data->mX*MAG_SCALE;
            mNewMagData = true;
            mMagMutex.unlock();

            //std::string msg = std::to_string(mLastMAGData.timestamp);
            //INFO_OUT(msg);
        }
        else
        {
            if(data->mag_valid==0)
                mLastMagData.valid = data::Magnetometer::NOT_PRESENT;
            else if(data->mag_valid==1)
                mLastMagData.valid = data::Magnetometer::OLD_VAL;
            else
                mLastMagData.valid = data::Magnetometer::NEW_VAL;
        }
        // <---- Magnetometer data

        // ----> Environmental data
        if(data->env_valid == data::Environment::NEW_VAL)
        {
            mEnvMutex.lock();
            mLastEnvData.valid = data::Environment::NEW_VAL;
            mLastEnvData.timestamp = current_data_ts;
            mLastEnvData.temp = data->temp*TEMP_SCALE;
            if( atLeast(mDevFwVer, ZED_2_FW::FW_3_9))
            {
                mLastEnvData.press = data->press*PRESS_SCALE_NEW;
                mLastEnvData.humid = data->humid*HUMID_SCALE_NEW;
            }
            else
            {
                mLastEnvData.press = data->press*PRESS_SCALE_OLD;
                mLastEnvData.humid = data->humid*HUMID_SCALE_OLD;
            }
            mNewEnvData = true;
            mEnvMutex.unlock();

            //std::string msg = std::to_string(mLastENVData.timestamp);
            //INFO_OUT(msg);
        }
        else
        {
            if(data->env_valid==0)
                mLastEnvData.valid = data::Environment::NOT_PRESENT;
            else if(data->env_valid==1)
                mLastEnvData.valid = data::Environment::OLD_VAL;
            else
                mLastEnvData.valid = data::Environment::NEW_VAL;
        }
        // <---- Environmental data

        // ----> Camera sensors temperature data
        if(data->temp_cam_left != TEMP_NOT_VALID &&
                data->temp_cam_left != TEMP_NOT_VALID &&
                data->env_valid == data::Environment::NEW_VAL ) // Sensor temperature is linked to Environmental data acquisition at FW level
        {
            mCamTempMutex.lock();
            mLastCamTempData.valid = data::Temperature::NEW_VAL;
            mLastCamTempData.timestamp = current_data_ts;
            mLastCamTempData.temp_left = data->temp_cam_left*TEMP_SCALE;
            mLastCamTempData.temp_right = data->temp_cam_right*TEMP_SCALE;
            mNewCamTempData=true;
            mCamTempMutex.unlock();

            //std::string msg = std::to_string(mLastCamTempData.timestamp);
            //INFO_OUT(msg);
        }
        else
        {
            mLastCamTempData.valid = data::Temperature::OLD_VAL;
        }
        // <---- Camera sensors temperature data
    }

    mGrabRunning = false;
}

#ifdef VIDEO_MOD_AVAILABLE
void SensorCapture::updateTimestampOffset( uint64_t frame_ts)
{
    static int64_t offset_sum = 0;
    static int count = 0;
    offset_sum += (static_cast<int64_t>(mSyncTs) - static_cast<int64_t>(frame_ts));
    count++;

    if(count==3)
    {
        int64_t offset = offset_sum/count;
        mSyncOffset += offset;
#if 0
        std::cout << "Offset: " << offset << std::endl;
        std::cout << "mSyncOffset: " << mSyncOffset << std::endl;
#endif

        offset_sum = 0;
        count=0;
    }
}
#endif

bool SensorCapture::sendPing() {
    if( !mDevHandle )
        return false;

    unsigned char buf[65];
    buf[0] = usb::REP_ID_REQUEST_SET;
    buf[1] = usb::RQ_CMD_PING;

    int res = hid_send_feature_report(mDevHandle, buf, 2);
    if (res < 0)
    {
        std::string msg = "Unable to send ping [REP_ID_REQUEST_SET-RQ_CMD_PING] - ";
        msg += wstr2str(hid_error(mDevHandle));

        WARNING_OUT(mVerbose,msg);

        return false;
    }

    return true;
}

bool SensorCapture::searchForConnectedDev(int* serial_number, unsigned short* found_pid)
{
    int in_serial_number;
    if(serial_number==nullptr)
         in_serial_number = 0;
    else
        in_serial_number = *serial_number;
    int found_serial_number = 0;

    // ----> Search for connected device
    struct hid_device_info *devs, *cur_dev;

    if (hid_init()==-1)
        return false;

    devs = hid_enumerate(SL_USB_VENDOR, 0x0);
    cur_dev = devs;

    bool found = false;
    uint16_t pid=0;
    std::string sn_str;

    while (cur_dev) {
        //int fw_major = cur_dev->release_number>>8;
        //int fw_minor = cur_dev->release_number&0x00FF;
        pid = cur_dev->product_id;
        sn_str = wstr2str( cur_dev->serial_number );
        int sn = std::stoi( sn_str );

        if(in_serial_number==0 || sn==in_serial_number)
        {
            if( pid==SL_USB_PROD_MCU_ZED2_REVA || pid==SL_USB_PROD_MCU_ZED2i_REVA)
            {
                found = true;
                found_serial_number = sn;
                break;
            }
            else
            {
                std::string msg = "The reset function works only for ZED2/ZED2i camera models.";
                std::cerr << msg << std::endl;

                if(in_serial_number==0)
                    continue;
                else
                    return false;
            }
        }

        cur_dev = cur_dev->next;
    }

    hid_free_enumeration(devs);
    // <---- Search for connected device

    if(!found) {
        return false;
    }

    if(serial_number)
        *serial_number = found_serial_number;
    if(found_pid)
        *found_pid = pid;
    return true;
}

bool SensorCapture::resetSensorModule(int serial_number)
{
    int found_sn = serial_number;
    unsigned short pid;
    bool res = searchForConnectedDev(&found_sn, &pid);
    if(!res)
    {
        std::string msg;
        if(serial_number!=0)
        {
            msg = "[sl_oc::sensors::SensorCapture] WARNING: Sensors Module reset failed. Unable to find the Sensor Module with serial number ";
            msg += std::to_string(serial_number);
        }
        else
        {
            msg = "[sl_oc::sensors::SensorCapture] WARNING: Sensors Module reset failed. Unable to find the Sensor Module of a ZED2 camera. Please verify the USB connection.";
        }

        std::cerr << msg << std::endl;

        return false;
    }

    std::string sn_str = std::to_string(found_sn);
    std::wstring wide_sn_string = std::wstring(sn_str.begin(), sn_str.end());
    const wchar_t* wsn = wide_sn_string.c_str();

    hid_device* devHandle = hid_open(SL_USB_VENDOR, pid, wsn );

    if(!devHandle)
    {
        std::string msg = "Unable to open the MCU HID device";
        std::cerr << msg << std::endl;

        return false;
    }

    unsigned char  buf[65];
    buf[0] = static_cast<unsigned char>(usb::REP_ID_REQUEST_SET);
    buf[1] = static_cast<unsigned char>(usb::RQ_CMD_RST);

    hid_send_feature_report(devHandle, buf, 2);
    // Note: cannot verify the return value of the `hid_send_feature_report` command because the MCU is suddenly reset
    // and it cannot return a valid value

    sleep(2); // Wait for MCU and OV580 to reboot

    std::cerr << "[sl_oc::sensors::SensorCapture] INFO: Sensors Module reset successful" << std::endl;

    return true;
}

bool SensorCapture::resetVideoModule(int serial_number)
{
    int found_sn = serial_number;
    unsigned short pid;
    bool res = searchForConnectedDev(&found_sn, &pid);
    if(!res)
    {
        std::string msg;
        if(serial_number!=0)
        {
            msg = "[sl_oc::sensors::SensorCapture] WARNING: Video Module reset failed. Unable to find the Sensor Module with serial number ";
            msg += std::to_string(serial_number);
        }
        else
        {
            msg = "[sl_oc::sensors::SensorCapture] WARNING: Video Module reset failed. Unable to find the Sensor Module of a ZED2 camera. Please verify the USB connection.";
        }

        std::cerr << msg << std::endl;

        return false;
    }

    std::string sn_str = std::to_string(found_sn);
    std::wstring wide_sn_string = std::wstring(sn_str.begin(), sn_str.end());
    const wchar_t* wsn = wide_sn_string.c_str();

    hid_device* devHandle = hid_open(SL_USB_VENDOR, pid, wsn );

    if(!devHandle)
    {
        std::string msg = "Unable to open the MCU HID device";
        std::cerr << msg << std::endl;

        return false;
    }

    usb::OV580CmdStruct cmd;
    cmd.struct_id = usb::REP_ID_OV580_CMD;
    cmd.cmd = usb::OV580_CMD_RESET;
    cmd.info=0;

    unsigned char buf[65];
    memcpy(buf, &(cmd.struct_id), sizeof(usb::OV580CmdStruct));

    int ret = hid_send_feature_report(devHandle, buf, sizeof(usb::OV580CmdStruct));
    hid_close(devHandle);

    if(ret!=sizeof(usb::OV580CmdStruct)) {
        std::cerr << "[sl_oc::sensors::SensorCapture] INFO: Video Module reset failed" << std::endl;
        return false;
    }

    sleep(2); // Wait for OV580 to reboot

    std::cerr << "[sl_oc::sensors::SensorCapture] INFO: Video Module reset successful" << std::endl;
    return true;
}

const data::Imu& SensorCapture::getLastIMUData(uint64_t timeout_usec)
{
    // ----> Wait for a new frame
    uint64_t time_count = (timeout_usec<100?100:timeout_usec)/100;
    while( !mNewIMUData )
    {
        if(time_count==0)
        {
            if(mLastIMUData.valid!=data::Imu::NOT_PRESENT)
                mLastIMUData.valid = data::Imu::OLD_VAL;
            return mLastIMUData;
        }
        time_count--;
        usleep(100);
    }
    // <---- Wait for a new frame

    // Get the frame mutex
    const std::lock_guard<std::mutex> lock(mIMUMutex);
    mNewIMUData = false;
    return mLastIMUData;
}

const data::Magnetometer& SensorCapture::getLastMagnetometerData(uint64_t timeout_usec)
{
    // ----> Wait for a new frame
    uint64_t time_count = (timeout_usec<100?100:timeout_usec)/10;
    while( !mNewMagData )
    {
        if(time_count==0)
        {
            if(mLastMagData.valid!=data::Magnetometer::NOT_PRESENT)
                mLastMagData.valid=data::Magnetometer::OLD_VAL;
            return mLastMagData;
        }
        time_count--;
        usleep(10);
    }
    // <---- Wait for a new frame

    // Get the frame mutex
    const std::lock_guard<std::mutex> lock(mMagMutex);
    mNewMagData = false;
    return mLastMagData;
}

const data::Environment &SensorCapture::getLastEnvironmentData(uint64_t timeout_usec)
{
    // ----> Wait for a new frame
    uint64_t time_count = (timeout_usec<100?100:timeout_usec)/10;
    while( !mNewEnvData )
    {
        if(time_count==0)
        {
            if(mLastEnvData.valid!=data::Environment::NOT_PRESENT)
                mLastEnvData.valid = data::Environment::OLD_VAL;
            return mLastEnvData;
        }
        time_count--;
        usleep(10);
    }
    // <---- Wait for a new frame

    // Get the frame mutex
    const std::lock_guard<std::mutex> lock(mEnvMutex);
    mNewEnvData = false;
    return mLastEnvData;
}

const data::Temperature& SensorCapture::getLastCameraTemperatureData(uint64_t timeout_usec)
{
    // ----> Wait for a new frame
    uint64_t time_count = (timeout_usec<100?100:timeout_usec)/10;
    while( !mNewCamTempData )
    {
        if(time_count==0)
        {
            if(mLastCamTempData.valid!=data::Temperature::NOT_PRESENT)
                mLastCamTempData.valid = data::Temperature::OLD_VAL;
            return mLastCamTempData;
        }
        time_count--;
        usleep(10);
    }
    // <---- Wait for a new frame

    // Get the frame mutex
    const std::lock_guard<std::mutex> lock(mCamTempMutex);
    mNewCamTempData = false;
    return mLastCamTempData;
}

}

}
