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

#include "videocapture.hpp"

#ifdef SENSORS_MOD_AVAILABLE
#include "sensorcapture.hpp"
#endif

#include <sys/stat.h>         // for stat, S_ISCHR
#include <errno.h>            // for errno, EBADRQC, EINVAL, ENOBUFS, ENOENT
#include <fcntl.h>            // for open, O_NONBLOCK, O_RDONLY, O_RDWR
#include <unistd.h>           // for usleep, close

#include <linux/usb/video.h>  // for UVC_GET_CUR, UVC_SET_CUR, UVC_GET_LEN
#include <linux/uvcvideo.h>   // for uvc_xu_control_query, UVCIOC_CTRL_QUERY
#include <linux/videodev2.h>  // for v4l2_buffer, v4l2_queryctrl, V4L2_BUF_T...
#include <sys/mman.h>         // for mmap, munmap, MAP_SHARED, PROT_READ
#include <sys/ioctl.h>        // for ioctl

#include <sstream>
#include <fstream>            // for char_traits, basic_istream::operator>>

#include <cmath>              // for round

#define IOCTL_RETRY 3

#define READ_MODE   1
#define WRITE_MODE  2

// ----> Camera control
#define cbs_xu_unit_id          0x04 //mapped to wIndex 0x0400
#define cbs_xu_control_selector 0x02 //mapped to wValue 0x0200

#define ISP_CTRL_LEFT    0x80181033
#define ISP_CTRL_RIGHT   0x80181833

#define AEG_AGC_MASK_ON     0x02
#define AEG_AGC_MASK_OFF    0xFD

#define ADDR_EXP_H   0x3500
#define ADDR_EXP_M   0x3501
#define ADDR_EXP_L   0x3502
#define ADDR_GAIN_H  0x3507
#define ADDR_GAIN_M  0x3508
#define ADDR_GAIN_L  0x3509

#define XU_TASK_SET     0x50
#define XU_TASK_GET     0x51
#define XU_ISP_CTRL     0x07
#define XU_EXP_GAIN     0x25

#define UNIQUE_ID_START 0x18000

#define LINUX_CTRL_BRIGHTNESS   9963776
#define LINUX_CTRL_CONTRAST     9963777
#define LINUX_CTRL_HUE          9963779
#define LINUX_CTRL_SATURATION   9963778
#define LINUX_CTRL_GAIN         9963795
#define LINUX_CTRL_AWB          9963802
#define LINUX_CTRL_AWB_AUTO     9963788
#define LINUX_CTRL_SHARPNESS    9963803
#define LINUX_CTRL_GAMMA        9963792

#define DEFAULT_GAMMA_NOECT 1
#define DEFAULT_MIN_GAMMA   1
#define DEFAULT_MAX_GAMMA   9

#define DEFAULT_MIN_GAIN   0
#define DEFAULT_MAX_GAIN   100
#define DEFAULT_MIN_EXP    0
#define DEFAULT_MAX_EXP    100

// Gain working zones
#define GAIN_ZONE1_MIN 0
#define GAIN_ZONE1_MAX 255
#define GAIN_ZONE2_MIN 378
#define GAIN_ZONE2_MAX 511
#define GAIN_ZONE3_MIN 890
#define GAIN_ZONE3_MAX 1023
#define GAIN_ZONE4_MIN 1914
#define GAIN_ZONE4_MAX 2047

// Raw exposure max values
#define EXP_RAW_MAX_15FPS   1550
#define EXP_RAW_MAX_30FPS   1100
#define EXP_RAW_MAX_60FPS   880
#define EXP_RAW_MAX_100FPS  720

#define EXP_RAW_MIN         2
// <---- Camera Control


namespace sl_oc {

namespace video {

VideoCapture::VideoCapture(VideoParams params)
{
    memcpy( &mParams, &params, sizeof(VideoParams) );

    if( mParams.verbose )
    {
        std::string ver =
                "ZED Open Capture - Camera module - Version: "
                + std::to_string(mMajorVer) + "."
                + std::to_string(mMinorVer) + "."
                + std::to_string(mPatchVer);
        INFO_OUT(mParams.verbose,ver);
    }

    // Check that FPS is coherent with user resolution
    checkResFps( );

    // Calculate gain zones (required because the raw gain control is not continuous in the range of values)
    mGainSegMax = (GAIN_ZONE4_MAX-GAIN_ZONE4_MIN)+(GAIN_ZONE3_MAX-GAIN_ZONE3_MIN)+(GAIN_ZONE2_MAX-GAIN_ZONE2_MIN)+(GAIN_ZONE1_MAX-GAIN_ZONE1_MIN);

    // FPS mapping
    if( mFps <= 15 )
        mExpoureRawMax = EXP_RAW_MAX_15FPS;
    else if( mFps <= 30 )
        mExpoureRawMax = EXP_RAW_MAX_30FPS;
    else if( mFps <= 60 )
        mExpoureRawMax = EXP_RAW_MAX_60FPS;
    else
        mExpoureRawMax = EXP_RAW_MAX_100FPS;
}

VideoCapture::~VideoCapture()
{
    reset();
}

void VideoCapture::reset()
{
    setLEDstatus( false );

    mStopCapture = true;

    if( mGrabThread.joinable() )
    {
        mGrabThread.join();
    }

    // ----> Stop capturing
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (mFileDesc != -1)
        xioctl(mFileDesc, VIDIOC_STREAMOFF, &type);
    // <---- Stop capturing

    // ----> deinit device
    if( mInitialized && mBuffers)
    {
        for (unsigned int i = 0; i < mBufCount; ++i)
            munmap(mBuffers[i].start, mBuffers[i].length);
        if (mBuffers)
            free(mBuffers);

        mBuffers = nullptr;
    }
    // <---- deinit device

    if (mFileDesc)
    {
        close(mFileDesc);
        mFileDesc=-1;
    }

    if(mLastFrame.data)
    {
        delete [] mLastFrame.data;
        mLastFrame.data = nullptr;
    }

    if( mParams.verbose && mInitialized)
    {
        std::string msg = "Device closed";
        INFO_OUT(mParams.verbose,msg );
    }

    mInitialized=false;
}

void VideoCapture::checkResFps()
{
    mWidth = cameraResolution[static_cast<int>(mParams.res)].width*2;
    mHeight = cameraResolution[static_cast<int>(mParams.res)].height;
    mFps = static_cast<int>(mParams.fps);

    switch (mParams.res)
    {
    default:
        WARNING_OUT(mParams.verbose,"RESOLUTION not supported. Using the best value");

    case RESOLUTION::HD2K:
        if( mFps!=15 )
        {
            WARNING_OUT(mParams.verbose,"FPS not supported for the chosen resolution. Using the best value");
            mFps = 15;
        }
        break;

    case RESOLUTION::HD1080:
        if( mFps!=15 && mFps!=30 )
        {
            WARNING_OUT(mParams.verbose,"FPS not supported for the chosen resolution. Using the best value");

            if( mFps <= 22  )
                mFps = 15;
            else
                mFps = 30;
        }
        break;

    case RESOLUTION::HD720:
        if( mFps!=15 && mFps!=30 && mFps!=60 )
        {
            WARNING_OUT(mParams.verbose,"FPS not supported for the chosen resolution. Using the best value");

            if( mFps <= 22  )
                mFps = 15;
            else if( mFps < 45  )
                mFps = 30;
            else
                mFps = 60;
        }
        break;

    case RESOLUTION::VGA:
        if( mFps!=15 && mFps!=30 && mFps!=60 && mFps!=100)
        {
            WARNING_OUT(mParams.verbose,"FPS not supported for the chosen resolution. Using the best value");

            if( mFps <= 22  )
                mFps = 15;
            else if( mFps < 45  )
                mFps = 30;
            else if( mFps < 80  )
                mFps = 60;
            else
                mFps = 100;
        }
    }

    if(mParams.verbose)
    {
        std::string msg = std::string("Camera resolution: ")
                + std::to_string(mWidth)
                + std::string("x")
                + std::to_string(mHeight)
                + std::string("@")
                + std::to_string(mFps)
                +std::string("Hz");

        INFO_OUT(mParams.verbose,msg);
    }
}

bool VideoCapture::initializeVideo( int devId/*=-1*/ )
{
    reset();

    bool opened=false;

    if( devId==-1 )
    {
        // Try to open all the devices until the first success (max allowed by v4l: 64)
        for( uint8_t id=0; id<64; id++ )
        {
            opened = openCamera( id );
            if(opened) break;
        }
    }
    else
    {
        opened = openCamera( static_cast<uint8_t>(devId) );
    }

    if(!opened)
    {
        return false;
    }

    mInitialized = startCapture();

    if( mParams.verbose && mInitialized)
    {
        std::string msg = "Device '" + mDevName + "' opened";
        INFO_OUT(mParams.verbose,msg );
    }

    setLEDstatus( true );

    resetAECAGC();
    resetAutoWhiteBalance();
    resetBrightness();
    resetContrast();
    resetGamma();
    resetHue();
    resetSaturation();
    resetSharpness();

    return mInitialized;
}

bool VideoCapture::openCamera( uint8_t devId )
{
    mDevId = devId;

    mDevName = std::string("/dev/video") + std::to_string(mDevId);

    if( mParams.verbose )
    {
        std::string msg = "Trying to open the device '" + mDevName + "'";
        INFO_OUT(mParams.verbose,msg );
    }

    // Check camera model
    mCameraModel = getCameraModel(mDevName);

    if( mCameraModel==SL_DEVICE::NONE )
    {
        if(mParams.verbose)
        {
            std::string msg = "The device '" + mDevName + "' is not a Stereolabs camera";
            WARNING_OUT(mParams.verbose,msg );
        }
        return false;
    }

    if( mCameraModel==SL_DEVICE::ZED ||
            mCameraModel==SL_DEVICE::ZED_M )
    {
        if(mParams.verbose)
        {
            std::string msg = "The FW of the device '" + mDevName + "' is not supported. Please update it.";
            ERROR_OUT(mParams.verbose,msg );
        }
        return false;
    }

    // ----> Open
    struct stat st;
    memset(&st, 0, sizeof (struct stat));
    if (-1 == stat(mDevName.c_str(), &st))
    {
        if(mParams.verbose)
        {
            std::string msg = std::string("Cannot identify '") + mDevName + "': ["
                    + std::to_string(errno) +std::string("] ") + std::string(strerror(errno));
            ERROR_OUT(mParams.verbose,msg);

            return false;
        }
    }

    if (!S_ISCHR(st.st_mode))
    {
        if(mParams.verbose)
        {
            std::string msg = mDevName + std::string(" is no device");
            ERROR_OUT(mParams.verbose,msg);

            return false;
        }
    }

    mFileDesc = 0;

    mFileDesc = open(mDevName.c_str(), O_RDWR|O_NONBLOCK,0); // Reading are non blocking

    if (-1 == mFileDesc)
    {
        if(mParams.verbose)
        {
            std::string msg = std::string("Cannot open '") + mDevName + "': ["
                    + std::to_string(errno) + std::string("] ") + std::string(strerror(errno));
            ERROR_OUT(mParams.verbose,msg);
        }

        return false;
    }
    // <---- Open

    int sn = getSerialNumber();
    if(mParams.verbose)
    {
        std::string msg = std::string("Opened camera with SN: ") + std::to_string(sn);
        INFO_OUT(mParams.verbose,msg);
    }

    // ----> Init
    struct v4l2_capability cap;
    memset(&cap, 0, sizeof (v4l2_capability));
    struct v4l2_cropcap cropcap;
    memset(&cropcap, 0, sizeof (v4l2_cropcap));
    struct v4l2_crop crop;
    memset(&crop, 0, sizeof (v4l2_crop));
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof (v4l2_format));

    if( -1==xioctl(mFileDesc, VIDIOC_QUERYCAP, &cap) )
    {
        if(mParams.verbose)
        {
            std::string msg = std::string("Cannot query capabilities of '") + mDevName + "': ["
                    + std::to_string(errno) +std::string("] ") + std::string(strerror(errno));
            ERROR_OUT(mParams.verbose,msg);
        }

        return false;
    }

    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    // Query the Capture
    if (0 == xioctl(mFileDesc, VIDIOC_CROPCAP, &cropcap))
    {
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect; /* reset to default */
    }

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field = V4L2_FIELD_ANY;

    if (mWidth * mHeight != 0)
    {
        fmt.fmt.pix.width = mWidth;
        fmt.fmt.pix.height = mHeight;
    }

    int width_tmp = mWidth;
    int height_tmp = mHeight;

    /* Preserve original settings as set by v4l2-ctl for example */
    if( -1==xioctl(mFileDesc, VIDIOC_S_FMT/*VIDIOC_G_FMT*/, &fmt) )
    {
        if(mParams.verbose)
        {
            std::string msg = std::string("Cannot set pixel format of '") + mDevName + "': ["
                    + std::to_string(errno) +std::string("] ") + std::string(strerror(errno));
            ERROR_OUT(mParams.verbose,msg);
        }

        return false;
    }

    mWidth = fmt.fmt.pix.width;
    mHeight = fmt.fmt.pix.height;
    mChannels = fmt.fmt.pix.bytesperline / mWidth;

    // Asked resolution not available, exiting
    if (mWidth != width_tmp || mHeight != height_tmp)
    {
        ERROR_OUT(mParams.verbose,"Error setting the camera resolution");
        return false;
    }

    if( -1==input_set_framerate(mFps) )
    {
        ERROR_OUT(mParams.verbose,"Error setting the camera framerate");
    }

    // ----> Output frame allocation
    mLastFrame.width = mWidth;
    mLastFrame.height = mHeight;
    mLastFrame.channels = mChannels;
    int bufSize = mLastFrame.width * mLastFrame.height * mLastFrame.channels;
    mLastFrame.data = new unsigned char[bufSize];
    // <---- Output frame allocation

    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof (v4l2_requestbuffers));

    req.count = mBufCount;

    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if( -1==xioctl(mFileDesc, VIDIOC_REQBUFS, &req) )
    {
        if(mParams.verbose)
        {
            std::string msg = std::string("Cannot request buffers for '") + mDevName + "': ["
                    + std::to_string(errno) +std::string("] ") + std::string(strerror(errno));
            ERROR_OUT(mParams.verbose,msg);
        }

        return false;
    }

    // Create buffers
    mBuffers = (UVCBuffer*) calloc(req.count, sizeof(*mBuffers));

    for(mBufCount = 0; mBufCount < req.count; ++mBufCount)
    {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof (v4l2_buffer));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = mBufCount;
        buf.flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
        if( -1==xioctl(mFileDesc, VIDIOC_QUERYBUF, &buf))
        {
            if(mParams.verbose)
            {
                std::string msg = std::string("Cannot query buffer for '") + mDevName + "': ["
                        + std::to_string(errno) +std::string("] ") + std::string(strerror(errno));
                ERROR_OUT(mParams.verbose,msg);
            }

            return false;
        }

        mBuffers[mBufCount].length = buf.length;

        mBuffers[mBufCount].start =
                mmap(nullptr /* start anywhere */,
                     buf.length,
                     PROT_READ | PROT_WRITE /* required */,
                     MAP_SHARED /* recommended */,
                     mFileDesc, buf.m.offset);
    }

    mBufCount = req.count;
    // <---- Init

    return true;
}

int VideoCapture::getSerialNumber()
{
    /*if(!mInitialized)
        return -1;*/

    int ulValue = -1;

    uint8_t UNIQUE_BUF[384];
    memset(UNIQUE_BUF, 0, 384);

    int res = -1;

    int try_count = 0;
    while(res!=0)
    {
        res = ll_SPI_FlashProgramRead(&UNIQUE_BUF[0], UNIQUE_ID_START, 64, true);

        //check bytes read
        if (UNIQUE_BUF[0] != 'O') {
            res =  -1;
        }
        if (UNIQUE_BUF[1] != 'V') {
            res =  -1;
        }

        try_count++;
        if (try_count>500)
            break;
        usleep(1000);
    }

    if (res != 0)
        return -1;

    //check bytes read
    if (UNIQUE_BUF[0] != 'O') {
        return -1;
    }
    if (UNIQUE_BUF[1] != 'V') {
        return -1;
    }

    ulValue = (UNIQUE_BUF[2] << 24) + (UNIQUE_BUF[3] << 16) + (UNIQUE_BUF[4] << 8) + UNIQUE_BUF[5];

    char buff[128];
    memset(buff, 0, 128);
    sprintf(buff, "%x", ulValue);
    return (int) atoi(buff);
}

bool VideoCapture::startCapture()
{
    // ----> Start capturing
    enum v4l2_buf_type type;
    for (unsigned int i = 0; i < mBufCount; ++i)
    {
        struct v4l2_buffer buf = {0};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        if( -1==xioctl(mFileDesc, VIDIOC_QBUF, &buf) )
        {
            if(mParams.verbose)
            {
                std::string msg = std::string("Cannot queue buffer for '") + mDevName + "': ["
                        + std::to_string(errno) +std::string("] ") + std::string(strerror(errno));
                ERROR_OUT(mParams.verbose,msg);
            }

            return false;
        }
    }
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    // Set priority
    int priority = V4L2_PRIORITY_RECORD;
    if( -1==xioctl(mFileDesc, VIDIOC_G_PRIORITY, &priority) )
    {
        if(mParams.verbose)
        {
            std::string msg = std::string("Cannot set priority for '") + mDevName + "': ["
                    + std::to_string(errno) +std::string("] ") + std::string(strerror(errno));
            ERROR_OUT(mParams.verbose,msg);
        }

        return false;
    }

    // Start streaming
    if( -1==xioctl(mFileDesc, VIDIOC_STREAMON, &type) )
    {
        if(mParams.verbose)
        {
            std::string msg = std::string("Cannot start streaming for '") + mDevName + "': ["
                    + std::to_string(errno) +std::string("] ") + std::string(strerror(errno));
            ERROR_OUT(mParams.verbose,msg);
        }

        return false;
    }
    // <---- Start capturing

    mGrabThread = std::thread( &VideoCapture::grabThreadFunc,this );

    return true;
}

int VideoCapture::input_set_framerate(int fps)
{
    struct v4l2_streamparm streamparm = {0}; // v4l2 stream parameters struct

    streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    streamparm.parm.capture.capturemode |= V4L2_CAP_TIMEPERFRAME;
    streamparm.parm.capture.timeperframe.numerator = 1;
    streamparm.parm.capture.timeperframe.denominator = fps;

    return xioctl(mFileDesc, VIDIOC_S_PARM, &streamparm);
}

int VideoCapture::xioctl(int fd, uint64_t IOCTL_X, void *arg)
{
    int ret = 0;
    int tries = IOCTL_RETRY;
    do
    {
        ret = ioctl(fd, IOCTL_X, arg);
        // usleep(1);
    } while (ret && tries-- &&
             ((errno == EINTR) || (errno == EAGAIN) || (errno == ETIMEDOUT)));

    if( ret==-1 )
    {
        if(mParams.verbose)
        {
            perror( "xioctl");
        }
    }

    return (ret);
}

SL_DEVICE VideoCapture::getCameraModel( std::string dev_name)
{
    sl_oc::video::SL_DEVICE camera_device = sl_oc::video::SL_DEVICE::NONE;
    int vid = 0, pid = 0;
    std::string modalias = "";
    std::string name = dev_name.erase(0, 5); //remove /dev/
    if (!(std::ifstream("/sys/class/video4linux/" + name + "/device/modalias") >> modalias))
    {
        if(mParams.verbose>sl_oc::VERBOSITY::ERROR)
        {
            std::string msg =
                    std::string(" Not a modalias : /sys/class/video4linux/")
                    + name + std::string("/device/modalias");
            WARNING_OUT(mParams.verbose,msg);
        }
        return camera_device;
    }

    if (modalias.size() < 14 || modalias.substr(0, 5) != "usb:v" || modalias[9] != 'p')
    {
        if(mParams.verbose)
        {
            std::string msg = std::string(" not a modalias 2" );
            WARNING_OUT(mParams.verbose,msg);
        }
        return camera_device;
    }

    if (!(std::istringstream(modalias.substr(5, 4)) >> std::hex >> vid))
    {
        if(mParams.verbose)
        {
            std::string msg = std::string("unable to read Vendor ID" );
            WARNING_OUT(mParams.verbose,msg);
        }


        return camera_device;
    }

    if (!(std::istringstream(modalias.substr(10, 4)) >> std::hex >> pid))
    {
        if(mParams.verbose)
        {
            std::string msg = std::string("unable to read Product ID" );
            WARNING_OUT(mParams.verbose,msg);
        }

        return camera_device;
    }

    // check PID VID
    if (pid == SL_USB_PROD_ZED_REVA && vid == SL_USB_VENDOR)
        camera_device = SL_DEVICE::ZED;
    else if (pid == SL_USB_PROD_ZED_M_REVA && vid == SL_USB_VENDOR)
        camera_device = SL_DEVICE::ZED_M;
    else if (pid == SL_USB_PROD_ZED_REVB && vid == SL_USB_VENDOR)
        camera_device = SL_DEVICE::ZED_CBS;
    else if (pid == SL_USB_PROD_ZED_M_REVB && vid == SL_USB_VENDOR)
        camera_device = SL_DEVICE::ZED_M_CBS;
    else if (pid == SL_USB_PROD_ZED_2_REVB && vid == SL_USB_VENDOR)
        camera_device = SL_DEVICE::ZED_2;
    else if (pid == SL_USB_PROD_ZED_2i && vid == SL_USB_VENDOR)
        camera_device = SL_DEVICE::ZED_2i;

    return camera_device;
}

void VideoCapture::grabThreadFunc()
{
    mNewFrame = false;
    mStopCapture = false;

    fd_set fds;
    struct timeval tv = {0};

    FD_ZERO(&fds);

    if (mFileDesc < 0)
        return;

    FD_SET(mFileDesc, &fds);
    tv.tv_sec = 2;
    tv.tv_usec = 0;
    select(mFileDesc + 1, &fds, nullptr, nullptr, &tv);

    struct v4l2_buffer buf;
    memset(&(buf), 0, sizeof (buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.bytesused = -1;
    buf.length = 0;

    uint64_t rel_ts = 0;
    int capture_frame_count = 0;

    mFirstFrame=true;

    while (!mStopCapture)
    {
        mGrabRunning=true;

        mComMutex.lock();
        int ret = ioctl(mFileDesc, VIDIOC_DQBUF, &buf);
        mComMutex.unlock();

        if (buf.bytesused == buf.length && ret == 0 && buf.index < mBufCount)
        {
            mCurrentIndex = buf.index;
            // get buffer timestamp in us

            uint64_t ts_uvc = ((uint64_t) buf.timestamp.tv_sec) * (1000 * 1000) + ((uint64_t) buf.timestamp.tv_usec);

            if(mFirstFrame)
            {
                mStartTs = getWallTimestamp();
                //std::cout << "VideoCapture: " << mStartTs << std::endl;

#ifdef SENSORS_MOD_AVAILABLE
                if(mSyncEnabled && mSensPtr)
                {
                    // Synchronize reference timestamp
                    mSensPtr->setStartTimestamp(mStartTs);
                }
#endif

                mFirstFrame = false;
                mInitTs = ts_uvc;
            }

            rel_ts = ts_uvc - mInitTs;
            // cvt to ns
            rel_ts *= 1000;

            mBufMutex.lock();
            if (mLastFrame.data != nullptr && mWidth != 0 && mHeight != 0 && mBuffers[mCurrentIndex].start != nullptr)
            {
                mLastFrame.frame_id++;
                memcpy(mLastFrame.data, (unsigned char*) mBuffers[mCurrentIndex].start, mBuffers[mCurrentIndex].length);
                mLastFrame.timestamp = mStartTs + rel_ts;

                //                static uint64_t last_ts=0;
                //                std::cout << "[Video] Frame TS: " << static_cast<double>(mLastFrame.timestamp)/1e9 << " sec" << std::endl;
                //                double dT = static_cast<double>(mLastFrame.timestamp-last_ts)/1e9;
                //                last_ts = mLastFrame.timestamp;
                //                std::cout << "[Video] Frame FPS: " << 1./dT << std::endl;

#ifdef SENSORS_MOD_AVAILABLE
                if(mSensReadyToSync)
                {
                    mSensReadyToSync = false;
                    mSensPtr->updateTimestampOffset(mLastFrame.timestamp);
                }
#endif

#ifdef SENSOR_LOG_AVAILABLE
                // ----> AEC/AGC register logging
                if(mLogEnable)
                {
                    static int frame_count =0;


                    if((++frame_count)==mLogFrameSkip)
                        frame_count = 0;

                    if(frame_count==0)
                    {
                        saveLogDataLeft();
                        saveLogDataRight();
                    }
                }
                // <---- AEC/AGC register logging
#endif

                mNewFrame=true;
            }
            mBufMutex.unlock();

            mComMutex.lock();
            ioctl(mFileDesc, VIDIOC_QBUF, &buf);
            mComMutex.unlock();

            capture_frame_count++;
        }
        else
        {
            if (buf.bytesused != buf.length)
            {
                mComMutex.lock();
                ioctl(mFileDesc, VIDIOC_QBUF, &buf);
                mComMutex.unlock();
            }
            usleep(200);
            buf.bytesused = -1;
            buf.length = 0;
        }
    }

    mGrabRunning = false;
}

const Frame& VideoCapture::getLastFrame( uint64_t timeout_msec )
{
    // ----> Wait for a new frame
    uint64_t time_count = timeout_msec*10;
    while( !mNewFrame )
    {
        if(time_count==0)
        {
            return mLastFrame;
        }
        time_count--;
        usleep(100);
    }
    // <---- Wait for a new frame

    // Get the frame mutex
    const std::lock_guard<std::mutex> lock(mBufMutex);
    mNewFrame = false;
    return mLastFrame;
}

int VideoCapture::ll_VendorControl(uint8_t *buf, int len, int readMode, bool safe, bool force)
{
    if (len > 384)
        return -2;

    if (!force && !mInitialized)
        return -3;

    unsigned char tmp[2] = {0};
    struct uvc_xu_control_query xu_query_info;
    xu_query_info.unit = cbs_xu_unit_id;
    xu_query_info.selector = cbs_xu_control_selector;
    xu_query_info.query = UVC_GET_LEN;
    xu_query_info.size = 2;
    xu_query_info.data = tmp;

    const std::lock_guard<std::mutex> lock(mComMutex);

    int io_err = ioctl(mFileDesc, UVCIOC_CTRL_QUERY, &xu_query_info);

    //std::cerr << "[ll_VendorControl] '" << mDevName << "' [" << mDevId << "] - mFileDesc: " << mFileDesc << std::endl;

    if (io_err != 0)
    {
        return -4;
    }
    else
    {
        len = (xu_query_info.data[1] << 8) + xu_query_info.data[0];
    }

    //len should be now 384 for USB3 and 64 for USB2
    // we use the UVC_SET_CUR to write the cmd
    struct uvc_xu_control_query xu_query_send;
    xu_query_send.unit = cbs_xu_unit_id;
    xu_query_send.selector = cbs_xu_control_selector;
    xu_query_send.query = UVC_SET_CUR;
    xu_query_send.size = static_cast<__u16> (len); //64 for USB2
    xu_query_send.data = buf;

    io_err = ioctl(mFileDesc, UVCIOC_CTRL_QUERY, &xu_query_send);
    if (io_err != 0)
    {
        int res = errno;

        const char *err=nullptr;
        switch (res) {
        case ENOENT:
            err = "Extension unit or control not found";
            break;
        case ENOBUFS:
            err = "Buffer size does not match control size";
            break;
        case EINVAL:
            err = "Invalid request code";
            break;
        case EBADRQC:
            err = "Request not supported by control";
            break;
        default:
            err = strerror(res);
            break;
        }

        if(mParams.verbose)
        {
            std::string msg = std::string("CBS SET failed") +
                    std::string(err) +
                    std::string(". (System code: ") +
                    std::to_string(res) +
                    std::string(") ") +
                    std::to_string(xu_query_send.size);
            ERROR_OUT(mParams.verbose,msg);
        }

        return -1;
    }

#if 1
    if (!safe)
        usleep(300);
    else
        usleep(2000);

    if (readMode == READ_MODE) {

        struct uvc_xu_control_query xu_query;
        xu_query.unit = cbs_xu_unit_id,
                xu_query.selector = cbs_xu_control_selector,
                xu_query.query = UVC_GET_CUR,
                xu_query.size = static_cast<__u16> (len),
                xu_query.data = buf;

        io_err = ioctl(mFileDesc, UVCIOC_CTRL_QUERY, &xu_query);
        if (io_err != 0) {
            int res = errno;

            const char *err;
            switch (res) {
            case ENOENT: err = "Extension unit or control not found";
                break;
            case ENOBUFS: err = "Buffer size does not match control size";
                break;
            case EINVAL: err = "Invalid request code";
                break;
            case EBADRQC: err = "Request not supported by control";
                break;
            default: err = strerror(res);
                break;
            }

            if(mParams.verbose)
            {
                std::string msg = std::string("CBS GET failed") +
                        std::string(err) +
                        std::string(". (System code: ") +
                        std::to_string(res) +
                        std::string(") ") +
                        std::to_string(xu_query_send.size);
                ERROR_OUT(mParams.verbose,msg);
            }
            return -1;
        }

        return 0;
    }
    else
    {
        return 0;
    }

#endif


}

/**
 * @brief cbs_set_gpio_value
 * @param gpio_number (0 to 4)
 * @param value(0x00: low, 0x01 : high)
 * @return 0 if success
 */
int VideoCapture::ll_get_gpio_value(int gpio_number, uint8_t *value)
{
    unsigned char xu_buf[384];
    memset(xu_buf, 0, 384);

    //Set xubuf
    xu_buf[0] = XU_TASK_GET;
    xu_buf[1] = 0x13;
    xu_buf[2] = gpio_number;

    int hr = ll_VendorControl(xu_buf, 384, 1);
    *value = xu_buf[17];
    return hr;
}

/**
 * @brief cbs_set_gpio_value
 * @param gpio_number (0 to 4)
 * @param value(0x00: low, 0x01 : high)
 * @return 0 if success
 */
int VideoCapture::ll_set_gpio_value(int gpio_number, uint8_t value)
{
    unsigned char xu_buf[64];
    memset(xu_buf, 0, 64);

    //Set xubuf
    xu_buf[0] = XU_TASK_SET;
    xu_buf[1] = 0x12;
    xu_buf[2] = gpio_number;
    xu_buf[3] = value;

    int hr = ll_VendorControl(xu_buf, 64, 0);
    return hr;
}

/**
 * @brief cbs_set_gpio_direction
 * @param gpio_number (0 to 4)
 * @param direction (0x00 output, 0x01 input)
 * @return  0 if success
 */
int VideoCapture::ll_set_gpio_direction(int gpio_number, int direction)
{
    unsigned char xu_buf[64];
    memset(xu_buf, 0, 64);

    //Set xubuf
    xu_buf[0] = XU_TASK_SET;
    xu_buf[1] = 0x10;
    xu_buf[2] = gpio_number;
    xu_buf[3] = direction;

    int hr = ll_VendorControl(xu_buf, 64, 0);
    return hr;
}

int VideoCapture::ll_read_system_register(uint64_t address, uint8_t *value)
{
    unsigned char xu_buf[384];
    memset(xu_buf, 0, 384);

    //Set xubuf
    xu_buf[0] = XU_TASK_GET;
    xu_buf[1] = 0xA2;
    xu_buf[2] = 0;
    xu_buf[3] = 0x04; //Address width in bytes
    xu_buf[4] = 0x01; //data width in bytes

    xu_buf[5] = ((address) >> 24) & 0xff;
    xu_buf[6] = ((address) >> 16) & 0xff;
    xu_buf[7] = ((address) >> 8) & 0xff;
    xu_buf[8] = (address) & 0xff;
    xu_buf[9] = 0x00;
    xu_buf[10] = 0x01;
    xu_buf[11] = 0x00;
    xu_buf[12] = 0x01;

    int hr = ll_VendorControl(xu_buf, 384, READ_MODE);
    *value = xu_buf[17];
    return hr;
}

int VideoCapture::ll_write_system_register(uint64_t address, uint8_t value)
{
    unsigned char xu_buf[384];
    memset(xu_buf, 0, 384);

    //Set xubuf
    xu_buf[0] = XU_TASK_SET;
    xu_buf[1] = 0xA2;
    xu_buf[2] = 0;
    xu_buf[3] = 0x04; //Address width in bytes
    xu_buf[4] = 0x01; //data width in bytes

    xu_buf[5] = ((address) >> 24) & 0xff;
    xu_buf[6] = ((address) >> 16) & 0xff;
    xu_buf[7] = ((address) >> 8) & 0xff;
    xu_buf[8] = (address) & 0xff;
    xu_buf[9] = 0x00;
    xu_buf[10] = 0x01;
    xu_buf[11] = 0x00;
    xu_buf[12] = 0x01;
    xu_buf[16] = value;

    int hr = ll_VendorControl(xu_buf, 384, 0);
    return hr;
}

#define ASIC_INT_NULL_I2C    0xa3
#define ASIC_INT_I2C         0xa5

int VideoCapture::ll_read_sensor_register(int side, int sscb_id, uint64_t address, uint8_t* value)
{
    unsigned char xu_buf[384];
    memset(xu_buf, 0, 384);


    //Set xubuf
    xu_buf[0] = XU_TASK_GET;
    if (side == 0)
        xu_buf[1] = ASIC_INT_NULL_I2C;
    else
        xu_buf[1] = ASIC_INT_I2C;
    xu_buf[2] = 0x6c;
    xu_buf[3] = sscb_id + 1; //Address width in bytes
    xu_buf[4] = 0x01; //data width in bytes

    xu_buf[5] = ((address) >> 24) & 0xff;
    xu_buf[6] = ((address) >> 16) & 0xff;
    xu_buf[7] = ((address) >> 8) & 0xff;
    xu_buf[8] = (address) & 0xff;

    xu_buf[9] = 0x00;
    xu_buf[10] = 0x01;
    xu_buf[11] = 0x00;
    xu_buf[12] = 0x01;

    int limit = 1;

    xu_buf[9] = (limit >> 8) & 0xff;
    xu_buf[10] = (limit >> 0) & 0xff;

    //set page addr
    xu_buf[9] = xu_buf[9] & 0x0f;
    xu_buf[9] = xu_buf[9] | 0x10;
    xu_buf[9] = xu_buf[9] | 0x80;

    int hr = ll_VendorControl(xu_buf, 384, READ_MODE);
    *value = xu_buf[17];
    return hr;
}

int VideoCapture::ll_write_sensor_register(int side, int sscb_id, uint64_t address, uint8_t value)
{

    unsigned char xu_buf[384];
    memset(xu_buf, 0, 384);

    //Set xubuf
    xu_buf[0] = XU_TASK_SET;
    if (side == 0)
        xu_buf[1] = ASIC_INT_NULL_I2C;
    else
        xu_buf[1] = ASIC_INT_I2C;
    xu_buf[2] = 0x6c;
    xu_buf[3] = sscb_id + 1; //Address width in bytes
    xu_buf[4] = 0x01; //data width in bytes

    xu_buf[5] = ((address) >> 24) & 0xff;
    xu_buf[6] = ((address) >> 16) & 0xff;
    xu_buf[7] = ((address) >> 8) & 0xff;
    xu_buf[8] = (address) & 0xff;
    xu_buf[9] = 0x00;
    xu_buf[10] = 0x01;
    xu_buf[11] = 0x00;
    xu_buf[12] = 0x01;
    xu_buf[16] = value;


    xu_buf[9] = 0x00;
    xu_buf[10] = 0x01;
    xu_buf[11] = 0x00;
    xu_buf[12] = 0x01;

    int limit = 1;
    xu_buf[9] = (limit >> 8) & 0xff;
    xu_buf[10] = (limit >> 0) & 0xff;

    //set page addr
    xu_buf[9] = xu_buf[9] & 0x0f;
    xu_buf[9] = xu_buf[9] | 0x10;
    xu_buf[9] = xu_buf[9] | 0x80;
    memcpy(&xu_buf[16], &value, sizeof (uint8_t));
    int hr = ll_VendorControl(xu_buf, 384, 0);

    return hr;
}

int VideoCapture::ll_SPI_FlashProgramRead(uint8_t *pBuf, int Adr, int len, bool force) {

    int hr = -1;
    uint8_t xu_buf[384];
    memset(xu_buf, 0, 384);

    xu_buf[0] = 0x51;
    xu_buf[1] = 0xA1;
    xu_buf[2] = 0x03;

    xu_buf[5] = (Adr >> 24) & 0xff;
    xu_buf[6] = (Adr >> 16) & 0xff;
    xu_buf[7] = (Adr >> 8) & 0xff;
    xu_buf[8] = (Adr) & 0xff;

    int pack_val = 36864 + len;
    xu_buf[9] = ((pack_val) >> 8) & 0xff;
    xu_buf[10] = ((pack_val) >> 0) & 0xff;

    xu_buf[11] = ((len) >> 8) & 0xff;
    xu_buf[12] = ((len) >> 0) & 0xff;

    hr = ll_VendorControl(xu_buf, len, 1, true, force);
    memcpy(pBuf, &xu_buf[17], len);
    return hr;
}

int VideoCapture::ll_isp_aecagc_enable(int side, bool enable) {
    uint64_t Address = 0;
    uint8_t value = 0;
    int hr = 0;
    if (side == 0)
        Address = ISP_CTRL_LEFT; //ISP L
    else if (side == 1)
        Address = ISP_CTRL_RIGHT; //ISP R
    else
        return -2;

    //Read Current sysregister
    hr += ll_read_system_register(Address, &value);

    // Adjust Value
    if (enable)
        value = value | AEG_AGC_MASK_ON;
    else
        value = value & AEG_AGC_MASK_OFF;

    hr += ll_write_system_register(Address, value);

    return hr;
}


int VideoCapture::ll_isp_is_aecagc(int side) {
    uint64_t Address = 0;
    uint8_t value = 0;
    int result = 0;
    int hr = 0;
    if (side == 0)
        Address = ISP_CTRL_LEFT; //ISP L
    else if (side == 1)
        Address = ISP_CTRL_RIGHT; //ISP R
    else
        return -2;

    //Read Current sysregister
    hr += ll_read_system_register(Address, &value);

    // Adjust Value
    if (hr == 0)
        result = ((value & AEG_AGC_MASK_ON) == AEG_AGC_MASK_ON); //check that second bit is on
    else
        return hr;

    return result;
}

int VideoCapture::ll_isp_get_gain(uint8_t *val, uint8_t sensorID) {
    int hr = 0;
    uint8_t buffL, buffM, buffH;

    hr += ll_read_sensor_register(sensorID, 1, ADDR_GAIN_H, &buffH);
    hr += ll_read_sensor_register(sensorID, 1, ADDR_GAIN_M, &buffM);
    hr += ll_read_sensor_register(sensorID, 1, ADDR_GAIN_L, &buffL);

    *val = buffL;
    *(val + 1) = buffM;
    *(val + 2) = buffH;

    return hr;
}

int VideoCapture::ll_isp_set_gain(unsigned char ucGainH, unsigned char ucGainM, unsigned char ucGainL, int sensorID)
{
    int hr = 0;
    hr += ll_write_sensor_register(sensorID, 1, ADDR_GAIN_H, ucGainH);
    hr += ll_write_sensor_register(sensorID, 1, ADDR_GAIN_M, ucGainM);
    hr += ll_write_sensor_register(sensorID, 1, ADDR_GAIN_L, ucGainL);
    return hr;
}

int VideoCapture::ll_isp_get_exposure(unsigned char *val, unsigned char sensorID)
{
    int hr = 0;
    uint8_t buffL = 0;
    uint8_t buffM = 0;
    uint8_t buffH = 0;

    hr += ll_read_sensor_register(sensorID, 1, ADDR_EXP_H, (uint8_t*) & buffH);
    usleep(10);
    hr += ll_read_sensor_register(sensorID, 1, ADDR_EXP_M, (uint8_t*) & buffM);
    usleep(10);
    hr += ll_read_sensor_register(sensorID, 1, ADDR_EXP_L, (uint8_t*) & buffL);

    *val = buffL;
    *(val + 1) = buffM;
    *(val + 2) = buffH;

    return hr;
}

int VideoCapture::ll_isp_set_exposure(unsigned char ucExpH, unsigned char ucExpM, unsigned char ucExpL, int sensorID)
{
    int hr = 0;
    hr += ll_write_sensor_register(sensorID, 1, ADDR_EXP_H, ucExpH);
    hr += ll_write_sensor_register(sensorID, 1, ADDR_EXP_M, ucExpM);
    hr += ll_write_sensor_register(sensorID, 1, ADDR_EXP_L, ucExpL);
    return hr;
}

void VideoCapture::ll_activate_sync()
{
    uint8_t sync_val_left = 0x0;
    uint8_t sync_val_right = 0x0;

    // Activate VSYNC output for both camera sensors
    if (ll_read_sensor_register(0, 1, 0x3002, &sync_val_left) == 0)
    {
        sync_val_left = sync_val_left | 0x80;

        ll_write_sensor_register(0, 1, 0x3002, sync_val_left);
    }

    if (ll_read_sensor_register(1, 1, 0x3002, &sync_val_right) == 0)
    {
        sync_val_right = sync_val_right | 0x80;

        ll_write_sensor_register(1, 1, 0x3002, sync_val_right);
    }
}

int VideoCapture::setLEDstatus(bool status)
{
    int hr = 0;
    //LED GPIO : GPIO 2
    if (status) {
        hr += ll_set_gpio_direction(2, 0);
        hr += ll_set_gpio_value(2, 1);
    } else {
        hr += ll_set_gpio_direction(2, 0);
        hr += ll_set_gpio_value(2, 0);
    }

    return hr;
}

int VideoCapture::getLEDstatus(bool *status)
{
    if( status==nullptr)
    {
        return -1;
    }

    uint8_t val;
    int hr = ll_set_gpio_direction(2, 1);
    hr += ll_get_gpio_value(2, &val);
    *status = val!=0;
    return hr;
}

int VideoCapture::toggleLED(bool* value)
{
    bool curVal;

    int ret = getLEDstatus( &curVal );

    if(ret==0)
    {
        bool newVal = !curVal;
        ret = setLEDstatus( newVal );

        if( ret==0 && value!=nullptr )
        {
            *value = newVal;
        }
    }

    return ret;
}

int VideoCapture::getCameraControlSettings(int ctrl_id)
{
    struct v4l2_control control_s;
    struct v4l2_queryctrl queryctrl;
    memset(&queryctrl, 0, sizeof (queryctrl));
    memset(&control_s, 0, sizeof (control_s));
    int res = -1;

    // save_controls(fd);
    queryctrl.id = ctrl_id;

    if (0 != ioctl(mFileDesc, VIDIOC_QUERYCTRL, &queryctrl))
        return res;

    control_s.id = ctrl_id;
    if (ioctl(mFileDesc, VIDIOC_G_CTRL, &control_s) == 0)
        res = (int) control_s.value;

    return res;
}

void VideoCapture::setCameraControlSettings(int ctrl_id, int ctrl_val) {
    struct v4l2_control control_s;
    struct v4l2_queryctrl queryctrl;
    memset(&queryctrl, 0, sizeof (queryctrl));
    memset(&control_s, 0, sizeof (control_s));
    int min, max/*, step, val_def*/;

    // save_controls(fd);
    queryctrl.id = ctrl_id;

    //std::cerr << "[setCameraControlSettings] '" << mDevName << "' [" << mDevId << "] - mFileDesc: " << mFileDesc << std::endl;

    int res = ioctl(mFileDesc, VIDIOC_QUERYCTRL, &queryctrl);
    if (0 == res) {
        min = queryctrl.minimum;
        max = queryctrl.maximum;
        //step = queryctrl.step;
        //val_def = queryctrl.default_value;

        if (ctrl_id == LINUX_CTRL_GAMMA) {
            min = DEFAULT_MIN_GAMMA;
            max = DEFAULT_MAX_GAMMA;
        }

    } else {
        min = 0; // queryctrl.minimum;
        max = 6500; // queryctrl.maximum;
        //step = queryctrl.step;
        //val_def = queryctrl.default_value;
    }

    if ((ctrl_val >= min) && (ctrl_val <= max)) {
        control_s.id = ctrl_id;
        control_s.value = ctrl_val;


        if (ioctl(mFileDesc, VIDIOC_S_CTRL, &control_s) == 0)
            return;
    } else
        return;
}

void VideoCapture::resetCameraControlSettings(int ctrl_id) {

    struct v4l2_control control_s;
    struct v4l2_queryctrl queryctrl;
    memset(&queryctrl, 0, sizeof (queryctrl));
    memset(&control_s, 0, sizeof (control_s));
    int val_def;
    // save_controls(fd);
    queryctrl.id = ctrl_id;
    ioctl(mFileDesc, VIDIOC_QUERYCTRL, &queryctrl);
    val_def = queryctrl.default_value;

    control_s.id = ctrl_id;
    control_s.value = val_def;
    ioctl(mFileDesc, VIDIOC_S_CTRL, &control_s);
    return;
}

int VideoCapture::setGammaPreset(int side, int value)
{
    if (!mInitialized)
        return -1;

    if(value < DEFAULT_MIN_GAMMA)
        value = DEFAULT_MIN_GAMMA;
    if(value > DEFAULT_MAX_GAMMA)
        value = DEFAULT_MAX_GAMMA;

    uint64_t ulAddr = 0x80181500;

    if (side == 1)
        ulAddr = 0x80181D00;

    int hr = 0;

    for (int i = 0; i < 15; i++) {
        hr += ll_write_system_register(ulAddr, PRESET_GAMMA[value-1][i]);
        usleep(10);
        uint8_t valRead = 0x0;
        hr += ll_read_system_register(ulAddr, &valRead);
        if (valRead != PRESET_GAMMA[value-1][i]) {
            return -3;
        }
        ulAddr++;
    }

    ulAddr = 0x80181510;

    if (side == 1)
        ulAddr = 0x80181D10;
    hr += ll_write_system_register(ulAddr, 0x01);
    usleep(10);
    uint8_t valRead = 0x0;
    hr += ll_read_system_register(ulAddr, &valRead);
    if (valRead != 0x01)
        return -2;

    return hr;
}

void VideoCapture::setBrightness(int brightness)
{
    setCameraControlSettings(LINUX_CTRL_BRIGHTNESS, brightness);
}

void VideoCapture::resetBrightness()
{
    resetCameraControlSettings(LINUX_CTRL_BRIGHTNESS);
}

int VideoCapture::getBrightness()
{
    return getCameraControlSettings(LINUX_CTRL_BRIGHTNESS);
}

void VideoCapture::setSharpness(int sharpness)
{
    setCameraControlSettings(LINUX_CTRL_SHARPNESS, sharpness);
}

void VideoCapture::resetSharpness()
{
    resetCameraControlSettings(LINUX_CTRL_SHARPNESS);
}

int VideoCapture::getSharpness()
{
    return getCameraControlSettings(LINUX_CTRL_SHARPNESS);
}

void VideoCapture::setContrast(int contrast)
{
    setCameraControlSettings(LINUX_CTRL_CONTRAST, contrast);
}

void VideoCapture::resetContrast()
{
    resetCameraControlSettings(LINUX_CTRL_CONTRAST);
}

int VideoCapture::getContrast()
{
    return getCameraControlSettings(LINUX_CTRL_CONTRAST);
}

void VideoCapture::setHue(int hue)
{
    setCameraControlSettings(LINUX_CTRL_HUE, hue);
}

void VideoCapture::resetHue()
{
    resetCameraControlSettings(LINUX_CTRL_HUE);
}

int VideoCapture::getHue()
{
    return getCameraControlSettings(LINUX_CTRL_HUE);
}

void VideoCapture::setSaturation(int saturation)
{
    setCameraControlSettings(LINUX_CTRL_SATURATION, saturation);
}

void VideoCapture::resetSaturation()
{
    resetCameraControlSettings(LINUX_CTRL_SATURATION);
}

int VideoCapture::getSaturation()
{
    return getCameraControlSettings(LINUX_CTRL_SATURATION);
}

int VideoCapture::getWhiteBalance()
{
    return getCameraControlSettings(LINUX_CTRL_AWB);
}

void VideoCapture::setWhiteBalance(int wb)
{
    // Disable auto white balance if active
    if (getAutoWhiteBalance() != 0)
        setAutoWhiteBalance(false);

    setCameraControlSettings(LINUX_CTRL_AWB, wb);
}

bool VideoCapture::getAutoWhiteBalance()
{
    return (getCameraControlSettings(LINUX_CTRL_AWB_AUTO)!=0);
}

void VideoCapture::setAutoWhiteBalance(bool active)
{
    setCameraControlSettings(LINUX_CTRL_AWB_AUTO, active?1:0);
}

void VideoCapture::resetAutoWhiteBalance()
{
    setAutoWhiteBalance(true);
}

void VideoCapture::setGamma(int gamma)
{
    int current_gamma = getCameraControlSettings(LINUX_CTRL_GAMMA);

    if (gamma!=current_gamma)
    {
        setGammaPreset(0,gamma);
        setGammaPreset(1,gamma);
        setCameraControlSettings(LINUX_CTRL_GAMMA, gamma);
    }
}

void VideoCapture::resetGamma()
{
    int def_value = DEFAULT_GAMMA_NOECT;
    setGammaPreset(0,def_value);
    setGammaPreset(1,def_value);
    setCameraControlSettings(LINUX_CTRL_GAMMA, def_value);
}

int VideoCapture::getGamma() {
    return getCameraControlSettings(LINUX_CTRL_GAMMA);
}

int VideoCapture::setAECAGC(bool active)
{
    int res = 0;
    res += ll_isp_aecagc_enable(0, active);
    res += ll_isp_aecagc_enable(1, active);
    return res;
}

bool VideoCapture::getAECAGC()
{
    int resL = ll_isp_is_aecagc(0);
    int resR = ll_isp_is_aecagc(1);
    return (resL && resR);
}

void VideoCapture::resetAECAGC()
{
    setAECAGC(true);
}

bool VideoCapture::setROIforAECAGC(CAM_SENS_POS side, uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
    if(side!=CAM_SENS_POS::LEFT && side!=CAM_SENS_POS::RIGHT)
    {
        return false;
    }
    if(w==0 || h==0)
    {
        return false;
    }
    if((x+w)>(mWidth/2) || (y+h)>mHeight)
    {
        return false;
    }
    if(w*h<=100)
    {
        return false;
    }

    int x_start_high = x / 256;
    int x_start_low = (x - x_start_high * 256);
    int y_start_high = y / 256;
    int y_start_low = (y - x_start_high * 256);
    int w_start_high = w / 256;
    int w_start_low = (w - x_start_high * 256);
    int h_start_high = h / 256;
    int h_start_low = (h - x_start_high * 256);


    uint32_t ulAddr = 0x801810C0;
    if (static_cast<int>(side)==1)
        ulAddr= 0x801818C0;

    int r = ll_write_system_register(ulAddr,static_cast<uint8_t>(x_start_high));ulAddr++;usleep(100);
    r += ll_write_system_register(ulAddr,static_cast<uint8_t>(x_start_low));ulAddr++;usleep(100);
    r += ll_write_system_register(ulAddr,static_cast<uint8_t>(y_start_high));ulAddr++;usleep(100);
    r += ll_write_system_register(ulAddr,static_cast<uint8_t>(y_start_low));ulAddr++;usleep(100);
    r += ll_write_system_register(ulAddr,static_cast<uint8_t>(w_start_high));ulAddr++;usleep(100);
    r += ll_write_system_register(ulAddr,static_cast<uint8_t>(w_start_low));ulAddr++;usleep(100);
    r += ll_write_system_register(ulAddr,static_cast<uint8_t>(h_start_high));ulAddr++;usleep(100);
    r += ll_write_system_register(ulAddr,static_cast<uint8_t>(h_start_low));ulAddr++;usleep(100);

    return (r==0);
}

bool VideoCapture::resetROIforAECAGC(CAM_SENS_POS side)
{
    return setROIforAECAGC(side, 0,0, mWidth/2, mHeight);
}

bool VideoCapture::getROIforAECAGC(CAM_SENS_POS side, uint16_t &x, uint16_t &y, uint16_t &w, uint16_t &h)
{
    uint8_t x_start_high = 0;
    uint8_t x_start_low = 0;
    uint8_t y_start_high =0;
    uint8_t y_start_low = 0;
    uint8_t w_start_high =0;
    uint8_t w_start_low = 0;
    uint8_t h_start_high = 0;
    uint8_t h_start_low = 0;
    uint32_t ulAddr = 0x801810C0;
    if (static_cast<int>(side)==1)
        ulAddr= 0x801818C0;
    int r =  ll_read_system_register(ulAddr,&x_start_high);ulAddr++;usleep(100);
    r +=  ll_read_system_register(ulAddr,&x_start_low);ulAddr++;usleep(100);
    r +=   ll_read_system_register(ulAddr,&y_start_high);ulAddr++;usleep(100);
    r +=   ll_read_system_register(ulAddr,&y_start_low);ulAddr++;usleep(100);
    r +=   ll_read_system_register(ulAddr,&w_start_high);ulAddr++;usleep(100);
    r +=   ll_read_system_register(ulAddr,&w_start_low);ulAddr++;usleep(100);
    r +=   ll_read_system_register(ulAddr,&h_start_high);ulAddr++;usleep(100);
    r +=   ll_read_system_register(ulAddr,&h_start_low);ulAddr++;usleep(100);

    x = x_start_high*256 + x_start_low;
    y = y_start_high*256 + y_start_low;
    w = w_start_high*256 + w_start_low;
    h = h_start_high*256 + h_start_low;

    return (r==0);
}

void VideoCapture::setGain(CAM_SENS_POS cam, int gain)
{
    if(getAECAGC())
        setAECAGC(false);

    if (gain <= DEFAULT_MIN_GAIN)
        gain = DEFAULT_MIN_GAIN;
    else if (gain >= DEFAULT_MAX_GAIN)
        gain = DEFAULT_MAX_GAIN;

    uint8_t ucGainH=0, ucGainM=0, ucGainL=0;

    int rawGain = calcRawGainValue(gain);

    int sensorId = static_cast<int>(cam);

    ucGainM = (rawGain >> 8) & 0xff;
    ucGainL = rawGain & 0xff;
    ll_isp_set_gain(ucGainH, ucGainM, ucGainL, sensorId);

}

int VideoCapture::getGain(CAM_SENS_POS cam)
{
    int rawGain=0;

    uint8_t val[3];
    memset(val, 0, 3);

    int sensorId = static_cast<int>(cam);
    int r = ll_isp_get_gain(val, sensorId);
    if(r<0)
        return r;

    rawGain = (int) ((val[1] << 8) + val[0]);
    return calcGainValue(rawGain);
}

void VideoCapture::setExposure(CAM_SENS_POS cam, int exposure)
{
    unsigned char ucExpH, ucExpM, ucExpL;

    if(getAECAGC())
        setAECAGC(false);

    if(exposure < DEFAULT_MIN_EXP)
        exposure = DEFAULT_MIN_EXP;
    if(exposure > DEFAULT_MAX_EXP)
        exposure = DEFAULT_MAX_EXP;

    int rawExp = (mExpoureRawMax * ((float) exposure / 100.0));
    if(rawExp<EXP_RAW_MIN)
        rawExp = EXP_RAW_MIN;

    //std::cout << "Set Raw Exp: " << rawExp << std::endl;

    int sensorId = static_cast<int>(cam);

    ucExpH = (rawExp >> 12) & 0xff;
    ucExpM = (rawExp >> 4) & 0xff;
    ucExpL = (rawExp << 4) & 0xf0;
    ll_isp_set_exposure(ucExpH, ucExpM, ucExpL, sensorId);
}

int VideoCapture::getExposure(CAM_SENS_POS cam)
{
    int rawExp=0;

    unsigned char val[3];
    memset(val, 0, 3);

    int sensorId = static_cast<int>(cam);

    int r = ll_isp_get_exposure(val, sensorId);
    if(r<0)
        return r;
    rawExp = (int) ((val[2] << 12) + (val[1] << 4) + (val[0] >> 4));

    //std::cout << "Get Raw Exp: " << rawExp << std::endl;

    int exposure = static_cast<int>(std::round((100.0*rawExp)/mExpoureRawMax));
    return exposure;
}

int VideoCapture::calcRawGainValue(int gain) {

    // From [0,100] to segmented gain
    int segmentedGain = static_cast<int>(std::round(mGainSegMax * (static_cast<double>(gain) / 100.0)));
    int rawGain = 0;

    // ----> Calculate correct GAIN ZONE
    int gainZone1nbVal = GAIN_ZONE1_MAX - GAIN_ZONE1_MIN;
    int gainZone2nbVal = GAIN_ZONE2_MAX - GAIN_ZONE2_MIN;
    int gainZone3nbVal = GAIN_ZONE3_MAX - GAIN_ZONE3_MIN;
    int gainZone4nbVal = GAIN_ZONE4_MAX - GAIN_ZONE4_MIN;

    if (segmentedGain <= gainZone1nbVal)
        rawGain = segmentedGain + GAIN_ZONE1_MIN;
    else if (segmentedGain <= gainZone1nbVal + gainZone2nbVal)
        rawGain = segmentedGain + GAIN_ZONE2_MIN - (gainZone1nbVal);
    else if (segmentedGain <= gainZone1nbVal + gainZone2nbVal + gainZone3nbVal)
        rawGain = segmentedGain + GAIN_ZONE3_MIN - (gainZone1nbVal + gainZone2nbVal);
    else if (segmentedGain <= gainZone1nbVal + gainZone2nbVal + gainZone3nbVal + gainZone4nbVal)
        rawGain = segmentedGain + GAIN_ZONE4_MIN - (gainZone1nbVal + gainZone2nbVal + gainZone3nbVal);
    // <---- Calculate correct GAIN ZONE

    return rawGain;
}

int VideoCapture::calcGainValue(int rawGain)
{
    int segmentedGain;

    // ----> Calculate correct GAIN ZONE
    int gainZone1nbVal = GAIN_ZONE1_MAX - GAIN_ZONE1_MIN;
    int gainZone2nbVal = GAIN_ZONE2_MAX - GAIN_ZONE2_MIN;
    int gainZone3nbVal = GAIN_ZONE3_MAX - GAIN_ZONE3_MIN;
    if (rawGain >= GAIN_ZONE1_MIN && rawGain <= GAIN_ZONE1_MAX)
        segmentedGain = rawGain - GAIN_ZONE1_MIN;
    else if (rawGain >= GAIN_ZONE2_MIN && rawGain <= GAIN_ZONE2_MAX)
        segmentedGain = rawGain - GAIN_ZONE2_MIN + gainZone1nbVal;
    else if (rawGain >= GAIN_ZONE3_MIN && rawGain <= GAIN_ZONE3_MAX)
        segmentedGain = rawGain - GAIN_ZONE3_MIN + gainZone1nbVal + gainZone2nbVal;
    else if (rawGain >= GAIN_ZONE4_MIN && rawGain <= GAIN_ZONE4_MAX)
        segmentedGain = rawGain - GAIN_ZONE4_MIN + gainZone1nbVal + gainZone2nbVal + gainZone3nbVal;
    else
        segmentedGain = -1;
    // <---- Calculate correct GAIN ZONE

    // From segmented gain to [0,100]
    int gain = static_cast<int>(std::round((100.0*segmentedGain)/mGainSegMax));

    return gain;
}

#ifdef SENSOR_LOG_AVAILABLE
bool VideoCapture::enableAecAgcSensLogging(bool enable, int frame_skip/*=10*/)
{
    if(!enable)
    {
        mLogEnable=false;
        if(mLogFileLeft.is_open())
        {
            mLogFileLeft.close();
        }
        if(mLogFileRight.is_open())
        {
            mLogFileRight.close();
        }
        return true;
    }

    mLogFrameSkip = frame_skip;

    mLogFilenameLeft = getCurrentDateTime(DATE);
    mLogFilenameLeft += "_";
    mLogFilenameLeft += getCurrentDateTime(TIME);
    mLogFilenameLeft += "_agc_aec_registers-LEFT.csv";

    mLogFilenameRight = getCurrentDateTime(DATE);
    mLogFilenameRight += "_";
    mLogFilenameRight += getCurrentDateTime(TIME);
    mLogFilenameRight += "_agc_aec_registers-RIGHT.csv";

    mLogFileLeft.open(mLogFilenameLeft, std::ofstream::out );
    mLogFileRight.open(mLogFilenameRight, std::ofstream::out );

    if(mLogFileLeft.bad())
    {
        std::cerr << "Logging not started. Error creating the log file: '" << mLogFilenameLeft << "'" << std::endl;
        mLogEnable = false;
        return false;
    }

    if(mLogFileRight.bad())
    {
        std::cerr << "Logging not started. Error creating the log file: '" << mLogFilenameRight << "'" << std::endl;
        mLogEnable = false;
        mLogFileLeft.close();
        return false;
    }

    mLogFileLeft << "TIMESTAMP" << LOG_SEP;
    mLogFileLeft << "OV580-ISP_EN_HIGH" << LOG_SEP;
    mLogFileLeft << "OV580-yavg_low" << LOG_SEP;
    mLogFileLeft << "OV580-yavg_high" << LOG_SEP;
    mLogFileLeft << "OV580-interrupt_ctrl1";

    mLogFileRight << "TIMESTAMP" << LOG_SEP;
    mLogFileRight << "OV580-ISP_EN_HIGH" << LOG_SEP;
    mLogFileRight << "OV580-yavg_low" << LOG_SEP;
    mLogFileRight << "OV580-yavg_high" << LOG_SEP;
    mLogFileRight << "OV580-interrupt_ctrl1";

    for (int addr = 0x00; addr <= 0x22; ++addr)
    {
        mLogFileLeft << LOG_SEP << "OV580-YAVG[0x" << std::hex << std::setfill('0') << std::setw(2) << addr << "]";
        mLogFileRight << LOG_SEP << "OV580-YAVG[0x" << std::hex << std::setfill('0') << std::setw(2) << addr << "]";
    }

    for (int addr = 0x3500; addr <= 0x3515; ++addr)
    {
        mLogFileLeft << LOG_SEP << "OV4689-GAIN_EXP[0x" << std::hex << std::setfill('0') << std::setw(4) << addr << "]";
        mLogFileRight << LOG_SEP << "OV4689-GAIN_EXP[0x" << std::hex << std::setfill('0') << std::setw(4) << addr << "]";
    }

    mLogFileLeft << std::endl;
    mLogFileRight << std::endl;

    mLogEnable = true;
    return true;
}

void VideoCapture::setColorBars(int side, bool c)
{


    unsigned long long ulAddr2 = 0x80181080;

    if (side==1)
         ulAddr2 = 0x80181880;

    unsigned char ulValue2 =c?128:0;
    ll_write_system_register(ulAddr2,ulValue2);
}


void VideoCapture::saveAllISPRegisters(std::string filename)
{
    std::ofstream logFile;
    logFile.open(filename,std::ofstream::out);
    int res = 0;
    unsigned long long ulAddrL =  0x80181000;
    unsigned long long   ulAddrR =  0x80181800;

    for (int p = 0;p<0x800;p++)
    {
        uint8_t valL,valR;
        res += ll_read_system_register( ulAddrL+p, &valL);
        res += ll_read_system_register( ulAddrR+p, &valR);

        logFile <<"0x" << std::hex << std::setfill('0') << std::setw(8) << static_cast<unsigned long long>(ulAddrL+p)<<" , "<<std::hex << std::setfill('0') << std::setw(2) << static_cast<unsigned long long>(valL)<<" , "<<std::hex << std::setfill('0') << std::setw(2) << static_cast<unsigned long long>(valR)<<std::endl;
        usleep(10);
    }

    logFile.close();
}

void VideoCapture::saveAllSensorsRegisters(std::string filename)
{
    std::ofstream logFile;
    logFile.open(filename,std::ofstream::out);

    int res = 0;
    for (int addr = 0x3000; addr <= 0x6000; ++addr)
    {
        uint8_t valL,valR;
        res += ll_read_sensor_register( 0, 1, addr, &valL);
        res += ll_read_sensor_register( 1, 1, addr, &valR);

        usleep(10);
        logFile <<"0x" << std::hex << std::setfill('0') << std::setw(8) << static_cast<int>(addr)<<" , "<<std::hex << std::setfill('0') << std::setw(2) << static_cast<unsigned long long>(valL)<<" , "<<std::hex << std::setfill('0') << std::setw(2) << static_cast<unsigned long long>(valR)<<std::endl;
    }

    logFile.close();
}

void VideoCapture::saveLogDataLeft()
{
    const int reg_count = 61;
    uint8_t values[reg_count];
    int idx = 0;

    int res = 0;
    res += ll_read_system_register( 0x80181002, &values[idx++]);
    res += ll_read_system_register( 0x80181031, &values[idx++]);
    res += ll_read_system_register( 0x80181032, &values[idx++]);
    res += ll_read_system_register( 0x80181033, &values[idx++]);

    for(int reg_addr = 0x00; reg_addr <= 0x22; ++reg_addr)
    {
        uint64_t addr = 0x801810C0+reg_addr;
        res += ll_read_system_register( addr, &values[idx++]);
    }

    for (int addr = 0x3500; addr <= 0x3515; ++addr)
    {
        res += ll_read_sensor_register( 0, 1, addr, &values[idx++]);
    }

    mLogFileLeft << std::dec << mLastFrame.timestamp << LOG_SEP;
    for(int i=0; i<reg_count; i++)
    {
        mLogFileLeft << "0x" << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(values[i]);
        if(i==(reg_count-1))
            mLogFileLeft << std::endl;
        else
            mLogFileLeft << LOG_SEP;
    }
}

void VideoCapture::saveLogDataRight()
{
    const int reg_count = 61;
    uint8_t values[reg_count];
    int idx = 0;

    int res = 0;
    res += ll_read_system_register( 0x80181802, &values[idx++]);
    res += ll_read_system_register( 0x80181831, &values[idx++]);
    res += ll_read_system_register( 0x80181832, &values[idx++]);
    res += ll_read_system_register( 0x80181833, &values[idx++]);

    for(int reg_addr = 0x00; reg_addr <= 0x22; ++reg_addr)
    {
        uint64_t addr = 0x801818C0+reg_addr;
        res += ll_read_system_register( addr, &values[idx++]);
    }

    for (int addr = 0x3500; addr <= 0x3515; ++addr)
    {
        res += ll_read_sensor_register( 1, 1, addr, &values[idx++]);
    }

    mLogFileRight << std::dec << mLastFrame.timestamp << LOG_SEP;
    for(int i=0; i<reg_count; i++)
    {
        mLogFileRight << "0x" << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(values[i]);
        if(i==(reg_count-1))
            mLogFileRight << std::endl;
        else
            mLogFileRight << LOG_SEP;
    }
}

bool VideoCapture::resetAGCAECregisters() {
    int res = 0;

    res += ll_write_sensor_register( 0, 1, 0x3503, 0x04);
    res += ll_write_sensor_register( 1, 1, 0x3503, 0x04);
    res += ll_write_sensor_register( 0, 1, 0x3505, 0x00);
    res += ll_write_sensor_register( 1, 1, 0x3505, 0x00);

    return res==0;
}
#endif

#ifdef SENSORS_MOD_AVAILABLE
bool VideoCapture::enableSensorSync( sensors::SensorCapture* sensCap )
{
    if(!sensCap)
        return false;

    // Activate low level sync mechanism
    ll_activate_sync();

    mSyncEnabled = true;
    mSensPtr = sensCap;

    mSensPtr->setVideoPtr(this);

    return true;
}
#endif

}

}



