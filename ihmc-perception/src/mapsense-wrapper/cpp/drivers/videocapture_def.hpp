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

#ifndef VIDEOCAPTURE_DEF_HPP
#define VIDEOCAPTURE_DEF_HPP

#ifndef NSEC_PER_SEC
#define NSEC_PER_SEC                   1000000000ULL
#endif

#include "defines.hpp"

namespace sl_oc {

namespace video {

/*!
 * \brief Camera models
 */
enum class SL_DEVICE {
    ZED,        //!< ZED old FW
    ZED_M,      //!< ZED Mini old FW
    ZED_CBS,    //!< ZED new FW
    ZED_M_CBS,  //!< ZED Mini new FW
    ZED_2,      //!< ZED2
    ZED_2i,      //!< ZED2
    NONE
};

/*!
 * \brief Available resolutions
 */
enum class RESOLUTION {
    HD2K,       /**< 2208*1242, available framerates: 15 fps.*/
    HD1080,     /**< 1920*1080, available framerates: 15, 30 fps.*/
    HD720,      /**< 1280*720, available framerates: 15, 30, 60 fps.*/
    VGA,        /**< 672*376, available framerates: 15, 30, 60, 100 fps.*/
    LAST
};

/*!
 * \brief Available frames per seconds
 */
enum class FPS {
    FPS_15 = 15,    //!< 15 Frames per second. Available for all the resolutions.
    FPS_30 = 30,    //!< 30 Frames per second. Not available for \ref RESOLUTION::HD2K.
    FPS_60 = 60,    //!< 60 Frames per second. Not available for \ref RESOLUTION::HD2K and  \ref RESOLUTION::HD1080.
    FPS_100 = 100,  //!< 100 Frames per second. Only available for \ref RESOLUTION::VGA.
    LAST = 101
};

/*!
 * \brief Position of the Camera CMOS sensors
 */
enum class CAM_SENS_POS {
    LEFT = 0,   //!< The left sensor
    RIGHT = 1,  //!< The right sensor
    LAST = 3
};

/*!
 * \brief The camera configuration parameters
 */
typedef struct VideoParams
{
    /*!
     * \brief Default constructor setting the default parameter values
     */
    VideoParams() {
        res = RESOLUTION::HD2K;
        fps = FPS::FPS_15;
        verbose= sl_oc::VERBOSITY::ERROR;
    }

    RESOLUTION res; //!< Camera resolution
    FPS fps;        //!< Frames per second
    int verbose;   //!< Verbose mode
} VideoParams;

/*!
 * \brief Resolution in pixel for each frame
 */
struct Resolution {
    size_t width; //!< array width in pixels
    size_t height; //!< array height in pixels

    /*!
     * \brief Constructor
     * \param w_ frame width in pixels
     * \param h_ frame height in pixels
     */
    Resolution(size_t w_ = 0, size_t h_ = 0) {
        width = w_;
        height = h_;
    }
};

/*! \brief Vector of the available resolutions */
static const std::vector<Resolution> cameraResolution = {
    Resolution(2208, 1242),     /**< HD2K */
    Resolution(1920, 1080),     /**< HD1080 */
    Resolution(1280, 720),      /**< HD720 */
    Resolution(672, 376)        /**< VGA */
};



/*!
 * \brief The Buffer struct used by UVC to store frame data
 */
struct UVCBuffer {
    void *start;    //!< Address of the first byte of the buffer
    size_t length;  //!< Size of the buffer
};


/*!
 *  Camera presets for gamma
 */

const unsigned char PRESET_GAMMA[9][16] = {
    {7,14,29,54,66,78,89,103,114,123,139,154,183,206,228,254}, // gamma min
    {9,17,34,58,71,83,89,108,118,127,143,158,186,208,229,254},
    {10,20,38,63,75,88,99,112,123,132,147,162,189,210,230,254},
    {12,23,43,67,80,92,103,117,127,136,151,165,192,212,231,254},
    {13,26,47,71,84,97,108,121,131,140,155,169,195,214,232,255}, // gamma middle/default
    {18,32,54,80,93,106,117,130,140,149,164,177,202,219,236,255},
    {24,38,61,88,102,115,127,139,148,157,172,186,209,225,240,255},
    {29,44,68,97,111,124,136,147,157,166,181,194,215,230,243,255},
    {34,50,75,105,120,133,145,156,165,174,189,202,222,235,247,255} // gamma strong
};


}

}

#endif // VIDEOCAPTURE_DEF_HPP
