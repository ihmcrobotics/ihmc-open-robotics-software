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

#ifndef DEFINES_HPP
#define DEFINES_HPP

#include <stdint.h>
#include <string>
#include <string.h>
#include <cxxabi.h>
#include <iostream>
#include <vector>
#include <chrono>

#if defined _WIN32
#if defined(SL_OC_COMPIL)
#define SL_OC_EXPORT __declspec(dllexport)
#else
#define SL_OC_EXPORT __declspec(dllimport)
#endif
#elif __GNUC__
#define SL_OC_EXPORT __attribute__((visibility("default")))
#if defined(__arm__) || defined(__aarch64__)
#define _SL_JETSON_
#endif
#endif

//// SDK VERSION NUMBER
#define ZED_OC_MAJOR_VERSION 0
#define ZED_OC_MINOR_VERSION 2
#define ZED_OC_PATCH_VERSION 0

#define ZED_OC_VERSION_ATTRIBUTE private: uint32_t mMajorVer = ZED_OC_MAJOR_VERSION, mMinorVer = ZED_OC_MINOR_VERSION, mPatchVer = ZED_OC_PATCH_VERSION

// Debug output
#define INFO_OUT(lvl,msg) { int status_dem_0; if (lvl>=3) std::cout << "[" << abi::__cxa_demangle(typeid(*this).name(), 0, 0, &status_dem_0) << "] INFO: " << msg << std::endl; }
#define WARNING_OUT(lvl,msg) { int status_dem_0; if (lvl>=2) std::cerr << "[" << abi::__cxa_demangle(typeid(*this).name(), 0, 0, &status_dem_0) << "] WARNING: " << msg << std::endl; }
#define ERROR_OUT(lvl,msg) { int status_dem_0; if (lvl>=1) std::cerr << "[" << abi::__cxa_demangle(typeid(*this).name(), 0, 0, &status_dem_0) << "] ERROR: " << msg << std::endl; }



/*!
 * \brief Get the current system clock as steady clock, so with no jumps even if the system time changes
 * \return the current steady system clock in nanoseconds
 */
inline uint64_t getSteadyTimestamp() {return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}

/*!
 * \brief Get the current system clock as wall clock (it can have jumps if the system clock is updated by a sync service)
 * \return the current wall system clock in nanoseconds
 */
inline uint64_t getWallTimestamp() {return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();}

namespace sl_oc {

static const uint16_t SL_USB_VENDOR = 0x2b03;           //!< Stereolabs Vendor ID

static const uint16_t SL_USB_PROD_ZED_REVA = 0xf580;         //!< Old ZED firmware Product ID
static const uint16_t SL_USB_PROD_ZED_M_REVA = 0xf680;       //!< Old ZED-M binary modified firmware Product ID
static const uint16_t SL_USB_PROD_ZED_REVB = 0xf582;     //!< CBS ZED Firmware Product ID
static const uint16_t SL_USB_PROD_ZED_M_REVB = 0xf682;   //!< CBS ZED-M Firmware Product ID
static const uint16_t SL_USB_PROD_ZED_2_REVB = 0xf780;   //!< CBS ZED 2 Firmware Product ID
static const uint16_t SL_USB_PROD_ZED_2i = 0xf880;   //!< CBS ZED 2i Firmware Product ID
static const uint16_t SL_USB_PROD_MCU_ZEDM_REVA= 0xf681; //!< MCU sensor device for ZED-M
static const uint16_t SL_USB_PROD_MCU_ZED2_REVA = 0xf781; //!< MCU sensor device for ZED2
static const uint16_t SL_USB_PROD_MCU_ZED2i_REVA = 0xf881; //!< MCU sensor device for ZED2i

enum VERBOSITY {
    NONE = 0,
    ERROR = 1,
    WARNING = 2,
    INFO = 3
};

}

#endif //DEFINES_HPP
