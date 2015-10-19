#pragma once
#include "DeckLinkAPIVersion.h"

#if (BLACKMAGIC_DECKLINK_API_VERSION > 0x07030000)
    /* IID_IDeckLinkVideoConversion is only available in recent DeckLink SDKs,
       at least 7.9 has it and 7.3 does not have it, so it's fine for now
       but we might need to find a better way determining the SDK version
    */
    /* recent SDK supports SetInt */
    #define DECKLINK_SET_VIDEO_CONNECTION(x) deckLinkConfiguration->SetInt(bmdDeckLinkConfigVideoInputConnection, x)
    #define DECKLINK_SET_AUDIO_CONNECTION(x) deckLinkConfiguration->SetInt(bmdDeckLinkConfigAudioInputConnection, x)
#else
    /* <=sdk-7.3 */
    #define DECKLINK_SET_VIDEO_CONNECTION(x) deckLinkConfiguration->SetVideoInputFormat(x)
    #define DECKLINK_SET_AUDIO_CONNECTION(x) deckLinkConfiguration->SetAudioInputFormat(x)
#endif

extern "C" {
#ifdef HAVE_CFSTRING
#include <CoreFoundation/CoreFoundation.h>
typedef CFStringRef BMDProbeString;
#define ToStr(str) CFStringGetCStringPtr(str, kCFStringEncodingMacRoman)
#define FreeStr(str) CFRelease(str)
#else
typedef const char* BMDProbeString;
#define ToStr(str) (str)
#define FreeStr(str) free((void *)str)
#endif
}
