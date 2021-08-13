/*
 * Copyright (c) 2015, Tal Regev
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Android Sensors Driver nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

package sensor_msgs;

/**
 *
 * @author tal.regev@gmail.com (Tal Regev)
 *
 */
public class ImageEncodings {
    public static final String RGB8 = "rgb8";
    public static final String RGBA8 = "rgba8";
    public static final String RGB16 = "rgb16";
    public static final String RGBA16 = "rgba16";
    public static final String BGR8 = "bgr8";
    public static final String BGRA8 = "bgra8";
    public static final String BGR16 = "bgr16";
    public static final String BGRA16 = "bgra16";
    public static final String MONO8 = "mono8";
    public static final String MONO16 = "mono16";

    // OpenCV CvMat types
    public static final String TYPE_8UC1 = "8UC1";
    public static final String TYPE_8UC2 = "8UC2";
    public static final String TYPE_8UC3 = "8UC3";
    public static final String TYPE_8UC4 = "8UC4";
    public static final String TYPE_8SC1 = "8SC1";
    public static final String TYPE_8SC2 = "8SC2";
    public static final String TYPE_8SC3 = "8SC3";
    public static final String TYPE_8SC4 = "8SC4";
    public static final String TYPE_16UC1 = "16UC1";
    public static final String TYPE_16UC2 = "16UC2";
    public static final String TYPE_16UC3 = "16UC3";
    public static final String TYPE_16UC4 = "16UC4";
    public static final String TYPE_16SC1 = "16SC1";
    public static final String TYPE_16SC2 = "16SC2";
    public static final String TYPE_16SC3 = "16SC3";
    public static final String TYPE_16SC4 = "16SC4";
    public static final String TYPE_32SC1 = "32SC1";
    public static final String TYPE_32SC2 = "32SC2";
    public static final String TYPE_32SC3 = "32SC3";
    public static final String TYPE_32SC4 = "32SC4";
    public static final String TYPE_32FC1 = "32FC1";
    public static final String TYPE_32FC2 = "32FC2";
    public static final String TYPE_32FC3 = "32FC3";
    public static final String TYPE_32FC4 = "32FC4";
    public static final String TYPE_64FC1 = "64FC1";
    public static final String TYPE_64FC2 = "64FC2";
    public static final String TYPE_64FC3 = "64FC3";
    public static final String TYPE_64FC4 = "64FC4";

    // Bayer encodings
    public static final String BAYER_RGGB8 = "bayer_rggb8";
    public static final String BAYER_BGGR8 = "bayer_bggr8";
    public static final String BAYER_GBRG8 = "bayer_gbrg8";
    public static final String BAYER_GRBG8 = "bayer_grbg8";
    public static final String BAYER_RGGB16 = "bayer_rggb16";
    public static final String BAYER_BGGR16 = "bayer_bggr16";
    public static final String BAYER_GBRG16 = "bayer_gbrg16";
    public static final String BAYER_GRBG16 = "bayer_grbg16";

    // Miscellaneous
    // This is the UYVY version of YUV422 codec http://www.fourcc.org/yuv.php#UYVY
    // with an 8-bit depth
    public static final String YUV422 = "yuv422";

    public static boolean isColor(final String encoding)
    {
        String lEncoding = encoding.toLowerCase();
        return lEncoding.equals(RGB8)  || lEncoding.equals(BGR8) ||
                lEncoding.equals(RGBA8) || lEncoding.equals(BGRA8) ||
                lEncoding.equals(RGB16) || lEncoding.equals(BGR16) ||
                lEncoding.equals(RGBA16) || lEncoding.equals(BGRA16);
    }

    public static boolean isMono(final String encoding)
    {
        String lEncoding = encoding.toLowerCase();
        return lEncoding.equals(MONO8) || lEncoding.equals(MONO16);
    }
    public static boolean isBayer(final String encoding )
    {
        String lEncoding = encoding.toLowerCase();
        return lEncoding.equals(BAYER_RGGB8) || lEncoding.equals(BAYER_BGGR8) ||
                lEncoding.equals(BAYER_GBRG8) || lEncoding.equals(BAYER_GRBG8) ||
                lEncoding.equals(BAYER_RGGB16) || lEncoding.equals(BAYER_BGGR16) ||
                lEncoding.equals(BAYER_GBRG16) || lEncoding.equals(BAYER_GRBG16);
    }

    public static boolean hasAlpha(final String encoding )
    {
        String lEncoding = encoding.toLowerCase();
        return lEncoding.equals(RGBA8) || lEncoding.equals(BGRA8) ||
                lEncoding.equals(RGBA16) || lEncoding.equals(BGRA16);
    }

    public static int numChannels(final String enc ) throws Exception {

        String encoding = enc.toLowerCase();
        // First do the common-case encodings
        if (encoding.equals(MONO8) ||
                encoding.equals(MONO16))
			return 1;
        if (encoding.equals(BGR8) ||
                encoding.equals(RGB8) ||
                encoding.equals(BGR16) ||
                encoding.equals(RGB16))
			return 3;
        if (encoding.equals(BGRA8) ||
                encoding.equals(RGBA8) ||
                encoding.equals(BGRA16) ||
                encoding.equals(RGBA16))
			return 4;
        if (encoding.equals(BAYER_RGGB8) ||
                encoding.equals(BAYER_BGGR8) ||
                encoding.equals(BAYER_GBRG8) ||
                encoding.equals(BAYER_GRBG8) ||
                encoding.equals(BAYER_RGGB16) ||
                encoding.equals(BAYER_BGGR16) ||
                encoding.equals(BAYER_GBRG16) ||
                encoding.equals(BAYER_GRBG16))
			return 1;

        if (encoding.equals(YUV422))
            return 2;

        encoding = encoding.toUpperCase();

        if (encoding.equals(TYPE_8UC1) ||
                encoding.equals(TYPE_8SC1) ||
                encoding.equals(TYPE_16UC1) ||
                encoding.equals(TYPE_16SC1) ||
                encoding.equals(TYPE_32SC1) ||
                encoding.equals(TYPE_32FC1) ||
                encoding.equals(TYPE_64FC1))
			return 1;

        if (encoding.equals(TYPE_8UC2) ||
                encoding.equals(TYPE_8SC2) ||
                encoding.equals(TYPE_16UC2) ||
                encoding.equals(TYPE_16SC2) ||
                encoding.equals(TYPE_32SC2) ||
                encoding.equals(TYPE_32FC2) ||
                encoding.equals(TYPE_64FC2))
			return 2;

        if (encoding.equals(TYPE_8UC3) ||
                encoding.equals(TYPE_8SC3) ||
                encoding.equals(TYPE_16UC3) ||
                encoding.equals(TYPE_16SC3) ||
                encoding.equals(TYPE_32SC3) ||
                encoding.equals(TYPE_32FC3) ||
                encoding.equals(TYPE_64FC3))
			return 3;

        if (encoding.equals(TYPE_8UC4) ||
                encoding.equals(TYPE_8SC4) ||
                encoding.equals(TYPE_16UC4) ||
                encoding.equals(TYPE_16SC4) ||
                encoding.equals(TYPE_32SC4) ||
                encoding.equals(TYPE_32FC4) ||
                encoding.equals(TYPE_64FC4))
			return 4;

        throw new Exception("Unknown encoding " + encoding);
    }

    public static int bitDepth(final String enc ) throws Exception {

        String encoding = enc.toLowerCase();
        if (encoding.equals(MONO16))
			return 16;
        if (encoding.equals(MONO8) ||
                encoding.equals(BGR8) ||
                encoding.equals(RGB8) ||
                encoding.equals(BGRA8) ||
                encoding.equals(RGBA8) ||
                encoding.equals(BAYER_RGGB8) ||
                encoding.equals(BAYER_BGGR8) ||
                encoding.equals(BAYER_GBRG8) ||
                encoding.equals(BAYER_GRBG8))
			return 8;

        if (encoding.equals(MONO16) ||
                encoding.equals(BGR16) ||
                encoding.equals(RGB16) ||
                encoding.equals(BGRA16) ||
                encoding.equals(RGBA16) ||
                encoding.equals(BAYER_RGGB16) ||
                encoding.equals(BAYER_BGGR16) ||
                encoding.equals(BAYER_GBRG16) ||
                encoding.equals(BAYER_GRBG16))
			return 16;

        if (encoding.equals(YUV422))
            return 8;

        encoding = encoding.toUpperCase();

        if (encoding.equals(TYPE_8UC1) ||
                encoding.equals(TYPE_8UC2) ||
                encoding.equals(TYPE_8UC3) ||
                encoding.equals(TYPE_8UC4))
			return 8;

        if (encoding.equals(TYPE_8SC1) ||
                encoding.equals(TYPE_8SC2) ||
                encoding.equals(TYPE_8SC3) ||
                encoding.equals(TYPE_8SC4))
			return 8;

        if (encoding.equals(TYPE_16UC1) ||
                encoding.equals(TYPE_16UC2) ||
                encoding.equals(TYPE_16UC3) ||
                encoding.equals(TYPE_16UC4))
			return 16;

        if (encoding.equals(TYPE_16SC1) ||
                encoding.equals(TYPE_16SC2) ||
                encoding.equals(TYPE_16SC3) ||
                encoding.equals(TYPE_16SC4))
			return 16;

        if (encoding.equals(TYPE_32SC1) ||
                encoding.equals(TYPE_32SC2) ||
                encoding.equals(TYPE_32SC3) ||
                encoding.equals(TYPE_32SC4))
			return 32;

        if (encoding.equals(TYPE_32FC1) ||
                encoding.equals(TYPE_32FC2) ||
                encoding.equals(TYPE_32FC3) ||
                encoding.equals(TYPE_32FC4))
			return 32;

        if (encoding.equals(TYPE_64FC1) ||
                encoding.equals(TYPE_64FC2) ||
                encoding.equals(TYPE_64FC3) ||
                encoding.equals(TYPE_64FC4))
			return 64;

        throw new Exception("Unknown encoding " + encoding);
    }
}
