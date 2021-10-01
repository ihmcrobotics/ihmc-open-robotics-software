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

package us.ihmc.perception;

import org.apache.commons.lang3.tuple.Pair;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.Vector;

/**
 * Adapted from https://github.com/talregev/android_cv_bridge/tree/master/cv_bridge_javacv/src/cv_bridge
 */
public class ImageEncodingTools
{
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

   public static final int SAME_FORMAT = -1;

   public static boolean isColor(String encoding)
   {
      encoding = encoding.toLowerCase();
      return     encoding.equals(RGB8)
              || encoding.equals(BGR8)
              || encoding.equals(RGBA8)
              || encoding.equals(BGRA8)
              || encoding.equals(RGB16)
              || encoding.equals(BGR16)
              || encoding.equals(RGBA16)
              || encoding.equals(BGRA16);
   }

   public static double getMaxBitValue(String encoding)
   {
      encoding = encoding.toLowerCase();
      if (encoding.equals("32uc1"))
         return 4294967295.0; // 2^32 - 1
      if (encoding.equals("16uc1") || encoding.equals(MONO16))
         return 65535.0; // 2^16 - 1
      else if (encoding.equals("8uc1") || encoding.equals(MONO8))
         return 255.0; // 2^8 - 1
      else
         return Double.NaN;
   }

   public static boolean isMono(String encoding)
   {
      encoding = encoding.toLowerCase();
      return encoding.equals("16uc1") || encoding.equals(MONO8) || encoding.equals(MONO16);
   }

   public static boolean isBayer(String encoding)
   {
      encoding = encoding.toLowerCase();
      return encoding.equals(BAYER_RGGB8)
          || encoding.equals(BAYER_BGGR8)
          || encoding.equals(BAYER_GBRG8)
          || encoding.equals(BAYER_GRBG8)
          || encoding.equals(BAYER_RGGB16)
          || encoding.equals(BAYER_BGGR16)
          || encoding.equals(BAYER_GBRG16)
          || encoding.equals(BAYER_GRBG16);
   }

   public static boolean hasAlpha(String encoding)
   {
      encoding = encoding.toLowerCase();
      return encoding.equals(RGBA8)
          || encoding.equals(BGRA8)
          || encoding.equals(RGBA16)
          || encoding.equals(BGRA16);
   }

   public static int numChannels(String encoding)
   {
      encoding = encoding.toLowerCase();
      // First do the common-case encodings
      if (encoding.equals("16uc1") || encoding.equals(MONO8) || encoding.equals(MONO16))
         return 1;
      if (encoding.equals(BGR8) || encoding.equals(RGB8) || encoding.equals(BGR16) || encoding.equals(RGB16))
         return 3;
      if (encoding.equals(BGRA8) || encoding.equals(RGBA8) || encoding.equals(BGRA16) || encoding.equals(RGBA16))
         return 4;
      if (encoding.equals(BAYER_RGGB8)
       || encoding.equals(BAYER_BGGR8)
       || encoding.equals(BAYER_GBRG8)
       || encoding.equals(BAYER_GRBG8)
       || encoding.equals(BAYER_RGGB16)
       || encoding.equals(BAYER_BGGR16)
       || encoding.equals(BAYER_GBRG16)
       || encoding.equals(BAYER_GRBG16))
         return 1;

      if (encoding.equals(YUV422))
         return 2;

      encoding = encoding.toUpperCase();

      if (encoding.equals(TYPE_8UC1)
       || encoding.equals(TYPE_8SC1)
       || encoding.equals(TYPE_16UC1)
       || encoding.equals(TYPE_16SC1)
       || encoding.equals(TYPE_32SC1)
       || encoding.equals(TYPE_32FC1)
       || encoding.equals(TYPE_64FC1))
         return 1;

      if (encoding.equals(TYPE_8UC2)
       || encoding.equals(TYPE_8SC2)
       || encoding.equals(TYPE_16UC2)
       || encoding.equals(TYPE_16SC2)
       || encoding.equals(TYPE_32SC2)
       || encoding.equals(TYPE_32FC2)
       || encoding.equals(TYPE_64FC2))
         return 2;

      if (encoding.equals(TYPE_8UC3)
       || encoding.equals(TYPE_8SC3)
       || encoding.equals(TYPE_16UC3)
       || encoding.equals(TYPE_16SC3)
       || encoding.equals(TYPE_32SC3)
       || encoding.equals(TYPE_32FC3)
       || encoding.equals(TYPE_64FC3))
         return 3;

      if (encoding.equals(TYPE_8UC4)
       || encoding.equals(TYPE_8SC4)
       || encoding.equals(TYPE_16UC4)
       || encoding.equals(TYPE_16SC4)
       || encoding.equals(TYPE_32SC4)
       || encoding.equals(TYPE_32FC4)
       || encoding.equals(TYPE_64FC4))
         return 4;

      throw new RuntimeException("Unknown encoding " + encoding);
   }

   public static int bitDepth(String encoding)
   {
      encoding = encoding.toLowerCase();
      if (encoding.equals(MONO8)
          || encoding.equals(BGR8)
          || encoding.equals(RGB8)
          || encoding.equals(BGRA8)
          || encoding.equals(RGBA8)
          || encoding.equals(BAYER_RGGB8)
          || encoding.equals(BAYER_BGGR8)
          || encoding.equals(BAYER_GBRG8)
          || encoding.equals(BAYER_GRBG8))
         return 8;

      if (encoding.equals("16uc1") ||
          encoding.equals(MONO16)
          || encoding.equals(BGR16)
          || encoding.equals(RGB16)
          || encoding.equals(BGRA16)
          || encoding.equals(RGBA16)
          || encoding.equals(BAYER_RGGB16)
          || encoding.equals(BAYER_BGGR16)
          || encoding.equals(BAYER_GBRG16)
          || encoding.equals(BAYER_GRBG16))
         return 16;

      if (encoding.equals(YUV422))
         return 8;

      encoding = encoding.toUpperCase();

      if (encoding.equals(TYPE_8UC1) || encoding.equals(TYPE_8UC2) || encoding.equals(TYPE_8UC3) || encoding.equals(TYPE_8UC4))
         return 8;

      if (encoding.equals(TYPE_8SC1) || encoding.equals(TYPE_8SC2) || encoding.equals(TYPE_8SC3) || encoding.equals(TYPE_8SC4))
         return 8;

      if (encoding.equals(TYPE_16UC1) || encoding.equals(TYPE_16UC2) || encoding.equals(TYPE_16UC3) || encoding.equals(TYPE_16UC4))
         return 16;

      if (encoding.equals(TYPE_16SC1) || encoding.equals(TYPE_16SC2) || encoding.equals(TYPE_16SC3) || encoding.equals(TYPE_16SC4))
         return 16;

      if (encoding.equals(TYPE_32SC1) || encoding.equals(TYPE_32SC2) || encoding.equals(TYPE_32SC3) || encoding.equals(TYPE_32SC4))
         return 32;

      if (encoding.equals(TYPE_32FC1) || encoding.equals(TYPE_32FC2) || encoding.equals(TYPE_32FC3) || encoding.equals(TYPE_32FC4))
         return 32;

      if (encoding.equals(TYPE_64FC1) || encoding.equals(TYPE_64FC2) || encoding.equals(TYPE_64FC3) || encoding.equals(TYPE_64FC4))
         return 64;

      throw new RuntimeException("Unknown encoding " + encoding);
   }

   public static int getCvType(String encoding)
   {
      encoding = encoding.toLowerCase();
      if (encoding.equals(ImageEncodingTools.BGR8))
         return opencv_core.CV_8UC3;
      if (encoding.equals(ImageEncodingTools.MONO8))
         return opencv_core.CV_8UC1;
      if (encoding.equals(ImageEncodingTools.RGB8))
         return opencv_core.CV_8UC3;
      if (encoding.equals(ImageEncodingTools.MONO16))
         return opencv_core.CV_16UC1;
      if (encoding.equals(ImageEncodingTools.BGR16))
         return opencv_core.CV_16UC3;
      if (encoding.equals(ImageEncodingTools.RGB16))
         return opencv_core.CV_16UC3;
      if (encoding.equals(ImageEncodingTools.BGRA8))
         return opencv_core.CV_8UC4;
      if (encoding.equals(ImageEncodingTools.RGBA8))
         return opencv_core.CV_8UC4;
      if (encoding.equals(ImageEncodingTools.BGRA16))
         return opencv_core.CV_16UC4;
      if (encoding.equals(ImageEncodingTools.RGBA16))
         return opencv_core.CV_16UC4;

      // For bayer, return one-channel
      if (encoding.equals(ImageEncodingTools.BAYER_RGGB8))
         return opencv_core.CV_8UC1;
      if (encoding.equals(ImageEncodingTools.BAYER_BGGR8))
         return opencv_core.CV_8UC1;
      if (encoding.equals(ImageEncodingTools.BAYER_GBRG8))
         return opencv_core.CV_8UC1;
      if (encoding.equals(ImageEncodingTools.BAYER_GRBG8))
         return opencv_core.CV_8UC1;
      if (encoding.equals(ImageEncodingTools.BAYER_RGGB16))
         return opencv_core.CV_16UC1;
      if (encoding.equals(ImageEncodingTools.BAYER_BGGR16))
         return opencv_core.CV_16UC1;
      if (encoding.equals(ImageEncodingTools.BAYER_GBRG16))
         return opencv_core.CV_16UC1;
      if (encoding.equals(ImageEncodingTools.BAYER_GRBG16))
         return opencv_core.CV_16UC1;

      // Miscellaneous
      if (encoding.equals(ImageEncodingTools.YUV422))
         return opencv_core.CV_8UC2;

      encoding = encoding.toUpperCase();

      //macro code
      if (encoding.equals(ImageEncodingTools.TYPE_8UC1))
         return opencv_core.CV_8UC1;
      if (encoding.equals(ImageEncodingTools.TYPE_8UC2))
         return opencv_core.CV_8UC2;
      if (encoding.equals(ImageEncodingTools.TYPE_8UC3))
         return opencv_core.CV_8UC3;
      if (encoding.equals(ImageEncodingTools.TYPE_8UC4))
         return opencv_core.CV_8UC4;
      if (encoding.equals(ImageEncodingTools.TYPE_8SC1))
         return opencv_core.CV_8SC1;
      if (encoding.equals(ImageEncodingTools.TYPE_8SC2))
         return opencv_core.CV_8SC2;
      if (encoding.equals(ImageEncodingTools.TYPE_8SC3))
         return opencv_core.CV_8SC3;
      if (encoding.equals(ImageEncodingTools.TYPE_8SC4))
         return opencv_core.CV_8SC4;
      if (encoding.equals(ImageEncodingTools.TYPE_16UC1))
         return opencv_core.CV_16UC1;
      if (encoding.equals(ImageEncodingTools.TYPE_16UC2))
         return opencv_core.CV_16UC2;
      if (encoding.equals(ImageEncodingTools.TYPE_16UC3))
         return opencv_core.CV_16UC3;
      if (encoding.equals(ImageEncodingTools.TYPE_16UC4))
         return opencv_core.CV_16UC4;
      if (encoding.equals(ImageEncodingTools.TYPE_16SC1))
         return opencv_core.CV_16SC1;
      if (encoding.equals(ImageEncodingTools.TYPE_16SC2))
         return opencv_core.CV_16SC2;
      if (encoding.equals(ImageEncodingTools.TYPE_16SC3))
         return opencv_core.CV_16SC3;
      if (encoding.equals(ImageEncodingTools.TYPE_16SC4))
         return opencv_core.CV_16SC4;
      if (encoding.equals(ImageEncodingTools.TYPE_32SC1))
         return opencv_core.CV_32SC1;
      if (encoding.equals(ImageEncodingTools.TYPE_32SC2))
         return opencv_core.CV_32SC2;
      if (encoding.equals(ImageEncodingTools.TYPE_32SC3))
         return opencv_core.CV_32SC3;
      if (encoding.equals(ImageEncodingTools.TYPE_32SC4))
         return opencv_core.CV_32SC4;
      if (encoding.equals(ImageEncodingTools.TYPE_32FC1))
         return opencv_core.CV_32FC1;
      if (encoding.equals(ImageEncodingTools.TYPE_32FC2))
         return opencv_core.CV_32FC2;
      if (encoding.equals(ImageEncodingTools.TYPE_32FC3))
         return opencv_core.CV_32FC3;
      if (encoding.equals(ImageEncodingTools.TYPE_32FC4))
         return opencv_core.CV_32FC4;
      if (encoding.equals(ImageEncodingTools.TYPE_64FC1))
         return opencv_core.CV_64FC1;
      if (encoding.equals(ImageEncodingTools.TYPE_64FC2))
         return opencv_core.CV_64FC2;
      if (encoding.equals(ImageEncodingTools.TYPE_64FC3))
         return opencv_core.CV_64FC3;
      if (encoding.equals(ImageEncodingTools.TYPE_64FC4))
         return opencv_core.CV_64FC4;

      throw new RuntimeException("Unrecognized image encoding [" + encoding + "]");
   }

   public static int safeLongToInt(long l)
   {
      if (l < Integer.MIN_VALUE || l > Integer.MAX_VALUE)
      {
         throw new IllegalArgumentException(l + " cannot be cast to int without changing its value.");
      }
      return (int) l;
   }

   public static Map<Pair<OpenCVColorFormats, OpenCVColorFormats>, Vector<Integer>> getConversionCodes()
   {
      Map<Pair<OpenCVColorFormats, OpenCVColorFormats>, Vector<Integer>> conversionCodeMap = new HashMap<>();

      for (int i = 0; i <= 5; ++i)
      {
         conversionCodeMap.put(Pair.of(OpenCVColorFormats.valueOf(i), OpenCVColorFormats.valueOf(i)), new Vector<>(Collections.singletonList(SAME_FORMAT)));
      }

      conversionCodeMap.put(Pair.of(OpenCVColorFormats.GRAY, OpenCVColorFormats.RGB), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_GRAY2RGB)));
      conversionCodeMap.put(Pair.of(OpenCVColorFormats.GRAY, OpenCVColorFormats.BGR), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_GRAY2BGR)));
      conversionCodeMap.put(Pair.of(OpenCVColorFormats.GRAY, OpenCVColorFormats.RGBA), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_GRAY2RGBA)));
      conversionCodeMap.put(Pair.of(OpenCVColorFormats.GRAY, OpenCVColorFormats.BGRA), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_GRAY2BGRA)));

      conversionCodeMap.put(Pair.of(OpenCVColorFormats.RGB, OpenCVColorFormats.GRAY), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_RGB2GRAY)));
      conversionCodeMap.put(Pair.of(OpenCVColorFormats.RGB, OpenCVColorFormats.BGR), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_RGB2BGR)));
      conversionCodeMap.put(Pair.of(OpenCVColorFormats.RGB, OpenCVColorFormats.RGBA), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_RGB2RGBA)));
      conversionCodeMap.put(Pair.of(OpenCVColorFormats.RGB, OpenCVColorFormats.BGRA), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_RGB2BGRA)));

      conversionCodeMap.put(Pair.of(OpenCVColorFormats.BGR, OpenCVColorFormats.GRAY), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_BGR2GRAY)));
      conversionCodeMap.put(Pair.of(OpenCVColorFormats.BGR, OpenCVColorFormats.RGB), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_BGR2RGB)));
      conversionCodeMap.put(Pair.of(OpenCVColorFormats.BGR, OpenCVColorFormats.RGBA), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_BGR2RGBA)));
      conversionCodeMap.put(Pair.of(OpenCVColorFormats.BGR, OpenCVColorFormats.BGRA), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_BGR2BGRA)));

      conversionCodeMap.put(Pair.of(OpenCVColorFormats.RGBA, OpenCVColorFormats.GRAY), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_RGBA2GRAY)));
      conversionCodeMap.put(Pair.of(OpenCVColorFormats.RGBA, OpenCVColorFormats.RGB), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_RGBA2RGB)));
      conversionCodeMap.put(Pair.of(OpenCVColorFormats.RGBA, OpenCVColorFormats.BGR), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_RGBA2BGR)));
      conversionCodeMap.put(Pair.of(OpenCVColorFormats.RGBA, OpenCVColorFormats.BGRA), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_RGBA2BGRA)));

      conversionCodeMap.put(Pair.of(OpenCVColorFormats.BGRA, OpenCVColorFormats.GRAY), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_BGRA2GRAY)));
      conversionCodeMap.put(Pair.of(OpenCVColorFormats.BGRA, OpenCVColorFormats.RGB), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_BGRA2RGB)));
      conversionCodeMap.put(Pair.of(OpenCVColorFormats.BGRA, OpenCVColorFormats.BGR), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_BGRA2BGR)));
      conversionCodeMap.put(Pair.of(OpenCVColorFormats.BGRA, OpenCVColorFormats.RGBA), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_BGRA2RGBA)));

      conversionCodeMap.put(Pair.of(OpenCVColorFormats.YUV422, OpenCVColorFormats.GRAY), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_YUV2GRAY_UYVY)));
      conversionCodeMap.put(Pair.of(OpenCVColorFormats.YUV422, OpenCVColorFormats.RGB), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_YUV2RGB_UYVY)));
      conversionCodeMap.put(Pair.of(OpenCVColorFormats.YUV422, OpenCVColorFormats.BGR), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_YUV2BGR_UYVY)));
      conversionCodeMap.put(Pair.of(OpenCVColorFormats.YUV422, OpenCVColorFormats.RGBA), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_YUV2RGBA_UYVY)));
      conversionCodeMap.put(Pair.of(OpenCVColorFormats.YUV422, OpenCVColorFormats.BGRA), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_YUV2BGRA_UYVY)));

      // Deal with Bayer
      conversionCodeMap.put(Pair.of(OpenCVColorFormats.BAYER_RGGB, OpenCVColorFormats.GRAY), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_BayerBG2GRAY)));
      conversionCodeMap.put(Pair.of(OpenCVColorFormats.BAYER_RGGB, OpenCVColorFormats.RGB), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_BayerBG2RGB)));
      conversionCodeMap.put(Pair.of(OpenCVColorFormats.BAYER_RGGB, OpenCVColorFormats.BGR), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_BayerBG2BGR)));

      conversionCodeMap.put(Pair.of(OpenCVColorFormats.BAYER_BGGR, OpenCVColorFormats.GRAY), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_BayerRG2GRAY)));
      conversionCodeMap.put(Pair.of(OpenCVColorFormats.BAYER_BGGR, OpenCVColorFormats.RGB), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_BayerRG2RGB)));
      conversionCodeMap.put(Pair.of(OpenCVColorFormats.BAYER_BGGR, OpenCVColorFormats.BGR), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_BayerRG2BGR)));

      conversionCodeMap.put(Pair.of(OpenCVColorFormats.BAYER_GBRG, OpenCVColorFormats.GRAY), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_BayerGR2GRAY)));
      conversionCodeMap.put(Pair.of(OpenCVColorFormats.BAYER_GBRG, OpenCVColorFormats.RGB), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_BayerGR2RGB)));
      conversionCodeMap.put(Pair.of(OpenCVColorFormats.BAYER_GBRG, OpenCVColorFormats.BGR), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_BayerGR2BGR)));

      conversionCodeMap.put(Pair.of(OpenCVColorFormats.BAYER_GRBG, OpenCVColorFormats.GRAY), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_BayerGB2GRAY)));
      conversionCodeMap.put(Pair.of(OpenCVColorFormats.BAYER_GRBG, OpenCVColorFormats.RGB), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_BayerGB2RGB)));
      conversionCodeMap.put(Pair.of(OpenCVColorFormats.BAYER_GRBG, OpenCVColorFormats.BGR), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_BayerGB2BGR)));

      return conversionCodeMap;
   }

   public static OpenCVColorFormats getColorFormat(String encoding)
   {
      encoding = encoding.toLowerCase();

      if (encoding.equals("16uc1") || encoding.equals(ImageEncodingTools.MONO8) || encoding.equals(ImageEncodingTools.MONO16))
         return OpenCVColorFormats.GRAY;
      if (encoding.equals(ImageEncodingTools.BGR8))
         return OpenCVColorFormats.BGR;
      if (encoding.equals(ImageEncodingTools.RGB8))
         return OpenCVColorFormats.RGB;
      if (encoding.equals(ImageEncodingTools.BGRA8))
         return OpenCVColorFormats.BGRA;
      if (encoding.equals(ImageEncodingTools.RGBA8))
         return OpenCVColorFormats.RGBA;
      if (encoding.equals(ImageEncodingTools.YUV422))
         return OpenCVColorFormats.YUV422;

      if (encoding.equals(ImageEncodingTools.BAYER_RGGB8))
         return OpenCVColorFormats.BAYER_RGGB;
      if (encoding.equals(ImageEncodingTools.BAYER_BGGR8))
         return OpenCVColorFormats.BAYER_BGGR;
      if (encoding.equals(ImageEncodingTools.BAYER_GBRG8))
         return OpenCVColorFormats.BAYER_GBRG;
      if (encoding.equals(ImageEncodingTools.BAYER_GRBG8))
         return OpenCVColorFormats.BAYER_GRBG;

      // We don't support conversions to/from other types
      return OpenCVColorFormats.INVALID;
   }

   public static Vector<Integer> getColorConversionCode(String sourceEncodingString, String destinationEncodingString)
   {
      OpenCVColorFormats sourceEncoding = getColorFormat(sourceEncodingString);
      OpenCVColorFormats destinationEncoding = getColorFormat(destinationEncodingString);

      boolean isSourceColorFormat = ImageEncodingTools.isColor(sourceEncodingString)
                                    || ImageEncodingTools.isMono(sourceEncodingString)
                                    || ImageEncodingTools.isBayer(sourceEncodingString)
                                    || (sourceEncodingString.equalsIgnoreCase(ImageEncodingTools.YUV422));

      boolean isDestinationColorFormat = ImageEncodingTools.isColor(destinationEncodingString)
                                         || ImageEncodingTools.isMono(destinationEncodingString)
                                         || ImageEncodingTools.isBayer(destinationEncodingString)
                                         || (destinationEncodingString.equalsIgnoreCase(ImageEncodingTools.YUV422));

      boolean isNumberOfChannelsTheSame = ImageEncodingTools.numChannels(sourceEncodingString) == ImageEncodingTools.numChannels(destinationEncodingString);

      // If we have no color info in the source, we can only convert to the same format which
      // was resolved in the previous condition. Otherwise, fail
      if (!isSourceColorFormat)
      {
         if (isDestinationColorFormat)
            throw new RuntimeException("[" + sourceEncodingString + "] is not a color format. but [" + destinationEncodingString + "] is. The conversion does not make sense");
         if (!isNumberOfChannelsTheSame)
            throw new RuntimeException("[" + sourceEncodingString + "] and [" + destinationEncodingString + "] do not have the same number of channel");
         return new Vector<>(1, SAME_FORMAT);
      }

      // If we are converting from a color type to a non color type, we can only do so if we stick
      // to the number of channels
      if (!isDestinationColorFormat)
      {
         if (!isNumberOfChannelsTheSame)
            throw new RuntimeException("[" + sourceEncodingString + "] is a color format but [" + destinationEncodingString + "] "
                                       + "is not so they must have the same OpenCV type, CV_8UC3, CV16UC1 ....");
         return new Vector<>(1, SAME_FORMAT);
      }

      // If we are converting from a color type to another type, then everything is fine
      final Map<Pair<OpenCVColorFormats, OpenCVColorFormats>, Vector<Integer>> conversionCodes = getConversionCodes();

      Pair<OpenCVColorFormats, OpenCVColorFormats> key = Pair.of(sourceEncoding, destinationEncoding);
      Vector<Integer> codesVector = conversionCodes.get(key);

      if (codesVector == null)
         throw new RuntimeException("Unsupported conversion from [" + sourceEncodingString + "] to [" + destinationEncodingString + "]");

      // And deal with depth differences if the colors are different
      if (ImageEncodingTools.bitDepth(sourceEncodingString) != ImageEncodingTools.bitDepth(destinationEncodingString) && (getColorFormat(sourceEncodingString) != getColorFormat(destinationEncodingString)))
         codesVector.add(SAME_FORMAT);

      return codesVector;
   }
}
