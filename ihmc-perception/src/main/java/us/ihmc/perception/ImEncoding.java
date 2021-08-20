package us.ihmc.perception;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;

import java.util.*;

public class ImEncoding
{
   protected static final int SAME_FORMAT = -1;

   public static int getCvType(String encoding)
   {
      encoding = encoding.toLowerCase();
      if (encoding.equals(ImageEncodings.BGR8))
         return opencv_core.CV_8UC3;
      if (encoding.equals(ImageEncodings.MONO8))
         return opencv_core.CV_8UC1;
      if (encoding.equals(ImageEncodings.RGB8))
         return opencv_core.CV_8UC3;
      if (encoding.equals(ImageEncodings.MONO16))
         return opencv_core.CV_16UC1;
      if (encoding.equals(ImageEncodings.BGR16))
         return opencv_core.CV_16UC3;
      if (encoding.equals(ImageEncodings.RGB16))
         return opencv_core.CV_16UC3;
      if (encoding.equals(ImageEncodings.BGRA8))
         return opencv_core.CV_8UC4;
      if (encoding.equals(ImageEncodings.RGBA8))
         return opencv_core.CV_8UC4;
      if (encoding.equals(ImageEncodings.BGRA16))
         return opencv_core.CV_16UC4;
      if (encoding.equals(ImageEncodings.RGBA16))
         return opencv_core.CV_16UC4;

      // For bayer, return one-channel
      if (encoding.equals(ImageEncodings.BAYER_RGGB8))
         return opencv_core.CV_8UC1;
      if (encoding.equals(ImageEncodings.BAYER_BGGR8))
         return opencv_core.CV_8UC1;
      if (encoding.equals(ImageEncodings.BAYER_GBRG8))
         return opencv_core.CV_8UC1;
      if (encoding.equals(ImageEncodings.BAYER_GRBG8))
         return opencv_core.CV_8UC1;
      if (encoding.equals(ImageEncodings.BAYER_RGGB16))
         return opencv_core.CV_16UC1;
      if (encoding.equals(ImageEncodings.BAYER_BGGR16))
         return opencv_core.CV_16UC1;
      if (encoding.equals(ImageEncodings.BAYER_GBRG16))
         return opencv_core.CV_16UC1;
      if (encoding.equals(ImageEncodings.BAYER_GRBG16))
         return opencv_core.CV_16UC1;

      // Miscellaneous
      if (encoding.equals(ImageEncodings.YUV422))
         return opencv_core.CV_8UC2;

      encoding = encoding.toUpperCase();

      //macro code
      if (encoding.equals(ImageEncodings.TYPE_8UC1))
         return opencv_core.CV_8UC1;
      if (encoding.equals(ImageEncodings.TYPE_8UC2))
         return opencv_core.CV_8UC2;
      if (encoding.equals(ImageEncodings.TYPE_8UC3))
         return opencv_core.CV_8UC3;
      if (encoding.equals(ImageEncodings.TYPE_8UC4))
         return opencv_core.CV_8UC4;
      if (encoding.equals(ImageEncodings.TYPE_8SC1))
         return opencv_core.CV_8SC1;
      if (encoding.equals(ImageEncodings.TYPE_8SC2))
         return opencv_core.CV_8SC2;
      if (encoding.equals(ImageEncodings.TYPE_8SC3))
         return opencv_core.CV_8SC3;
      if (encoding.equals(ImageEncodings.TYPE_8SC4))
         return opencv_core.CV_8SC4;
      if (encoding.equals(ImageEncodings.TYPE_16UC1))
         return opencv_core.CV_16UC1;
      if (encoding.equals(ImageEncodings.TYPE_16UC2))
         return opencv_core.CV_16UC2;
      if (encoding.equals(ImageEncodings.TYPE_16UC3))
         return opencv_core.CV_16UC3;
      if (encoding.equals(ImageEncodings.TYPE_16UC4))
         return opencv_core.CV_16UC4;
      if (encoding.equals(ImageEncodings.TYPE_16SC1))
         return opencv_core.CV_16SC1;
      if (encoding.equals(ImageEncodings.TYPE_16SC2))
         return opencv_core.CV_16SC2;
      if (encoding.equals(ImageEncodings.TYPE_16SC3))
         return opencv_core.CV_16SC3;
      if (encoding.equals(ImageEncodings.TYPE_16SC4))
         return opencv_core.CV_16SC4;
      if (encoding.equals(ImageEncodings.TYPE_32SC1))
         return opencv_core.CV_32SC1;
      if (encoding.equals(ImageEncodings.TYPE_32SC2))
         return opencv_core.CV_32SC2;
      if (encoding.equals(ImageEncodings.TYPE_32SC3))
         return opencv_core.CV_32SC3;
      if (encoding.equals(ImageEncodings.TYPE_32SC4))
         return opencv_core.CV_32SC4;
      if (encoding.equals(ImageEncodings.TYPE_32FC1))
         return opencv_core.CV_32FC1;
      if (encoding.equals(ImageEncodings.TYPE_32FC2))
         return opencv_core.CV_32FC2;
      if (encoding.equals(ImageEncodings.TYPE_32FC3))
         return opencv_core.CV_32FC3;
      if (encoding.equals(ImageEncodings.TYPE_32FC4))
         return opencv_core.CV_32FC4;
      if (encoding.equals(ImageEncodings.TYPE_64FC1))
         return opencv_core.CV_64FC1;
      if (encoding.equals(ImageEncodings.TYPE_64FC2))
         return opencv_core.CV_64FC2;
      if (encoding.equals(ImageEncodings.TYPE_64FC3))
         return opencv_core.CV_64FC3;
      if (encoding.equals(ImageEncodings.TYPE_64FC4))
         return opencv_core.CV_64FC4;

      throw new RuntimeException("Unrecognized image encoding [" + encoding + "]");
   }

   protected static int safeLongToInt(long l)
   {
      if (l < Integer.MIN_VALUE || l > Integer.MAX_VALUE)
      {
         throw new IllegalArgumentException(l + " cannot be cast to int without changing its value.");
      }
      return (int) l;
   }

   protected static Map<Pair<Encoding, Encoding>, Vector<Integer>> getConversionCodes()
   {
      Map<Pair<Encoding, Encoding>, Vector<Integer>> conversionCodeMap = new HashMap<>();

      for (int i = 0; i <= 5; ++i)
      {
         conversionCodeMap.put(new Pair<>(Encoding.valueOf(i), Encoding.valueOf(i)), new Vector<>(Collections.singletonList(SAME_FORMAT)));
      }

      conversionCodeMap.put(new Pair<>(Encoding.GRAY, Encoding.RGB), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_GRAY2RGB)));
      conversionCodeMap.put(new Pair<>(Encoding.GRAY, Encoding.BGR), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_GRAY2BGR)));
      conversionCodeMap.put(new Pair<>(Encoding.GRAY, Encoding.RGBA), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_GRAY2RGBA)));
      conversionCodeMap.put(new Pair<>(Encoding.GRAY, Encoding.BGRA), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_GRAY2BGRA)));

      conversionCodeMap.put(new Pair<>(Encoding.RGB, Encoding.GRAY), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_RGB2GRAY)));
      conversionCodeMap.put(new Pair<>(Encoding.RGB, Encoding.BGR), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_RGB2BGR)));
      conversionCodeMap.put(new Pair<>(Encoding.RGB, Encoding.RGBA), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_RGB2RGBA)));
      conversionCodeMap.put(new Pair<>(Encoding.RGB, Encoding.BGRA), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_RGB2BGRA)));

      conversionCodeMap.put(new Pair<>(Encoding.BGR, Encoding.GRAY), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_BGR2GRAY)));
      conversionCodeMap.put(new Pair<>(Encoding.BGR, Encoding.RGB), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_BGR2RGB)));
      conversionCodeMap.put(new Pair<>(Encoding.BGR, Encoding.RGBA), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_BGR2RGBA)));
      conversionCodeMap.put(new Pair<>(Encoding.BGR, Encoding.BGRA), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_BGR2BGRA)));

      conversionCodeMap.put(new Pair<>(Encoding.RGBA, Encoding.GRAY), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_RGBA2GRAY)));
      conversionCodeMap.put(new Pair<>(Encoding.RGBA, Encoding.RGB), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_RGBA2RGB)));
      conversionCodeMap.put(new Pair<>(Encoding.RGBA, Encoding.BGR), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_RGBA2BGR)));
      conversionCodeMap.put(new Pair<>(Encoding.RGBA, Encoding.BGRA), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_RGBA2BGRA)));

      conversionCodeMap.put(new Pair<>(Encoding.BGRA, Encoding.GRAY), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_BGRA2GRAY)));
      conversionCodeMap.put(new Pair<>(Encoding.BGRA, Encoding.RGB), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_BGRA2RGB)));
      conversionCodeMap.put(new Pair<>(Encoding.BGRA, Encoding.BGR), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_BGRA2BGR)));
      conversionCodeMap.put(new Pair<>(Encoding.BGRA, Encoding.RGBA), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_BGRA2RGBA)));

      conversionCodeMap.put(new Pair<>(Encoding.YUV422, Encoding.GRAY), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_YUV2GRAY_UYVY)));
      conversionCodeMap.put(new Pair<>(Encoding.YUV422, Encoding.RGB), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_YUV2RGB_UYVY)));
      conversionCodeMap.put(new Pair<>(Encoding.YUV422, Encoding.BGR), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_YUV2BGR_UYVY)));
      conversionCodeMap.put(new Pair<>(Encoding.YUV422, Encoding.RGBA), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_YUV2RGBA_UYVY)));
      conversionCodeMap.put(new Pair<>(Encoding.YUV422, Encoding.BGRA), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_YUV2BGRA_UYVY)));

      // Deal with Bayer
      conversionCodeMap.put(new Pair<>(Encoding.BAYER_RGGB, Encoding.GRAY), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_BayerBG2GRAY)));
      conversionCodeMap.put(new Pair<>(Encoding.BAYER_RGGB, Encoding.RGB), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_BayerBG2RGB)));
      conversionCodeMap.put(new Pair<>(Encoding.BAYER_RGGB, Encoding.BGR), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_BayerBG2BGR)));

      conversionCodeMap.put(new Pair<>(Encoding.BAYER_BGGR, Encoding.GRAY), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_BayerRG2GRAY)));
      conversionCodeMap.put(new Pair<>(Encoding.BAYER_BGGR, Encoding.RGB), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_BayerRG2RGB)));
      conversionCodeMap.put(new Pair<>(Encoding.BAYER_BGGR, Encoding.BGR), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_BayerRG2BGR)));

      conversionCodeMap.put(new Pair<>(Encoding.BAYER_GBRG, Encoding.GRAY), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_BayerGR2GRAY)));
      conversionCodeMap.put(new Pair<>(Encoding.BAYER_GBRG, Encoding.RGB), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_BayerGR2RGB)));
      conversionCodeMap.put(new Pair<>(Encoding.BAYER_GBRG, Encoding.BGR), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_BayerGR2BGR)));

      conversionCodeMap.put(new Pair<>(Encoding.BAYER_GRBG, Encoding.GRAY), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_BayerGB2GRAY)));
      conversionCodeMap.put(new Pair<>(Encoding.BAYER_GRBG, Encoding.RGB), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_BayerGB2RGB)));
      conversionCodeMap.put(new Pair<>(Encoding.BAYER_GRBG, Encoding.BGR), new Vector<>(Collections.singletonList(opencv_imgproc.COLOR_BayerGB2BGR)));

      return conversionCodeMap;
   }

   public static Encoding getEncoding(String encoding)
   {
      encoding = encoding.toLowerCase();

      if (encoding.equals(ImageEncodings.MONO8))
         return Encoding.GRAY;
      if (encoding.equals(ImageEncodings.BGR8))
         return Encoding.BGR;
      if (encoding.equals(ImageEncodings.RGB8))
         return Encoding.RGB;
      if (encoding.equals(ImageEncodings.BGRA8))
         return Encoding.BGRA;
      if (encoding.equals(ImageEncodings.RGBA8))
         return Encoding.RGBA;
      if (encoding.equals(ImageEncodings.YUV422))
         return Encoding.YUV422;

      if (encoding.equals(ImageEncodings.BAYER_RGGB8))
         return Encoding.BAYER_RGGB;
      if (encoding.equals(ImageEncodings.BAYER_BGGR8))
         return Encoding.BAYER_BGGR;
      if (encoding.equals(ImageEncodings.BAYER_GBRG8))
         return Encoding.BAYER_GBRG;
      if (encoding.equals(ImageEncodings.BAYER_GRBG8))
         return Encoding.BAYER_GRBG;

      // We don't support conversions to/from other types
      return Encoding.INVALID;
   }

   public static Vector<Integer> getConversionCode(String sourceEncodingString, String destinationEncodingString)
   {
      Encoding sourceEncoding = getEncoding(sourceEncodingString);
      Encoding destinationEncoding = getEncoding(destinationEncodingString);

      boolean isSourceColorFormat = ImageEncodings.isColor(sourceEncodingString)
                                 || ImageEncodings.isMono(sourceEncodingString)
                                 || ImageEncodings.isBayer(sourceEncodingString)
                                 || (sourceEncodingString.equalsIgnoreCase(ImageEncodings.YUV422));

      boolean isDestinationColorFormat = ImageEncodings.isColor(destinationEncodingString)
                                      || ImageEncodings.isMono(destinationEncodingString)
                                      || ImageEncodings.isBayer(destinationEncodingString)
                                      || (destinationEncodingString.equalsIgnoreCase(ImageEncodings.YUV422));

      boolean isNumberOfChannelsTheSame = ImageEncodings.numChannels(sourceEncodingString) == ImageEncodings.numChannels(destinationEncodingString);

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
      final Map<Pair<Encoding, Encoding>, Vector<Integer>> conversionCodes = getConversionCodes();

      Pair<Encoding, Encoding> key = new Pair<>(sourceEncoding, destinationEncoding);
      Vector<Integer> codesVector = conversionCodes.get(key);

      if (codesVector == null)
         throw new RuntimeException("Unsupported conversion from [" + sourceEncodingString + "] to [" + destinationEncodingString + "]");

      // And deal with depth differences if the colors are different
      if (ImageEncodings.bitDepth(sourceEncodingString) != ImageEncodings.bitDepth(destinationEncodingString) && (getEncoding(sourceEncodingString) != getEncoding(destinationEncodingString)))
         codesVector.add(SAME_FORMAT);

      return codesVector;
   }
}
