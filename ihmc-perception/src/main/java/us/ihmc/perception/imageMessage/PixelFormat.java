package us.ihmc.perception.imageMessage;

import org.apache.commons.lang3.NotImplementedException;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_cudaimgproc;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ImageMessage;

import static org.bytedeco.ffmpeg.global.avutil.*;
import static org.bytedeco.opencv.global.opencv_imgproc.*;

public enum PixelFormat
{
   BGR8(1, 3, COLOR_BGR2RGBA, COLOR_RGBA2BGR, AV_PIX_FMT_BGR24),     // 24 bits per pixel, in BGR order
   BGRA8(1, 4, COLOR_BGRA2RGBA, COLOR_RGBA2BGRA, AV_PIX_FMT_BGRA),   // 32 bits per pixel, in BGRA order
   RGB8(1, 3, COLOR_RGB2RGBA, COLOR_RGBA2RGB, AV_PIX_FMT_RGB24),     // 24 bits per pixel, in RGB order
   RGBA8(1, 4, -1, -1, AV_PIX_FMT_RGBA),                             // 32 bits per pixel, in RGBA order
   YUV_I420(1, 1, COLOR_YUV2RGBA_I420, COLOR_RGBA2YUV_I420, -1),     // YUV420 format
   YUV_444P16(2, 3, -1, -1, AV_PIX_FMT_YUV444P16),                   // 16 bit planar YUV444, 48 bits per pixel.
   GRAY8(1, 1, COLOR_GRAY2RGBA, COLOR_RGBA2GRAY, AV_PIX_FMT_GRAY8),  // monochrome
   GRAY16(2, 1, -1, -1, AV_PIX_FMT_GRAY16),                          // aka depth
   UNKNOWN(-1, -1, -1, -1, -1);

   /** Equivalent to {@code elemsize} of the corresponding OpenCV type */
   public final long bytesPerElement;
   /** How many elements represent a pixel. E.g. GRAY has 1 element per pixel, BGR has 3, and BGRA has 4. */
   public final int elementsPerPixel;
   /** Number of bytes that represent 1 pixel */
   public final long bytesPerPixel;
   /** One of {@code opencv_imgproc.COLOR_*} to achieve a conversion from this pixel format to RGBA. -1 if the conversion is not possible */
   private final int opencvToRGBAConversion;
   /** One of {@code opencv_imgproc.COLOR_*} to achieve a conversion from RGBA to this pixel format. -1 if the conversion is not possible */
   private final int opencvFromRGBAConversion;
   /** One of {@code avutil.AV_PIX_FMT_*} that corresponds to this pixel format. -1 if there is no corresponding pixel format */
   private final int correspondingAvPixelFormat;
   PixelFormat(int bytesPerElement, int elementsPerPixel, int opencvToRGBAConversion, int opencvFromRGBAConversion, int correspondingAvPixelFormat)
   {
      this.bytesPerElement = bytesPerElement;
      this.elementsPerPixel = elementsPerPixel;
      bytesPerPixel = bytesPerElement * (long) elementsPerPixel;
      this.opencvToRGBAConversion = opencvToRGBAConversion;
      this.opencvFromRGBAConversion = opencvFromRGBAConversion;
      this.correspondingAvPixelFormat = correspondingAvPixelFormat;
   }

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static PixelFormat fromByte(byte pixelFormatAsByte)
   {
      return values()[pixelFormatAsByte];
   }

   public static PixelFormat fromImageMessage(ImageMessage imageMessage)
   {
      return fromByte(imageMessage.getPixelFormat());
   }

   public static PixelFormat fromFFmpegPixelFormat(int avPixelFormat)
   {
      for (PixelFormat format : values())
      {
         if (format.correspondingAvPixelFormat == avPixelFormat)
            return format;
      }

      return UNKNOWN;
   }

   public int toOpenCVType()
   {
      return toOpenCVType(elementsPerPixel);
   }

   public int toOpenCVType(int numberOfChannels)
   {
      return switch ((int) bytesPerElement)
      {
         case 1 -> opencv_core.CV_8UC(numberOfChannels);
         case 2 -> opencv_core.CV_16UC(numberOfChannels);
         default -> throw new NotImplementedException("Tomasz has been too busy (or lazy) to implement this. Feel free to do it yourself!");
      };
   }

   public int toFFmpegPixelFormat()
   {
      return correspondingAvPixelFormat;
   }

   /**
    * Converts the pixel format of {@code source} from {@code this} pixel format to the {@code targetPixelFormat}.
    * This method uses a two-step process of first converting the image to RGBA, then to the target pixel format.
    * Although this method is convenient, it is not recommended when fast color conversions are required.
    * In such a case, use {@code opencv_imgproc.cvtColor(source, destination, colorConversion)} instead to accomplish the conversion in one step.
    * @param source Source image in {@code this} pixel format.
    * @param destination Destination image, will be in {@code targetPixelFormat}.
    * @param targetPixelFormat The desired pixel format
    * @return {@code true} if the conversion was successful, {@code false} if the conversion could not be done.
    */
   public boolean convertToPixelFormat(Mat source, Mat destination, PixelFormat targetPixelFormat)
   {
      if (this == targetPixelFormat)
      {
         source.copyTo(destination);
         return true;
      }

      return this.convertToRGBA(source, destination) && targetPixelFormat.convertFromRGBA(destination, destination);
   }

   /**
    * Converts the pixel format of {@code source} from {@code this} pixel format to the {@code targetPixelFormat}.
    * This method uses a two-step process of first converting the image to RGBA, then to the target pixel format.
    * Although this method is convenient, it is not recommended when fast color conversions are required.
    * In such a case, use {@code opencv_cudaimgproc.cvtColor(source, destination, colorConversion)} instead to accomplish the conversion in one step.
    * @param source Source image in {@code this} pixel format.
    * @param destination Destination image, will be in {@code targetPixelFormat}.
    * @param targetPixelFormat The desired pixel format
    * @return {@code true} if the conversion was successful, {@code false} if the conversion could not be done.
    */
   public boolean convertToPixelFormat(GpuMat source, GpuMat destination, PixelFormat targetPixelFormat)
   {
      if (this == targetPixelFormat)
      {
         source.copyTo(destination);
         return true;
      }

      return this.convertToRGBA(source, destination) && targetPixelFormat.convertFromRGBA(destination, destination);
   }

   public boolean convertToRGBA(Mat source, Mat destination)
   {
      if (opencvToRGBAConversion < 0)
      {
         source.copyTo(destination);
         return this == RGBA8;
      }

      opencv_imgproc.cvtColor(source, destination, opencvToRGBAConversion);
      return true;
   }

   public boolean convertToRGBA(GpuMat source, GpuMat destination)
   {
      if (opencvToRGBAConversion < 0)
      {
         source.copyTo(destination);
         return this == RGBA8;
      }

      if (this == YUV_I420)
      {  // YUV I420 conversions are not supported on the GPU :(
         try (Mat sourceCopy = new Mat();
              Mat tempDestination = new Mat())
         {
            source.download(sourceCopy);
            opencv_imgproc.cvtColor(sourceCopy, tempDestination, opencvToRGBAConversion);
            destination.upload(tempDestination);
         }
      }
      else
         opencv_cudaimgproc.cvtColor(source, destination, opencvToRGBAConversion);

      return true;
   }

   public boolean convertFromRGBA(Mat source, Mat destination)
   {
      if (opencvFromRGBAConversion < 0)
      {
         source.copyTo(destination);
         return this == RGBA8;
      }

      opencv_imgproc.cvtColor(source, destination, opencvFromRGBAConversion);
      return true;
   }

   public boolean convertFromRGBA(GpuMat source, GpuMat destination)
   {
      if (opencvFromRGBAConversion < 0)
      {
         source.copyTo(destination);
         return this == RGBA8;
      }

      if (this == YUV_I420)
      {  // YUV I420 conversions are not supported on the GPU :(
         try (Mat sourceCopy = new Mat();
              Mat tempDestination = new Mat())
         {
            source.download(sourceCopy);
            opencv_imgproc.cvtColor(sourceCopy, tempDestination, opencvFromRGBAConversion);
            destination.upload(tempDestination);
         }
      }
      else
         opencv_cudaimgproc.cvtColor(source, destination, opencvFromRGBAConversion);

      return true;
   }
}
