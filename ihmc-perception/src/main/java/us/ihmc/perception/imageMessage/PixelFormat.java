package us.ihmc.perception.imageMessage;

import org.apache.commons.lang3.NotImplementedException;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_cudaimgproc;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.communication.ros2.ROS2SRTStreamTopicPair.ImageType;

import static org.bytedeco.opencv.global.opencv_imgproc.*;

public enum PixelFormat
{
   BGR8(1, 3, COLOR_BGR2RGBA, COLOR_RGBA2BGR),                 // 24 bits per pixel, in BGR order
   BGRA8(1, 4, COLOR_BGRA2RGBA, COLOR_RGBA2BGRA),              // 32 bits per pixel, in BGRA order
   RGB8(1, 3, COLOR_RGB2RGBA, COLOR_RGBA2RGB),                 // 24 bits per pixel, in RGB order
   RGBA8(1, 4, -1, -1),                                        // 32 bits per pixel, in RGBA order
   YUV_I420(1, 1, COLOR_YUV2RGBA_I420, COLOR_RGBA2YUV_I420),   // YUV420 format
   GRAY8(1, 1, COLOR_GRAY2RGBA, COLOR_RGBA2GRAY),              // monochrome
   GRAY16(2, 1, -1, -1);                                       // aka depth

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

   PixelFormat(int bytesPerElement, int elementsPerPixel, int opencvToRGBAConversion, int opencvFromRGBAConversion)
   {
      this.bytesPerElement = bytesPerElement;
      this.elementsPerPixel = elementsPerPixel;
      bytesPerPixel = bytesPerElement * (long) elementsPerPixel;
      this.opencvToRGBAConversion = opencvToRGBAConversion;
      this.opencvFromRGBAConversion = opencvFromRGBAConversion;
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

   public static PixelFormat fromImageType(ImageType imageType)
   {
      return switch (imageType)
      {
         case COLOR -> BGR8;
         case DEPTH -> GRAY16;
      };
   }

   public int toOpenCVType()
   {
      return switch ((int) bytesPerElement)
      {  // TODO: What about floating point types?
         case 1 -> opencv_core.CV_8UC(elementsPerPixel);
         case 2 -> opencv_core.CV_16UC(elementsPerPixel);
         default -> throw new NotImplementedException("Tomasz has been too busy (or lazy) to implement this. Feel free to do it yourself!");
      };
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
