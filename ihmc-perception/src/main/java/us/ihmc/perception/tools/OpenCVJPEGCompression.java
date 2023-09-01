package us.ihmc.perception.tools;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;

/**
 * JPEG which lossy but achieves really high compression ratio
 * really quickly, which makes it great for color video streaming.
 */
public class OpenCVJPEGCompression
{
   private final IntPointer compressionParameters;
   private BytePointer jpegImageBytePointer;

   private Mat yuv420Image;

   /**
    * JPEG 95 quality.
    */
   public OpenCVJPEGCompression()
   {
      this(95);
   }

   /**
    * @param quality 0 - 100 (default 95)
    */
   public OpenCVJPEGCompression(int quality)
   {
      compressionParameters = new IntPointer(opencv_imgcodecs.IMWRITE_JPEG_QUALITY, quality);
   }

   public void allocate(int decompressedImageBytes)
   {
      jpegImageBytePointer = new BytePointer(decompressedImageBytes);
      yuv420Image = new Mat();
   }

   public void compressRGB(Mat rgbImageToCompress)
   {
      opencv_imgproc.cvtColor(rgbImageToCompress, yuv420Image, opencv_imgproc.COLOR_RGB2YUV_I420);
      compressYUVI420(yuv420Image);
   }

   public void compressYUVI420(Mat yuvI420ImageToCompress)
   {
      opencv_imgcodecs.imencode(".jpg", yuvI420ImageToCompress, jpegImageBytePointer, compressionParameters);
   }

   public BytePointer getCompressedData()
   {
      return jpegImageBytePointer;
   }
}
