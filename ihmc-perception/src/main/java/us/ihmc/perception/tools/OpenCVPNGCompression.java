package us.ihmc.perception.tools;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.opencv_core.Mat;

/**
 * PNG which is lossless and handles single channel images, which
 * makes it useful for depth images.
 */
public class OpenCVPNGCompression
{
   private final IntPointer compressionParameters = new IntPointer(opencv_imgcodecs.IMWRITE_PNG_COMPRESSION, 1);
   private BytePointer pngImageBytePointer;

   public void allocate(int decompressedImageBytes)
   {
      pngImageBytePointer = new BytePointer(decompressedImageBytes);
   }

   public void compress(Mat imageToCompress)
   {
      opencv_imgcodecs.imencode(".png", imageToCompress, pngImageBytePointer, compressionParameters);
   }

   public BytePointer getCompressedData()
   {
      return pngImageBytePointer;
   }
}
