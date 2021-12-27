package us.ihmc.gdx.perception;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class GDXBytedecoImage
{
   private ByteBuffer backingDirectByteBuffer;
   private BytePointer bytedecoByteBufferPointer;
   private Mat bytedecoOpenCVMat;

   public GDXBytedecoImage(int imageWidth, int imageHeight, int cvMatType)
   {
      int bytesPerPixel = 0;
      if (cvMatType == opencv_core.CV_16UC1)
         bytesPerPixel = 2;
      else if (cvMatType == opencv_core.CV_32FC1)
         bytesPerPixel = 4;
      else if (cvMatType == opencv_core.CV_8UC4)
         bytesPerPixel = 4;

      backingDirectByteBuffer = ByteBuffer.allocateDirect(imageWidth * imageHeight * bytesPerPixel);
      backingDirectByteBuffer.order(ByteOrder.nativeOrder());

      setup(imageWidth, imageHeight, cvMatType);
   }

   public GDXBytedecoImage(int imageWidth, int imageHeight, int cvMatType, ByteBuffer backingDirectByteBuffer)
   {
      this.backingDirectByteBuffer = backingDirectByteBuffer;

      setup(imageWidth, imageHeight, cvMatType);
   }

   private void setup(int imageWidth, int imageHeight, int cvMatType)
   {
      bytedecoByteBufferPointer = new BytePointer(backingDirectByteBuffer);
      bytedecoOpenCVMat = new Mat(imageHeight, imageWidth, cvMatType, bytedecoByteBufferPointer);
   }

   public void rewind()
   {
      backingDirectByteBuffer.rewind();
      bytedecoByteBufferPointer.position(0);
   }

   public Mat getBytedecoOpenCVMat()
   {
      return bytedecoOpenCVMat;
   }

   public ByteBuffer getBackingDirectByteBuffer()
   {
      return backingDirectByteBuffer;
   }

   public BytePointer getBytedecoByteBufferPointer()
   {
      return bytedecoByteBufferPointer;
   }
}
