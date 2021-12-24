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

   public GDXBytedecoImage(int width, int height, int cvMatType)
   {
      int bytesPerPixel = 0;
      if (cvMatType == opencv_core.CV_16UC1)
         bytesPerPixel = 2;
      else if (cvMatType == opencv_core.CV_32FC1)
         bytesPerPixel = 4;
      else if (cvMatType == opencv_core.CV_8UC4)
         bytesPerPixel = 4;

      backingDirectByteBuffer = ByteBuffer.allocateDirect(width * height * bytesPerPixel);
      backingDirectByteBuffer.order(ByteOrder.nativeOrder());

      setup(width, height, cvMatType);
   }

   public GDXBytedecoImage(int width, int height, int cvMatType, ByteBuffer backingDirectByteBuffer)
   {
      this.backingDirectByteBuffer = backingDirectByteBuffer;

      setup(width, height, cvMatType);
   }

   private void setup(int width, int height, int cvMatType)
   {
      bytedecoByteBufferPointer = new BytePointer(backingDirectByteBuffer);
      bytedecoOpenCVMat = new Mat(height, width, cvMatType, bytedecoByteBufferPointer);
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
