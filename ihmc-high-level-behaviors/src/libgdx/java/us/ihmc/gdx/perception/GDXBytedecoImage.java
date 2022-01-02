package us.ihmc.gdx.perception;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencl._cl_mem;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.perception.OpenCLManager;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class GDXBytedecoImage
{
   private final ByteBuffer backingDirectByteBuffer;
   private final BytePointer bytedecoByteBufferPointer;
   private final Mat bytedecoOpenCVMat;
   private final int openCLChannelOrder;
   private final int openCLChannelDataType;
   private final int imageWidth;
   private final int imageHeight;
   private _cl_mem openCLImageObject;

   public GDXBytedecoImage(int imageWidth, int imageHeight, int cvMatType)
   {
      this(imageWidth, imageHeight, cvMatType, null);
   }

   public GDXBytedecoImage(int imageWidth, int imageHeight, int cvMatType, ByteBuffer backingDirectByteBuffer)
   {
      this.imageWidth = imageWidth;
      this.imageHeight = imageHeight;

      int bytesPerPixel = 0;
      if (cvMatType == opencv_core.CV_16UC1)
      {
         bytesPerPixel = 2;
         openCLChannelOrder = OpenCL.CL_DEPTH;
         openCLChannelDataType = OpenCL.CL_UNSIGNED_INT16;
      }
      else if (cvMatType == opencv_core.CV_32FC1)
      {
         bytesPerPixel = 4;
         openCLChannelOrder = OpenCL.CL_DEPTH;
         openCLChannelDataType = OpenCL.CL_FLOAT;
      }
      else if (cvMatType == opencv_core.CV_8UC1)
      {
         bytesPerPixel = 1;
         openCLChannelOrder = OpenCL.CL_DEPTH;
         openCLChannelDataType = OpenCL.CL_UNSIGNED_INT8;
      }
      else if (cvMatType == opencv_core.CV_8UC4)
      {
         bytesPerPixel = 4;
         openCLChannelOrder = OpenCL.CL_RGBA;
         openCLChannelDataType = OpenCL.CL_UNSIGNED_INT8;
      }
      else
      {
         throw new RuntimeException("Implement bytesPerPixel for this type!");
      }

      if (backingDirectByteBuffer == null)
      {
         this.backingDirectByteBuffer = ByteBuffer.allocateDirect(imageWidth * imageHeight * bytesPerPixel);
         this.backingDirectByteBuffer.order(ByteOrder.nativeOrder());
      }
      else
      {
         this.backingDirectByteBuffer = backingDirectByteBuffer;
      }

      bytedecoByteBufferPointer = new BytePointer(this.backingDirectByteBuffer);
      bytedecoOpenCVMat = new Mat(imageHeight, imageWidth, cvMatType, bytedecoByteBufferPointer);
   }

   public void createOpenCLImage(OpenCLManager openCLManager, int flags)
   {
      openCLImageObject = openCLManager.createImage(flags, openCLChannelOrder, openCLChannelDataType, imageWidth, imageHeight, bytedecoByteBufferPointer);
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

   public _cl_mem getOpenCLImageObject()
   {
      return openCLImageObject;
   }
}
