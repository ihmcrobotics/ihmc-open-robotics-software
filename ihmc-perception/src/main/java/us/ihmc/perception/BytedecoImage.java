package us.ihmc.perception;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencl._cl_mem;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class BytedecoImage
{
   private ByteBuffer backingDirectByteBuffer;
   private BytePointer bytedecoByteBufferPointer;
   private Mat bytedecoOpenCVMat;
   private final int openCLChannelOrder;
   private final int openCLChannelDataType;
   private int imageWidth;
   private int imageHeight;
   private final int cvMatType;
   private int bytesPerPixel;
   private _cl_mem openCLImageObject;
   private int openCLImageObjectFlags;
   private final boolean isBackedByExternalByteBuffer;

   public BytedecoImage(int imageWidth, int imageHeight, int cvMatType)
   {
      this(imageWidth, imageHeight, cvMatType, null);
   }

   public BytedecoImage(int imageWidth, int imageHeight, int cvMatType, ByteBuffer backingDirectByteBuffer)
   {
      this(imageWidth, imageHeight, cvMatType, backingDirectByteBuffer, null);
   }

   public BytedecoImage(Mat suppliedMat)
   {
      this(BytedecoOpenCVTools.getImageWidth(suppliedMat),
           BytedecoOpenCVTools.getImageHeight(suppliedMat),
           suppliedMat.type(),
           suppliedMat.data().asByteBuffer(),
           suppliedMat);
   }

   public BytedecoImage(int imageWidth, int imageHeight, int cvMatType, ByteBuffer backingDirectByteBuffer, Mat suppliedMat)
   {
      this.imageWidth = imageWidth;
      this.imageHeight = imageHeight;
      this.cvMatType = cvMatType;

      bytesPerPixel = 0;
      if (cvMatType == opencv_core.CV_16UC1)
      {
         bytesPerPixel = 2;
         openCLChannelOrder = OpenCL.CL_R;
         openCLChannelDataType = OpenCL.CL_UNSIGNED_INT16;
      }
      else if (cvMatType == opencv_core.CV_32FC1)
      {
         bytesPerPixel = 4;
         openCLChannelOrder = OpenCL.CL_R;
         openCLChannelDataType = OpenCL.CL_FLOAT;
      }
      else if (cvMatType == opencv_core.CV_32FC(6))
      {
         bytesPerPixel = 4 * 6;
         openCLChannelOrder = OpenCL.CL_R;
         openCLChannelDataType = OpenCL.CL_FLOAT;
      }
      else if (cvMatType == opencv_core.CV_8UC1)
      {
         bytesPerPixel = 1;
         openCLChannelOrder = OpenCL.CL_R;
         openCLChannelDataType = OpenCL.CL_UNSIGNED_INT8;
      }
      else if (cvMatType == opencv_core.CV_8UC3)
      {
         bytesPerPixel = 3;
         openCLChannelOrder = OpenCL.CL_RGB;
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

      isBackedByExternalByteBuffer = backingDirectByteBuffer != null;
      if (isBackedByExternalByteBuffer)
      {
         this.backingDirectByteBuffer = backingDirectByteBuffer;
      }
      else
      {
         this.backingDirectByteBuffer = ByteBuffer.allocateDirect(imageWidth * imageHeight * bytesPerPixel);
         this.backingDirectByteBuffer.order(ByteOrder.nativeOrder());
      }

      bytedecoByteBufferPointer = new BytePointer(this.backingDirectByteBuffer);
      if (suppliedMat != null)
      {
         bytedecoOpenCVMat = suppliedMat;
      }
      else
      {
         bytedecoOpenCVMat = new Mat(imageHeight, imageWidth, cvMatType, bytedecoByteBufferPointer);
      }
   }

   public void createOpenCLImage(OpenCLManager openCLManager, int flags)
   {
      openCLImageObjectFlags = flags;
      openCLImageObject = openCLManager.createImage(flags, openCLChannelOrder, openCLChannelDataType, imageWidth, imageHeight, bytedecoByteBufferPointer);
   }

   public void writeOpenCLImage(OpenCLManager openCLManager)
   {
      openCLManager.enqueueWriteImage(openCLImageObject, imageWidth, imageHeight, bytedecoByteBufferPointer);
   }

   public void readOpenCLImage(OpenCLManager openCLManager)
   {
      openCLManager.enqueueReadImage(openCLImageObject, imageWidth, imageHeight, bytedecoByteBufferPointer);
   }

   public void resize(int imageWidth, int imageHeight, OpenCLManager openCLManager, ByteBuffer externalByteBuffer)
   {
      this.imageWidth = imageWidth;
      this.imageHeight = imageHeight;

      boolean openCLObjectCreated = openCLImageObject != null;
      if (openCLObjectCreated)
      {
         openCLManager.releaseBufferObject(openCLImageObject);
      }

      if (isBackedByExternalByteBuffer)
      {
         backingDirectByteBuffer = externalByteBuffer;
      }
      else
      {
         bytedecoByteBufferPointer.deallocate();
         backingDirectByteBuffer = ByteBuffer.allocateDirect(imageWidth * imageHeight * bytesPerPixel);
         backingDirectByteBuffer.order(ByteOrder.nativeOrder());
      }
      bytedecoByteBufferPointer = new BytePointer(backingDirectByteBuffer);
      bytedecoOpenCVMat = new Mat(imageHeight, imageWidth, cvMatType, bytedecoByteBufferPointer);

      if (openCLObjectCreated)
      {
         createOpenCLImage(openCLManager, openCLImageObjectFlags);
      }
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

   public int getImageWidth()
   {
      return imageWidth;
   }

   public int getImageHeight()
   {
      return imageHeight;
   }
}
