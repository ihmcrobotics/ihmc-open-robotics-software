package us.ihmc.perception;

import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.opencl._cl_mem;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.perception.opencl.OpenCLManager;

import java.nio.FloatBuffer;

public class BytedecoFloatImage
{
   private final FloatBuffer backingDirectFloatBuffer;
   private final FloatPointer bytedecoFloatBufferPointer;
   private final Mat bytedecoOpenCVMat;
   private final int openCLChannelOrder;
   private final int openCLChannelDataType;
   private final int imageWidth;
   private final int imageHeight;
   private _cl_mem openCLImageObject;

   public BytedecoFloatImage(int imageWidth, int imageHeight, int cvMatType, FloatBuffer backingDirectFloatBuffer)
   {
      this.imageWidth = imageWidth;
      this.imageHeight = imageHeight;
      this.backingDirectFloatBuffer = backingDirectFloatBuffer;

      openCLChannelOrder = OpenCL.CL_R;
      openCLChannelDataType = OpenCL.CL_FLOAT;

      bytedecoFloatBufferPointer = new FloatPointer(backingDirectFloatBuffer);
      bytedecoOpenCVMat = new Mat(imageHeight, imageWidth, cvMatType, bytedecoFloatBufferPointer);
   }

   public void createOpenCLImage(OpenCLManager openCLManager, int flags)
   {
      openCLImageObject = openCLManager.createImage(flags, openCLChannelOrder, openCLChannelDataType, imageWidth, imageHeight, bytedecoFloatBufferPointer);
   }

   public void writeOpenCLImage(OpenCLManager openCLManager)
   {
      openCLManager.enqueueWriteImage(openCLImageObject, imageWidth, imageHeight, bytedecoFloatBufferPointer);
   }

   public void readOpenCLImage(OpenCLManager openCLManager)
   {
      openCLManager.enqueueReadImage(openCLImageObject, imageWidth, imageHeight, bytedecoFloatBufferPointer);
   }

   public void rewind()
   {
      backingDirectFloatBuffer.rewind();
      bytedecoFloatBufferPointer.position(0);
   }

   public Mat getBytedecoOpenCVMat()
   {
      return bytedecoOpenCVMat;
   }

   public FloatBuffer getBackingDirectFloatBuffer()
   {
      return backingDirectFloatBuffer;
   }

   public FloatPointer getBytedecoFloatBufferPointer()
   {
      return bytedecoFloatBufferPointer;
   }

   public _cl_mem getOpenCLImageObject()
   {
      return openCLImageObject;
   }
}
