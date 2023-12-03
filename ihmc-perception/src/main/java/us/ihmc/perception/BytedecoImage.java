package us.ihmc.perception;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencl._cl_mem;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.perception.tools.NativeMemoryTools;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class BytedecoImage
{
   private ByteBuffer backingDirectByteBuffer;
   private MutableBytePointer bytedecoByteBufferPointer;
   private Mat bytedecoOpenCVMat;
   private BytePointer pointerForAccessSpeed = null;
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
      this(OpenCVTools.getImageWidth(suppliedMat),
           OpenCVTools.getImageHeight(suppliedMat),
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
      else if (cvMatType == opencv_core.CV_32FC3) // Not sure if this one works
      {
         bytesPerPixel = 4 * 3;
         openCLChannelOrder = OpenCL.CL_RGB;
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
         this.backingDirectByteBuffer = NativeMemoryTools.allocate(imageWidth * imageHeight * bytesPerPixel);
      }

      bytedecoByteBufferPointer = new MutableBytePointer(this.backingDirectByteBuffer);
      if (suppliedMat != null)
      {
         bytedecoOpenCVMat = suppliedMat;
      }
      else
      {
         bytedecoOpenCVMat = new Mat(imageHeight, imageWidth, cvMatType, bytedecoByteBufferPointer);
      }
      pointerForAccessSpeed = bytedecoOpenCVMat.ptr(0);
   }

   public void destroy(OpenCLManager openCLManager)
   {
      if (openCLImageObject != null)
      {
         openCLManager.releaseBufferObject(openCLImageObject);
         openCLImageObject.releaseReference();
      }
   }

   public void changeAddress(long address)
   {
      bytedecoByteBufferPointer.deallocate();
      bytedecoByteBufferPointer.setAddress(address);
      backingDirectByteBuffer = bytedecoByteBufferPointer.asByteBuffer(); // Allocates, but on the native side?
      bytedecoOpenCVMat.data(bytedecoByteBufferPointer);
      pointerForAccessSpeed = bytedecoOpenCVMat.ptr(0);
   }

   public void createOpenCLImage(OpenCLManager openCLManager, int flags)
   {
      if (openCLChannelOrder == OpenCL.CL_RGB)
      {
         throw new RuntimeException("OpenCL will throw CL_OUT_OF_RESOURCES unless you use RGBA and CV_8UC4."
                                    + "It's probably something about memory alignment on hardware. Just include the alpha channel.");
      }
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

   public void fill(OpenCLManager openCLManager, byte value)
   {
      bytedecoByteBufferPointer.fill(value);
      openCLManager.enqueueFillBuffer(openCLImageObject, 1, value);
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
         if (backingDirectByteBuffer.capacity() < imageWidth * imageHeight * bytesPerPixel)
         {
            throw new RuntimeException("Externally managed byte buffer large enough."
                                       + "Resize it before calling this resize method.");
         }
      }
      else
      {
         bytedecoByteBufferPointer.deallocate();
         backingDirectByteBuffer = ByteBuffer.allocateDirect(imageWidth * imageHeight * bytesPerPixel);
         backingDirectByteBuffer.order(ByteOrder.nativeOrder());
      }
      bytedecoByteBufferPointer = new MutableBytePointer(backingDirectByteBuffer);
      bytedecoOpenCVMat = new Mat(imageHeight, imageWidth, cvMatType, bytedecoByteBufferPointer);
      pointerForAccessSpeed = bytedecoOpenCVMat.ptr(0);

      if (openCLObjectCreated)
      {
         createOpenCLImage(openCLManager, openCLImageObjectFlags);
      }
   }

   /**
    * Resizes this image to match the dimensions of other if necessary.
    *
    * Warning: Assumes we are not using OpenCL on this image and this BytedecoImage is not
    *   backed by an external buffer.
    */
   public void ensureDimensionsMatch(BytedecoImage other)
   {
      ensureDimensionsMatch(other, null);
   }

   /**
    * Resizes this image to match the dimensions of other if necessary.
    *
    * // FIXME: Broken for external byte buffers
    */
   public void ensureDimensionsMatch(BytedecoImage other, OpenCLManager openCLManager)
   {
      if (!OpenCVTools.dimensionsMatch(this, other))
      {
         resize(other.getImageWidth(), other.getImageHeight(), openCLManager, backingDirectByteBuffer);
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

   /**
    * Retrieve a float from the image.
    *
    * This uses a precalulated pointer to allow for faster access.
    */
   public float getFloat(int row, int column)
   {
      return pointerForAccessSpeed.getFloat(getLinearizedIndex(row, column) * Float.BYTES);
   }

   /**
    * Retrieve an int from the image.
    *
    * This uses a precalulated pointer to allow for faster access.
    */
   public int getInt(int row, int column)
   {
      return pointerForAccessSpeed.getInt((getLinearizedIndex(row, column)) * Float.BYTES);
   }

   /**
    * Set a float in the image.
    *
    * This uses a precalulated pointer to allow for faster access.
    */
   public void setValue(int row, int column, float value)
   {
      pointerForAccessSpeed.putFloat(getLinearizedIndex(row, column) * Float.BYTES, value);
   }

   /**
    * Retrieve a byte from the image. The value of the byte is given as a positive value
    * in and int.
    *
    * This uses a precalulated pointer to allow for faster access.
    */
   public int getByteAsInteger(int row, int column)
   {
      return Byte.toUnsignedInt(pointerForAccessSpeed.get(getLinearizedIndex(row, column)));
   }

   /**
    * Retrieve a byte from the image. The value of the byte is given as a positive value
    * in and int.
    */
   public int getByteAsInteger(int byteIndex)
   {
      return Byte.toUnsignedInt(backingDirectByteBuffer.get(byteIndex));
   }

   /**
    * Calculate the index for the data entry located at (column, row). This handles whether the image is row major or column major.
    */
   private long getLinearizedIndex(int row, int column)
   {
      return (long) row * imageWidth + column;
   }

   public BytePointer getPointerForAccessSpeed()
   {
      return pointerForAccessSpeed;
   }
}
