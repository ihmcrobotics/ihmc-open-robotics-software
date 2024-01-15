package us.ihmc.perception;

import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;

import javax.annotation.Nullable;
import java.time.Instant;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * This class represents a raw image, as taken from a sensor with no compression.
 * It should contain all information needed to create and publish an ImageMessage,
 * once the raw image is compressed.
 * <p>
 * When initialized, this class only needs either a Mat or a GpuMat. For convenience,
 * when the missing version of the image matrix is requested this class will create
 * the missing Mat or GpuMat objects by copying the existing image matrix to the CPU or GPU.
 * </p>
 * <p>
 *  To ensure all Mat and GpuMat objects get deallocated properly, this class uses
 *  a reference count. Whenever passing the image to another class, the
 *  {@code RawImage::get()} method should be used instead of passing it directly.
 *  Each class is responsible for releasing the image once it is done using the image.
 * </p>
 */
public class RawImage
{
   private final long sequenceNumber;
   private final Instant acquisitionTime;
   private final int imageWidth;
   private final int imageHeight;
   private final float depthDiscretization;
   /*
    * Although both cpu & gpu image matrices are nullable,
    * at least one should be not null.
    */
   @Nullable
   private Mat cpuImageMatrix;
   @Nullable
   private GpuMat gpuImageMatrix;
   private final int openCVType;
   private final float focalLengthX;
   private final float focalLengthY;
   private final float principalPointX;
   private final float principalPointY;
   private final FixedFramePoint3DBasics position;
   private final FixedFrameQuaternionBasics orientation;

   private final AtomicInteger numberOfReferences = new AtomicInteger(1);

   public RawImage(long sequenceNumber,
                   Instant acquisitionTime,
                   int imageWidth,
                   int imageHeight,
                   float depthDiscretization,
                   @Nullable Mat cpuImageMatrix,
                   @Nullable GpuMat gpuImageMatrix,
                   int openCVType,
                   float focalLengthX,
                   float focalLengthY,
                   float principalPointX,
                   float principalPointY,
                   FixedFramePoint3DBasics position,
                   FixedFrameQuaternionBasics orientation)
   {
      this.sequenceNumber = sequenceNumber;
      this.acquisitionTime = acquisitionTime;
      this.imageWidth = imageWidth;
      this.imageHeight = imageHeight;
      this.depthDiscretization = depthDiscretization;
      this.cpuImageMatrix = cpuImageMatrix;
      this.gpuImageMatrix = gpuImageMatrix;
      this.openCVType = openCVType;
      this.focalLengthX = focalLengthX;
      this.focalLengthY = focalLengthY;
      this.principalPointX = principalPointX;
      this.principalPointY = principalPointY;
      this.position = position;
      this.orientation = orientation;
   }

   public RawImage(RawImage other)
   {
      this.sequenceNumber = other.sequenceNumber;
      this.acquisitionTime = other.acquisitionTime;
      this.imageWidth = other.imageWidth;
      this.imageHeight = other.imageHeight;
      this.depthDiscretization = other.depthDiscretization;
      if (!other.isEmpty())
      {
         if (other.cpuImageMatrix != null && !other.cpuImageMatrix.isNull())
            this.cpuImageMatrix = other.cpuImageMatrix.clone();
         if (other.gpuImageMatrix != null && !other.gpuImageMatrix.isNull())
            this.gpuImageMatrix = other.gpuImageMatrix.clone();
      }
      this.openCVType = other.openCVType;
      this.focalLengthX = other.focalLengthX;
      this.focalLengthY = other.focalLengthY;
      this.principalPointX = other.principalPointX;
      this.principalPointY = other.principalPointY;
      this.position = other.position;
      this.orientation = other.orientation;
   }

   public long getSequenceNumber()
   {
      return sequenceNumber;
   }

   public Instant getAcquisitionTime()
   {
      return acquisitionTime;
   }

   public int getImageWidth()
   {
      return imageWidth;
   }

   public int getImageHeight()
   {
      return imageHeight;
   }

   public float getDepthDiscretization()
   {
      return depthDiscretization;
   }

   public Mat getCpuImageMatrix()
   {
      if (cpuImageMatrix == null && gpuImageMatrix == null)
      {
         throw new NullPointerException("Neither CPU nor GPU matrices were initialized");
      }
      else if (cpuImageMatrix == null && gpuImageMatrix != null && !gpuImageMatrix.isNull())
      {
         cpuImageMatrix = new Mat(imageHeight, imageWidth, openCVType);
         gpuImageMatrix.download(cpuImageMatrix);
      }

      return cpuImageMatrix;
   }

   public GpuMat getGpuImageMatrix()
   {
      if (gpuImageMatrix == null && cpuImageMatrix == null)
      {
         throw new NullPointerException("Neither CPU nor GPU matrices were initialized");
      }
      else if (gpuImageMatrix == null && !cpuImageMatrix.isNull())
      {
         gpuImageMatrix = new GpuMat(imageHeight, imageWidth, openCVType);
         gpuImageMatrix.upload(cpuImageMatrix);
      }

      if (gpuImageMatrix == null || gpuImageMatrix.isNull())
      {
         throw new NullPointerException("Failed to initialize GPU image");
      }
      if (gpuImageMatrix.isNull())
      {
         throw new NullPointerException("Failed to upload CPU image to GPU");
      }

      return gpuImageMatrix;
   }

   public int getOpenCVType()
   {
      return openCVType;
   }

   public float getFocalLengthX()
   {
      return focalLengthX;
   }

   public float getFocalLengthY()
   {
      return focalLengthY;
   }

   public float getPrincipalPointX()
   {
      return principalPointX;
   }

   public float getPrincipalPointY()
   {
      return principalPointY;
   }

   public FixedFramePoint3DBasics getPosition()
   {
      return position;
   }

   public FixedFrameQuaternionBasics getOrientation()
   {
      return orientation;
   }

   public boolean isEmpty()
   {
      return cpuImageMatrix == null && gpuImageMatrix == null;
   }

   public RawImage get()
   {
      numberOfReferences.incrementAndGet();
      return this;
   }

   public void release()
   {
      if (numberOfReferences.decrementAndGet() <= 0)
         destroy();
   }

   private void destroy()
   {
      if (cpuImageMatrix != null)
         cpuImageMatrix.close();
      if (gpuImageMatrix != null)
         gpuImageMatrix.close();
   }
}
