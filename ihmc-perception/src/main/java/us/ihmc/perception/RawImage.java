package us.ihmc.perception;

import com.esotericsoftware.kryo.util.Null;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;

import javax.annotation.Nullable;
import java.time.Instant;

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
    * */
   @Nullable
   private Mat cpuImageMatrix;
   @Nullable
   private GpuMat gpuImageMatrix;
   private final int openCVType;
   private final float focalLengthX;
   private final float focalLengthY;
   private final float principalPointX;
   private final float principalPointY;

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
                   float principalPointY)
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
      if (cpuImageMatrix == null && gpuImageMatrix != null)
      {
         cpuImageMatrix = new Mat(imageHeight, imageWidth, openCVType);
         gpuImageMatrix.download(cpuImageMatrix);
      }
      else
      {
         throw new NullPointerException("Neither CPU nor GPU matrices were initialized");
      }

      return cpuImageMatrix;
   }

   public GpuMat getGpuImageMatrix()
   {
      if (gpuImageMatrix == null && cpuImageMatrix != null)
      {
         gpuImageMatrix = new GpuMat(imageHeight, imageWidth, openCVType);
         gpuImageMatrix.upload(cpuImageMatrix);
      }
      else
      {
         throw new NullPointerException("Neither CPU nor GPU matrices were initialized");
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

   public void destroy()
   {
      if (cpuImageMatrix != null)
         cpuImageMatrix.close();
      if (gpuImageMatrix != null)
         gpuImageMatrix.close();
   }
}
