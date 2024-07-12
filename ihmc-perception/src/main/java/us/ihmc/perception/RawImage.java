package us.ihmc.perception;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;
import us.ihmc.perception.camera.CameraIntrinsics;

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
 * To ensure all Mat and GpuMat objects get deallocated properly, this class uses
 * a reference count. The counter is incremented upon construction and each call to
 * {@link RawImage#get()}, and decremented for each call to {@link RawImage#release()}.
 * Once the reference count hits zero, the image data is deallocated.
 * </p>
 * <p>
 * Each method should call {@link RawImage#get()} before accessing the image data
 * to ensure it is not deallocated during access. After the data has been accessed,
 * {@link RawImage#release()} should be called.
 * </p>
 */
public class RawImage
{
   private final long sequenceNumber;
   private final Instant acquisitionTime;
   private final float depthDiscretization;
   /*
    * Although both cpu & gpu image matrices are nullable,
    * at least one should be not null.
    */
   @Nullable
   private Mat cpuImageMat = null;
   @Nullable
   private GpuMat gpuImageMat = null;
   private final float focalLengthX;
   private final float focalLengthY;
   private final float principalPointX;
   private final float principalPointY;
   private final FixedFramePoint3DBasics position;
   private final FixedFrameQuaternionBasics orientation;

   private final AtomicInteger numberOfReferences = new AtomicInteger(1);

   public RawImage(long sequenceNumber,
                   Instant acquisitionTime,
                   float depthDiscretization,
                   @Nullable Mat cpuImageMat,
                   @Nullable GpuMat gpuImageMat,
                   float focalLengthX,
                   float focalLengthY,
                   float principalPointX,
                   float principalPointY,
                   FixedFramePoint3DBasics position,
                   FixedFrameQuaternionBasics orientation)
   {
      this.sequenceNumber = sequenceNumber;
      this.acquisitionTime = acquisitionTime;
      this.depthDiscretization = depthDiscretization;
      this.cpuImageMat = cpuImageMat;
      this.gpuImageMat = gpuImageMat;
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
      this.depthDiscretization = other.depthDiscretization;
      if (!other.isEmpty())
      {
         if (other.cpuImageMat != null && !other.cpuImageMat.isNull())
            this.cpuImageMat = other.cpuImageMat.clone();
         if (other.gpuImageMat != null && !other.gpuImageMat.isNull())
            this.gpuImageMat = other.gpuImageMat.clone();
      }
      this.focalLengthX = other.focalLengthX;
      this.focalLengthY = other.focalLengthY;
      this.principalPointX = other.principalPointX;
      this.principalPointY = other.principalPointY;
      this.position = other.position;
      this.orientation = other.orientation;
   }

   public static RawImage fromMessage(ImageMessage imageMessage)
   {
      try (BytePointer compressedImageData = new BytePointer(imageMessage.getData().size());
           Mat compressedImageMat = new Mat(1, imageMessage.getData().size(), opencv_core.CV_8UC1))
      {
         compressedImageData.put(imageMessage.getData().toArray());
         compressedImageMat.data(compressedImageData);
         Mat imageMat = new Mat();
         opencv_imgcodecs.imdecode(compressedImageMat, opencv_imgcodecs.IMREAD_UNCHANGED, imageMat);

         return new RawImage(imageMessage.getSequenceNumber(),
                             MessageTools.toInstant(imageMessage.getAcquisitionTime()),
                             imageMessage.getDepthDiscretization(),
                             imageMat,
                             null,
                             imageMessage.getFocalLengthXPixels(),
                             imageMessage.getFocalLengthYPixels(),
                             imageMessage.getPrincipalPointXPixels(),
                             imageMessage.getPrincipalPointYPixels(),
                             new FramePoint3D(ReferenceFrame.getWorldFrame(), imageMessage.getPosition()),
                             new FrameQuaternion(ReferenceFrame.getWorldFrame(), imageMessage.getOrientation()));
      }
   }

   /**
    * Provides a new {@link RawImage} with the same intrinsics and metadata as this one, but with a different image.
    * Useful when applying changes to Mats and wishing to keep the same intrinsics & metadata in the {@link RawImage}.
    * @param newCpuImageMat new CPU image mat to replace the current image. Must have the same dimensions & type.
    * @return A new {@link RawImage} with the same intrinsics & metadata, but with a different image.
    */
   public RawImage replaceImage(Mat newCpuImageMat)
   {
      if (getImageWidth() != newCpuImageMat.cols() || getImageHeight() != newCpuImageMat.rows())
         throw new IllegalArgumentException("New image must have the same dimensions as the current image");

      RawImage newRawImage = new RawImage(this);
      newCpuImageMat.copyTo(newRawImage.getCpuImageMat());
      newRawImage.getGpuImageMat().upload(newCpuImageMat);
      return newRawImage;
   }

   /**
    * Provides a new {@link RawImage} with the same intrinsics and metadata as this one, but with a different image.
    * Useful when applying changes to Mats and wishing to keep the same intrinsics & metadata in the {@link RawImage}.
    * @param newGpuImageMat new GPU image mat to replace the current image. Must have the same dimensions & type.
    * @return A new {@link RawImage} with the same intrinsics & metadata, but with a different image.
    */
   public RawImage replaceImage(GpuMat newGpuImageMat)
   {
      if (getImageWidth() != newGpuImageMat.cols() || getImageHeight() != newGpuImageMat.rows())
         throw new IllegalArgumentException("New image must have the same dimensions as the current image");
      if (getOpenCVType() != newGpuImageMat.type())
         throw new IllegalArgumentException("New image must be the same OpenCV type as the current image");

      RawImage newRawImage = new RawImage(this);
      newGpuImageMat.copyTo(newRawImage.getGpuImageMat());
      newGpuImageMat.download(newRawImage.getCpuImageMat());
      return newRawImage;
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
      if (cpuImageMat != null && !cpuImageMat.isNull())
         return cpuImageMat.cols();
      else if (gpuImageMat != null && !gpuImageMat.isNull())
         return gpuImageMat.cols();

      throw new NullPointerException("Neither CPU nor GPU Mats were initialized");
   }

   public int getImageHeight()
   {
      if (cpuImageMat != null && !cpuImageMat.isNull())
         return cpuImageMat.rows();
      else if (gpuImageMat != null && !gpuImageMat.isNull())
         return gpuImageMat.rows();

      throw new NullPointerException("Neither CPU nor GPU Mats were initialized");
   }

   public float getDepthDiscretization()
   {
      return depthDiscretization;
   }

   public Mat getCpuImageMat()
   {
      if (cpuImageMat == null && gpuImageMat == null)
      {
         throw new NullPointerException("Neither CPU nor GPU Mats were initialized");
      }
      else if (cpuImageMat == null && !gpuImageMat.isNull())
      {
         cpuImageMat = new Mat(gpuImageMat.size(), gpuImageMat.type());
         gpuImageMat.download(cpuImageMat);
      }

      if (cpuImageMat == null)
      {
         throw new NullPointerException("Failed to initialize CPU image");
      }
      if (cpuImageMat.isNull())
      {
         throw new NullPointerException("Failed to download GPU image to CPU");
      }

      return cpuImageMat;
   }

   public GpuMat getGpuImageMat()
   {
      if (gpuImageMat == null && cpuImageMat == null)
      {
         throw new NullPointerException("Neither CPU nor GPU Mats were initialized");
      }
      else if (gpuImageMat == null && !cpuImageMat.isNull())
      {
         gpuImageMat = new GpuMat(cpuImageMat.size(), cpuImageMat.type());
         gpuImageMat.upload(cpuImageMat);
      }

      if (gpuImageMat == null)
      {
         throw new NullPointerException("Failed to initialize GPU image");
      }
      if (gpuImageMat.isNull())
      {
         throw new NullPointerException("Failed to upload CPU image to GPU");
      }

      return gpuImageMat;
   }

   public CameraIntrinsics getIntrinsicsCopy()
   {
      return new CameraIntrinsics(getImageHeight(), getImageWidth(), focalLengthX, focalLengthY, principalPointX, principalPointY);
   }

   public int getOpenCVType()
   {
      if (cpuImageMat != null && !cpuImageMat.isNull())
         return cpuImageMat.type();
      else if (gpuImageMat != null && !gpuImageMat.isNull())
         return gpuImageMat.type();

      throw new NullPointerException("Neither CPU nor GPU Mats were initialized");
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
      return cpuImageMat == null && gpuImageMat == null;
   }

   public boolean isAvailable()
   {
      return numberOfReferences.get() > 0;
   }

   public RawImage get()
   {
      if (numberOfReferences.incrementAndGet() > 1)
         return this;
      else
         return null;
   }

   public void release()
   {
      if (numberOfReferences.decrementAndGet() <= 0)
         destroy();
   }

   private void destroy()
   {
      if (cpuImageMat != null)
         cpuImageMat.close();
      if (gpuImageMat != null)
         gpuImageMat.close();
   }
}
