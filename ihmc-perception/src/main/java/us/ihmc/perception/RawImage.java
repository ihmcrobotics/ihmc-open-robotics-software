package us.ihmc.perception;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.ImageMessage;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.sensors.CameraIntrinsics;

import java.time.Instant;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import javax.annotation.Nullable;

/**
 * <p>
 * This class represents a raw image, as taken from a sensor with no compression.
 * It should contain all information needed to create and publish an {@link ImageMessage},
 * once the raw image is compressed.
 * </p>
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
@SuppressWarnings("ConstantConditions")
public class RawImage
{
   /*
    * Although both cpu & gpu image matrices are nullable,
    * at least one should be not null.
    */
   @Nullable
   private Mat cpuImageMat = null;
   private final Lock cpuImageLock = new ReentrantLock();
   @Nullable
   private GpuMat gpuImageMat = null;
   private final Lock gpuImageLock = new ReentrantLock();
   private final PixelFormat pixelFormat;
   private final CameraIntrinsics cameraIntrinsics;
   private final CameraModel cameraModel;
   private final float depthDiscretization;
   private final long sequenceNumber;
   private final Instant acquisitionTime;
   private final RigidBodyTransformReadOnly sensorToWorldTransform;

   private final AtomicInteger numberOfReferences = new AtomicInteger(1);

   public RawImage(@Nullable Mat cpuImageMat,
                   @Nullable GpuMat gpuImageMat,
                   PixelFormat pixelFormat,
                   CameraIntrinsics cameraIntrinsics,
                   CameraModel cameraModel,
                   RigidBodyTransformReadOnly sensorToWorldTransform,
                   Instant acquisitionTime,
                   long sequenceNumber,
                   float depthDiscretization)
   {
      if ((cpuImageMat == null || cpuImageMat.isNull()) && (gpuImageMat == null || gpuImageMat.isNull()))
         throw new IllegalArgumentException("At least one Mat must be non-null");

      this.cpuImageMat = cpuImageMat;
      this.gpuImageMat = gpuImageMat;
      this.pixelFormat = pixelFormat;
      this.cameraIntrinsics = new CameraIntrinsics(cameraIntrinsics);
      this.cameraModel = cameraModel;
      this.sensorToWorldTransform = new RigidBodyTransform(sensorToWorldTransform);
      this.acquisitionTime = acquisitionTime;
      this.sequenceNumber = sequenceNumber;
      this.depthDiscretization = depthDiscretization;
   }

   public RawImage(RawImage other)
   {
      if (other.hasCpuImage())
         this.cpuImageMat = other.cpuImageMat.clone();
      if (other.hasGpuImage())
         this.gpuImageMat = other.gpuImageMat.clone();
      this.pixelFormat = other.pixelFormat;
      this.cameraIntrinsics = other.cameraIntrinsics;
      this.cameraModel = other.cameraModel;
      this.sensorToWorldTransform = other.sensorToWorldTransform;
      this.acquisitionTime = other.acquisitionTime;
      this.sequenceNumber = other.sequenceNumber;
      this.depthDiscretization = other.depthDiscretization;
   }

   public static RawImage createWithBGRImage(Pointer matPointer,
                                             CameraIntrinsics cameraIntrinsics,
                                             RigidBodyTransformReadOnly sensorTransformToWorld,
                                             Instant acquisitionTime,
                                             long sequenceNumber)
   {
      return createWithBGRImage(matPointer, cameraIntrinsics, CameraModel.PINHOLE, sensorTransformToWorld, acquisitionTime, sequenceNumber);
   }

   public static RawImage createWithBGRImage(Pointer matPointer,
                                             CameraIntrinsics cameraIntrinsics,
                                             CameraModel cameraModel,
                                             RigidBodyTransformReadOnly sensorTransformToWorld,
                                             Instant acquisitionTime,
                                             long sequenceNumber)
   {
      if (matPointer instanceof Mat cpuImage)
         return new RawImage(cpuImage, null, PixelFormat.BGR8, cameraIntrinsics, cameraModel, sensorTransformToWorld, acquisitionTime, sequenceNumber, -1.0f);
      else if (matPointer instanceof GpuMat gpuImage)
         return new RawImage(null, gpuImage, PixelFormat.BGR8, cameraIntrinsics, cameraModel, sensorTransformToWorld, acquisitionTime, sequenceNumber, -1.0f);

      throw new IllegalArgumentException("The pointer passed in was neither a Mat nor GpuMat");
   }

   public static RawImage createWith16BitDepth(Pointer matPointer,
                                               CameraIntrinsics cameraIntrinsics,
                                               RigidBodyTransformReadOnly sensorTransformToWorld,
                                               Instant acquisitionTime,
                                               long sequenceNumber,
                                               float depthDiscretization)
   {
      return createWith16BitDepth(matPointer,
                                  cameraIntrinsics,
                                  CameraModel.PINHOLE,
                                  sensorTransformToWorld,
                                  acquisitionTime,
                                  sequenceNumber,
                                  depthDiscretization);
   }

   public static RawImage createWith16BitDepth(Pointer matPointer,
                                               CameraIntrinsics cameraIntrinsics,
                                               CameraModel cameraModel,
                                               RigidBodyTransformReadOnly sensorTransformToWorld,
                                               Instant acquisitionTime,
                                               long sequenceNumber,
                                               float depthDiscretization)
   {
      if (matPointer instanceof Mat cpuImage)
         return new RawImage(cpuImage,
                             null,
                             PixelFormat.GRAY16,
                             cameraIntrinsics,
                             cameraModel,
                             sensorTransformToWorld,
                             acquisitionTime,
                             sequenceNumber,
                             depthDiscretization);
      else if (matPointer instanceof GpuMat gpuImage)
         return new RawImage(null,
                             gpuImage,
                             PixelFormat.GRAY16,
                             cameraIntrinsics,
                             cameraModel,
                             sensorTransformToWorld,
                             acquisitionTime,
                             sequenceNumber,
                             depthDiscretization);

      throw new IllegalArgumentException("The pointer passed in was neither a Mat nor GpuMat");
   }

   /**
    * Provides a new {@link RawImage} with the same pixel format, intrinsics, and metadata as this one, but with a different image.
    * Useful when applying changes to Mats and wishing to keep the same intrinsics & metadata in the {@link RawImage}.
    *
    * @param newCpuImageMat new CPU image mat to replace the current image. Must have the same dimensions.
    * @return A new {@link RawImage} with the same intrinsics & metadata, but with a different image.
    */
   public RawImage replaceImage(Mat newCpuImageMat)
   {
      return replaceImage(newCpuImageMat, this.pixelFormat);
   }

   /**
    * Provides a new {@link RawImage} with the same intrinsics and metadata as this one, but with a different image.
    * Useful when applying changes to Mats and wishing to keep the same intrinsics & metadata in the {@link RawImage}.
    *
    * @param newCpuImageMat new CPU image mat to replace the current image. Must have the same dimensions.
    * @param newPixelFormat the PixelFormat of the new image.
    * @return A new {@link RawImage} with the same intrinsics & metadata, but with a different image.
    */
   public RawImage replaceImage(Mat newCpuImageMat, PixelFormat newPixelFormat)
   {
      if (getWidth() != newCpuImageMat.cols() || getHeight() != newCpuImageMat.rows())
         throw new IllegalArgumentException("New image must have the same dimensions as the current image");

      return new RawImage(newCpuImageMat,
                          null,
                          newPixelFormat,
                          this.cameraIntrinsics,
                          this.cameraModel,
                          this.sensorToWorldTransform,
                          this.acquisitionTime,
                          this.sequenceNumber,
                          this.depthDiscretization);
   }

   /**
    * Provides a new {@link RawImage} with the same pixel format, intrinsics, and metadata as this one, but with a different image.
    * Useful when applying changes to Mats and wishing to keep the same intrinsics & metadata in the {@link RawImage}.
    *
    * @param newGpuImageMat new GPU image mat to replace the current image. Must have the same dimensions.
    * @return A new {@link RawImage} with the same intrinsics & metadata, but with a different image.
    */
   public RawImage replaceImage(GpuMat newGpuImageMat)
   {
      return replaceImage(newGpuImageMat, this.pixelFormat);
   }

   /**
    * Provides a new {@link RawImage} with the same intrinsics and metadata as this one, but with a different image.
    * Useful when applying changes to Mats and wishing to keep the same intrinsics & metadata in the {@link RawImage}.
    *
    * @param newGpuImageMat new GPU image mat to replace the current image. Must have the same dimensions.
    * @param newPixelFormat the PixelFormat of the new image.
    * @return A new {@link RawImage} with the same intrinsics & metadata, but with a different image.
    */
   public RawImage replaceImage(GpuMat newGpuImageMat, PixelFormat newPixelFormat)
   {
      if (getWidth() != newGpuImageMat.cols() || getHeight() != newGpuImageMat.rows())
         throw new IllegalArgumentException("New image must have the same dimensions as the current image");

      return new RawImage(null,
                          newGpuImageMat,
                          newPixelFormat,
                          this.cameraIntrinsics,
                          this.cameraModel,
                          this.sensorToWorldTransform,
                          this.acquisitionTime,
                          this.sequenceNumber,
                          this.depthDiscretization);
   }

   public long getSequenceNumber()
   {
      return sequenceNumber;
   }

   public Instant getAcquisitionTime()
   {
      return acquisitionTime;
   }

   public int getWidth()
   {
      return cameraIntrinsics.getWidth();
   }

   public int getHeight()
   {
      return cameraIntrinsics.getHeight();
   }

   public float getDepthDiscretization()
   {
      return depthDiscretization;
   }

   /**
    * <p>
    * Get the image's {@link Mat}.
    * </p>
    * <p>
    * If this image only has a {@link GpuMat}, a new {@link Mat} will be created
    * and the image data will be downloaded from the device (GPU).
    * </p>
    *
    * @return The {@link Mat} containing the image data.
    */
   public Mat getCpuImageMat()
   {
      cpuImageLock.lock();
      try
      {
         if (cpuImageMat == null)
         {
            cpuImageMat = new Mat(gpuImageMat.size(), gpuImageMat.type());
            gpuImageMat.download(cpuImageMat);
         }
      }
      finally
      {
         cpuImageLock.unlock();
      }

      return cpuImageMat;
   }

   /**
    * <p>
    * Get the image's {@link GpuMat}.
    * </p>
    * <p>
    * If this image only has a {@link Mat}, a new {@link GpuMat} will be created
    * and the image data will be uploaded to the device (GPU).
    * </p>
    *
    * @return The {@link GpuMat} containing the image data.
    */
   public GpuMat getGpuImageMat()
   {
      gpuImageLock.lock();
      try
      {
         if (gpuImageMat == null)
         {
            gpuImageMat = new GpuMat(cpuImageMat.size(), cpuImageMat.type());
            gpuImageMat.upload(cpuImageMat);
         }
      }
      finally
      {
         gpuImageLock.unlock();
      }

      return gpuImageMat;
   }

   /**
    * <p>
    * Get the pointer to the image data.
    * </p>
    * <p>
    * Same as calling {@code getCpuImageMat().data()}. As such, if this image does not have a {@link Mat},
    * a new {@link Mat} will be created and the image data will be downloaded from the device (GPU).
    * </p>
    *
    * @return The pointer to the image data.
    */
   public BytePointer getDataPointer()
   {
      return getCpuImageMat().data();
   }

   /**
    * <p>
    * Get the CUDA pointer to the image data.
    * </p>
    * <p>
    * Same as calling {@code getGpuImageMat().data()}. As such, if this image does not have a {@link GpuMat},
    * a new {@link GpuMat} will be created and the image data will be uploaded to the device (GPU).
    * </p>
    *
    * @return The CUDA pointer to the image data.
    */
   public BytePointer getCUDADataPointer()
   {
      return getGpuImageMat().data();
   }

   public CameraIntrinsics getIntrinsicsCopy()
   {
      return new CameraIntrinsics(cameraIntrinsics);
   }

   public CameraModel getCameraModel()
   {
      return cameraModel;
   }

   public PixelFormat getPixelFormat()
   {
      return pixelFormat;
   }

   public int getOpenCVType()
   {
      if (hasCpuImage())
         return cpuImageMat.type();
      else if (hasGpuImage())
         return gpuImageMat.type();

      throw new NullPointerException("Neither CPU nor GPU Mats were initialized");
   }

   public float getFocalLengthX()
   {
      return (float) cameraIntrinsics.getFx();
   }

   public float getFocalLengthY()
   {
      return (float) cameraIntrinsics.getFy();
   }

   public float getPrincipalPointX()
   {
      return (float) cameraIntrinsics.getCx();
   }

   public float getPrincipalPointY()
   {
      return (float) cameraIntrinsics.getCy();
   }

   public RigidBodyTransformReadOnly getTransformToWorld()
   {
      return sensorToWorldTransform;
   }

   public Tuple3DReadOnly getTranslation()
   {
      return sensorToWorldTransform.getTranslation();
   }

   public Orientation3DReadOnly getRotation()
   {
      return sensorToWorldTransform.getRotation();
   }

   public boolean hasCpuImage()
   {
      return cpuImageMat != null && !cpuImageMat.isNull();
   }

   public boolean hasGpuImage()
   {
      return gpuImageMat != null && !gpuImageMat.isNull();
   }

   public boolean isEmpty()
   {
      return !hasCpuImage() && !hasGpuImage();
   }

   public boolean isAvailable()
   {
      return numberOfReferences.get() > 0;
   }

   public int getReferenceCount()
   {
      return numberOfReferences.get();
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
      if (hasCpuImage())
         cpuImageMat.close();
      if (hasGpuImage())
         gpuImageMat.close();
   }
}
