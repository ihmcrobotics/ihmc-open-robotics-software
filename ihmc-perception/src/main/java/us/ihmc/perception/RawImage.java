package us.ihmc.perception;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.imageMessage.CompressionType;
import us.ihmc.perception.imageMessage.PixelFormat;

import javax.annotation.Nullable;
import java.nio.ByteBuffer;
import java.time.Instant;
import java.util.concurrent.atomic.AtomicInteger;

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
   @Nullable
   private GpuMat gpuImageMat = null;
   private final PixelFormat pixelFormat;
   private final CameraIntrinsics cameraIntrinsics;
   private final float depthDiscretization;
   private final long sequenceNumber;
   private final Instant acquisitionTime;
   private final FixedFramePose3DBasics sensorPose;

   private final AtomicInteger numberOfReferences = new AtomicInteger(1);

   public RawImage(@Nullable Mat cpuImageMat,
                   @Nullable GpuMat gpuImageMat,
                   PixelFormat pixelFormat,
                   CameraIntrinsics cameraIntrinsics,
                   FixedFramePose3DBasics sensorPose,
                   Instant acquisitionTime,
                   long sequenceNumber,
                   float depthDiscretization)
   {
      if ((cpuImageMat == null || cpuImageMat.isNull()) && (gpuImageMat == null || gpuImageMat.isNull()))
         throw new IllegalArgumentException("At least one Mat must be non-null");

      this.cpuImageMat = cpuImageMat;
      this.gpuImageMat = gpuImageMat;
      this.pixelFormat = pixelFormat;
      this.cameraIntrinsics = cameraIntrinsics;
      this.sensorPose = sensorPose;
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
      this.sensorPose = other.sensorPose;
      this.acquisitionTime = other.acquisitionTime;
      this.sequenceNumber = other.sequenceNumber;
      this.depthDiscretization = other.depthDiscretization;
   }

   public static RawImage createWithBGRImage(Pointer matPointer,
                                             CameraIntrinsics cameraIntrinsics,
                                             FixedFramePose3DBasics sensorPose,
                                             Instant acquisitionTime,
                                             long sequenceNumber)
   {
      if (matPointer instanceof Mat cpuImage)
         return new RawImage(cpuImage, null, PixelFormat.BGR8, cameraIntrinsics, sensorPose, acquisitionTime, sequenceNumber, -1.0f);
      else if (matPointer instanceof GpuMat gpuImage)
         return new RawImage(null, gpuImage, PixelFormat.BGR8, cameraIntrinsics, sensorPose, acquisitionTime, sequenceNumber, -1.0f);

      throw new IllegalArgumentException("The pointer passed in was neither a Mat nor GpuMat");
   }

   public static RawImage createWith16BitDepth(Pointer matPointer,
                                               CameraIntrinsics cameraIntrinsics,
                                               FixedFramePose3DBasics sensorPose,
                                               Instant acquisitionTime,
                                               long sequenceNumber,
                                               float depthDiscretization)
   {
      if (matPointer instanceof Mat cpuImage)
         return new RawImage(cpuImage, null, PixelFormat.GRAY16, cameraIntrinsics, sensorPose, acquisitionTime, sequenceNumber, depthDiscretization);
      else if (matPointer instanceof GpuMat gpuImage)
         return new RawImage(null, gpuImage, PixelFormat.GRAY16, cameraIntrinsics, sensorPose, acquisitionTime, sequenceNumber, depthDiscretization);

      throw new IllegalArgumentException("The pointer passed in was neither a Mat nor GpuMat");
   }

   /**
    * Provides a new {@link RawImage} with the same intrinsics and metadata as this one, but with a different image.
    * Useful when applying changes to Mats and wishing to keep the same intrinsics & metadata in the {@link RawImage}.
    * @param newCpuImageMat new CPU image mat to replace the current image. Must have the same dimensions.
    * @return A new {@link RawImage} with the same intrinsics & metadata, but with a different image.
    */
   public RawImage replaceImage(Mat newCpuImageMat)
   {
      if (getWidth() != newCpuImageMat.cols() || getHeight() != newCpuImageMat.rows())
         throw new IllegalArgumentException("New image must have the same dimensions as the current image");

      return new RawImage(newCpuImageMat,
                          null,
                          this.pixelFormat,
                          this.cameraIntrinsics,
                          this.sensorPose,
                          this.acquisitionTime,
                          this.sequenceNumber,
                          this.depthDiscretization);
   }

   /**
    * Provides a new {@link RawImage} with the same intrinsics and metadata as this one, but with a different image.
    * Useful when applying changes to Mats and wishing to keep the same intrinsics & metadata in the {@link RawImage}.
    * @param newGpuImageMat new GPU image mat to replace the current image. Must have the same dimensions.
    * @return A new {@link RawImage} with the same intrinsics & metadata, but with a different image.
    */
   public RawImage replaceImage(GpuMat newGpuImageMat)
   {
      if (getWidth() != newGpuImageMat.cols() || getHeight() != newGpuImageMat.rows())
         throw new IllegalArgumentException("New image must have the same dimensions as the current image");

      return new RawImage(null,
                          newGpuImageMat,
                          this.pixelFormat,
                          this.cameraIntrinsics,
                          this.sensorPose,
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
      if (hasCpuImage())
         return cpuImageMat.cols();
      else if (hasGpuImage())
         return gpuImageMat.cols();

      throw new NullPointerException("Neither CPU nor GPU Mats were initialized");
   }

   public int getHeight()
   {
      if (hasCpuImage())
         return cpuImageMat.rows();
      else if (hasGpuImage())
         return gpuImageMat.rows();

      throw new NullPointerException("Neither CPU nor GPU Mats were initialized");
   }

   public float getDepthDiscretization()
   {
      return depthDiscretization;
   }

   public Mat getCpuImageMat()
   {
      if (cpuImageMat == null && !gpuImageMat.isNull())
      {
         cpuImageMat = new Mat(gpuImageMat.size(), gpuImageMat.type());
         gpuImageMat.download(cpuImageMat);
      }

      return cpuImageMat;
   }

   public GpuMat getGpuImageMat()
   {
      if (gpuImageMat == null)
      {
         gpuImageMat = new GpuMat(cpuImageMat.size(), cpuImageMat.type());
         gpuImageMat.upload(cpuImageMat);
      }

      return gpuImageMat;
   }

   public CameraIntrinsics getIntrinsicsCopy()
   {
      return new CameraIntrinsics(cameraIntrinsics);
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

   public FixedFramePose3DBasics getPose()
   {
      return sensorPose;
   }

   public FixedFramePoint3DBasics getPosition()
   {
      return sensorPose.getPosition();
   }

   public FixedFrameQuaternionBasics getOrientation()
   {
      return sensorPose.getOrientation();
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

   /**
    * Packs the {@link ImageMessage} with the {@link RawImage} metadata,
    * EXCEPT:
    * <ul>
    * <li> the CameraModel, </li>
    * <li> the CompressionType, </li>
    * <li> the ouster beam altitude angles, </li>
    * <li> the ouster beam azimuth angles, and </li>
    * <li> the compressed data </li>
    * </ul>
    * To pack everything, use this instead:
    * {@link us.ihmc.perception.tools.PerceptionMessageTools#packImageMessage(RawImage, BytePointer, CompressionType, CameraModel, ImageMessage)}
    * @param messageToPack The message to pack
    */
   public void packImageMessageMetaData(ImageMessage messageToPack)
   {
      messageToPack.setPixelFormat(getPixelFormat().toByte());
      messageToPack.setImageWidth(getWidth());
      messageToPack.setImageHeight(getHeight());
      messageToPack.setFocalLengthXPixels(getFocalLengthX());
      messageToPack.setFocalLengthYPixels(getFocalLengthY());
      messageToPack.setPrincipalPointXPixels(getPrincipalPointX());
      messageToPack.setPrincipalPointYPixels(getPrincipalPointY());
      messageToPack.setDepthDiscretization(getDepthDiscretization());
      messageToPack.setSequenceNumber(getSequenceNumber());
      MessageTools.toMessage(getAcquisitionTime(), messageToPack.getAcquisitionTime());
      messageToPack.getPosition().set(getPosition());
      messageToPack.getOrientation().set(getOrientation());
   }

   private void destroy()
   {
      if (hasCpuImage())
         cpuImageMat.close();
      if (hasGpuImage())
         gpuImageMat.close();
   }
}
