package us.ihmc.perception.detections.yolo;

import org.bytedeco.opencv.opencv_core.GpuMat;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.detections.InstantDetection;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.sensors.ImageSensor;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

public class YOLOv8DetectionThread extends RepeatingTaskThread
{
   private final YOLOv8DetectionExecutor yoloExecutor;

   private ImageSensor imageSensor;
   private int colorImageKey;
   private int depthImageKey;

   public YOLOv8DetectionThread(ROS2PeerClockOffsetEstimator clockOffsetEstimator, BooleanSupplier annotatedImageDemandSupplier)
   {
      super(YOLOv8DetectionThread.class.getSimpleName());

      yoloExecutor = new YOLOv8DetectionExecutor(clockOffsetEstimator, annotatedImageDemandSupplier);
   }

   public void addDetectionConsumerCallback(Consumer<List<InstantDetection>> consumer)
   {
      yoloExecutor.addDetectionConsumerCallback(consumer);
   }

   // synchronized along with runInLoop() to not change sensors in middle of execution
   public synchronized void setImageSensor(ImageSensor imageSensor, int colorImageKey, int depthImageKey)
   {
      this.imageSensor = imageSensor;
      this.colorImageKey = colorImageKey;
      this.depthImageKey = depthImageKey;
   }

   @Override
   protected synchronized void runTask()
   {
      try
      {
         imageSensor.waitForGrab();

         RawImage colorImage = imageSensor.getImage(colorImageKey);
         RawImage depthImage = imageSensor.getImage(depthImageKey);

         // Ensure color image is in BGR8
         if (colorImage.getPixelFormat() != PixelFormat.BGR8)
         {
            GpuMat bgrMat = new GpuMat();
            colorImage.getPixelFormat().convertToPixelFormat(colorImage.getGpuImageMat(), bgrMat, PixelFormat.BGR8);
            colorImage.release();
            colorImage = colorImage.replaceImage(bgrMat, PixelFormat.BGR8);
         }

         yoloExecutor.runNextEnabledModel(colorImage, depthImage);

         colorImage.release();
         depthImage.release();
      } catch (InterruptedException ignored) {}
   }

   @Override
   public void kill()
   {
      super.kill();
      interrupt();
      yoloExecutor.destroy();
   }

   public YOLOv8DetectionExecutor getYoloExecutor()
   {
      return yoloExecutor;
   }
}
