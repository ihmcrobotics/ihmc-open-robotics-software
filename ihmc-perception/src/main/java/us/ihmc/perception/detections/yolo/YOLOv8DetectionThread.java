package us.ihmc.perception.detections.yolo;

import org.bytedeco.opencv.opencv_core.GpuMat;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.detections.DetectionManager;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.sensors.ImageSensor;

import java.util.function.BooleanSupplier;

public class YOLOv8DetectionThread extends RepeatingTaskThread
{
   private final YOLOv8DetectionExecutor yoloExecutor;

   private ImageSensor imageSensor;
   private int colorImageKey;
   private int depthImageKey;

   public YOLOv8DetectionThread(ROS2Helper ros2Helper, DetectionManager detectionManager, BooleanSupplier annotatedImageDemandSupplier)
   {
      super(YOLOv8DetectionThread.class.getSimpleName());

      yoloExecutor = new YOLOv8DetectionExecutor(ros2Helper, annotatedImageDemandSupplier);
      yoloExecutor.addDetectionConsumerCallback(detectionManager::addDetections);
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

         yoloExecutor.runYOLODetectionOnAllModels(colorImage, depthImage);

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
}
