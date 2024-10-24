package us.ihmc.perception.detections.yolo;

import org.bytedeco.opencv.opencv_core.GpuMat;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.detections.DetectionManager;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.sensors.ImageSensor;
import us.ihmc.tools.thread.RestartableThread;

import java.util.function.BooleanSupplier;

public class YOLOv8DetectionRunner implements AutoCloseable
{
   private final YOLOv8DetectionExecutor yoloExecutor;
   private final RestartableThread yoloThread;

   private ImageSensor imageSensor;
   private int colorImageKey;
   private int depthImageKey;

   public YOLOv8DetectionRunner(ROS2Helper ros2Helper, DetectionManager detectionManager, BooleanSupplier annotatedImageDemandSupplier)
   {
      yoloExecutor = new YOLOv8DetectionExecutor(ros2Helper, annotatedImageDemandSupplier);
      yoloExecutor.addDetectionConsumerCallback(detectionManager::addDetections);
      yoloThread = new RestartableThread(getClass().getSimpleName() + "Thread", this::run);
   }

   public synchronized void run(boolean run)
   {
      if (run)
         yoloThread.start();
      else
         yoloThread.stop();
   }

   public synchronized void setImageSensor(ImageSensor imageSensor, int colorImageKey, int depthImageKey)
   {
      this.imageSensor = imageSensor;
      this.colorImageKey = colorImageKey;
      this.depthImageKey = depthImageKey;
   }

   public synchronized void run() throws InterruptedException
   {
      imageSensor.waitForGrab();

      RawImage colorImage = imageSensor.getImage(colorImageKey);
      RawImage depthImage = imageSensor.getImage(depthImageKey);

      // Ensure color image is in RGB8
      if (colorImage.getPixelFormat() != PixelFormat.RGB8)
      {
         GpuMat rgbMat = new GpuMat();
         colorImage.getPixelFormat().convertToPixelFormat(colorImage.getGpuImageMat(), rgbMat, PixelFormat.RGB8);
         colorImage.release();
         colorImage = colorImage.replaceImage(rgbMat);
      }

      yoloExecutor.runYOLODetectionOnAllModels(colorImage, depthImage);

      colorImage.release();
      depthImage.release();
   }

   @Override
   public void close()
   {
      yoloThread.blockingStop();
      yoloExecutor.destroy();
   }
}
