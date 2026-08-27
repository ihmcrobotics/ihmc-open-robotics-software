package us.ihmc.perception.detections.yolo;

import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.log.LogTools;
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

   public YOLOv8DetectionThread(ROS2Node ros2Node, ROS2PeerClockOffsetEstimator clockOffsetEstimator, BooleanSupplier annotatedImageDemandSupplier)
   {
      super(YOLOv8DetectionThread.class.getSimpleName());

      yoloExecutor = new YOLOv8DetectionExecutor(ros2Node, clockOffsetEstimator, annotatedImageDemandSupplier);
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

         // CPU convert — uploading the full color frame just to get BGR is a GpuMat allocate that
         // OOMs when Ollama / other YOLO nets already own the device. YOLOv8Model.run() is CPU-in too.
         if (colorImage.getPixelFormat() != PixelFormat.BGR8)
         {
            Mat bgrMat = new Mat();
            colorImage.getPixelFormat().convertToPixelFormat(colorImage.getCpuImageMat(), bgrMat, PixelFormat.BGR8);
            colorImage.release();
            colorImage = colorImage.replaceImage(bgrMat, PixelFormat.BGR8);
         }

         yoloExecutor.runNextModel(colorImage, depthImage);

         colorImage.release();
         depthImage.release();
      }
      catch (InterruptedException ignored)
      {
      }
      catch (RuntimeException e)
      {
         if (!isCudaOutOfMemory(e))
            throw e;
         LogTools.error("YOLO ran out of GPU memory and will stop. Load fewer nets with -Dyolo.models.load=yolov8n-seg, "
                        + "or disable person YOLO with -Dalex.commands.vision.personViaYolo=false. {}",
                        e.getMessage());
         yoloExecutor.disableAllModels();
         interrupt();
      }
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

   private static boolean isCudaOutOfMemory(Throwable error)
   {
      while (error != null)
      {
         String message = error.getMessage();
         if (message != null && (message.contains("out of memory") || message.contains("(-217:Gpu API call)")))
            return true;
         error = error.getCause();
      }
      return false;
   }
}
