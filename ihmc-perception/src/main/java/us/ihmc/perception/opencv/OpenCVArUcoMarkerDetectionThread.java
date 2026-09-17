package us.ihmc.perception.opencv;

import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.perception.RawImage;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.sensors.ImageSensor;
import us.ihmc.tools.thread.SwapReference;

import java.util.function.Function;

public class OpenCVArUcoMarkerDetectionThread extends RepeatingTaskThread
{
   private final OpenCVArUcoMarkerDetector detector = new OpenCVArUcoMarkerDetector();
   private final SwapReference<OpenCVArUcoMarkerDetectionResults> resultSwapReference = new SwapReference<>(OpenCVArUcoMarkerDetectionResults::new);

   private OpenCVArUcoMarkerROS2Publisher publisher;
   private OpenCVArUcoMarkerDetectionResults resultsToPublish;

   private final ImageSensor imageSensor;
   private final MutableReferenceFrame sensorFrame = new MutableReferenceFrame();
   private final int colorImageKey;

   private boolean firstRun = true;

   public OpenCVArUcoMarkerDetectionThread(ImageSensor imageSensor, int colorImageKey)
   {
      super(imageSensor.getSensorName() + OpenCVArUcoMarkerDetectionThread.class.getSimpleName());
      this.imageSensor = imageSensor;
      this.colorImageKey = colorImageKey;
   }

   public OpenCVArUcoMarkerDetectionThread enablePublishing(ROS2Node ros2Node, Function<Integer, Double> markerSizeProvider)
   {
      resultsToPublish = new OpenCVArUcoMarkerDetectionResults();
      publisher = new OpenCVArUcoMarkerROS2Publisher(resultsToPublish, ros2Node, markerSizeProvider, sensorFrame.getReferenceFrame());
      return this;
   }

   @Override
   protected void runTask()
   {
      try
      {  // Get an image
         imageSensor.waitForGrab();
         RawImage colorImage = imageSensor.getImage(colorImageKey);

         if (firstRun)
         {  // Set the camera intrinsics on first run
            detector.setCameraIntrinsics(colorImage.getIntrinsicsCopy());
            firstRun = false;
         }

         // Update the sensor frame
         sensorFrame.update(transformToWorld -> transformToWorld.set(colorImage.getTransformToWorld()));

         // Detect markers
         detector.update(colorImage.getCpuImageMat());
         resultSwapReference.getForThreadOne().copyOutputData(detector);
         resultSwapReference.swap();

         if (publisher != null)
         {  // Publish results if enabled
            resultsToPublish.copyOutputData(detector);
            publisher.update();
         }

         colorImage.release();
      } catch (InterruptedException ignored) {}
   }

   /**
    * Get the result swap reference.
    * <p>
    * This is thread one, and you are thread two.
    * Synchronize over the swap reference while using the results.
    * Do not call {@link SwapReference#swap()} (this thread takes care of that for you).
    *
    * @return A swap reference to the latest results.
    */
   public SwapReference<OpenCVArUcoMarkerDetectionResults> getResultSwapReference()
   {
      return resultSwapReference;
   }

   public ReferenceFrame getSensorFrame()
   {
      return sensorFrame.getReferenceFrame();
   }

   @Override
   public void kill()
   {
      super.kill();
      interrupt();
   }
}
