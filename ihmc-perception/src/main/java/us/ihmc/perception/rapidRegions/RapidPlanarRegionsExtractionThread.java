package us.ihmc.perception.rapidRegions;

import org.bytedeco.opencl.global.OpenCL;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.property.ROS2StoredPropertySet;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.comms.PerceptionComms;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.robotics.geometry.FramePlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.sensors.ImageSensor;

public class RapidPlanarRegionsExtractionThread extends RepeatingTaskThread
{
   private static final double UPDATE_FREQUENCY = 10.0;

   private final ROS2PublishSubscribeAPI ros2;

   private final OpenCLManager openCLManager;

   private final ImageSensor imageSensor;
   private final int depthImageKey;
   private final MutableReferenceFrame sensorFrame;

   private RapidPlanarRegionsExtractor extractor;
   private ROS2StoredPropertySet<RapidRegionsExtractorParameters> extractorParametersSync;

   public RapidPlanarRegionsExtractionThread(ROS2PublishSubscribeAPI ros2, OpenCLManager openCLManager, ImageSensor imageSensor, int depthImageKey)
   {
      super(imageSensor.getSensorName() + RapidPlanarRegionsExtractionThread.class.getSimpleName());
      setFrequencyLimit(UPDATE_FREQUENCY);

      this.ros2 = ros2;
      this.openCLManager = openCLManager;
      this.imageSensor = imageSensor;
      this.depthImageKey = depthImageKey;

      sensorFrame = new MutableReferenceFrame("PlanarRegionExtractionSensorFrame", ReferenceFrame.getWorldFrame());
   }

   @Override
   protected void runTask()
   {
      try
      {
         // Get an image from the sensor
         imageSensor.waitForGrab(); // TODO: Do we need this?
         RawImage depthImage = imageSensor.getImage(depthImageKey);

         // Initialize if not yet initialized
         if (extractor == null)
            initialize(depthImage);

         // Update parameters
         extractorParametersSync.updateAndPublishThrottledStatus();

         // Update the sensor frame using the depth image pose
         sensorFrame.update(transformToWorld -> transformToWorld.set(depthImage.getOrientation(), depthImage.getPosition()));

         // Extract the planar regions
         BytedecoImage bytedecoImage = new BytedecoImage(depthImage.getCpuImageMat().clone());
         bytedecoImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
         FramePlanarRegionsList framePlanarRegions = new FramePlanarRegionsList();
         extractor.update(bytedecoImage, sensorFrame.getReferenceFrame(), framePlanarRegions);
         extractor.setProcessing(false);
         bytedecoImage.destroy(openCLManager);

         // Get planar regions in world frame
         PlanarRegionsList planarRegionsInWorldFrame = framePlanarRegions.getPlanarRegionsList().copy();
         planarRegionsInWorldFrame.applyTransform(sensorFrame.getTransformToParent());

         // Publish the frame planar regions
         PerceptionMessageTools.publishFramePlanarRegionsList(framePlanarRegions, PerceptionAPI.PERSPECTIVE_RAPID_REGIONS, ros2);

         depthImage.release();
      } catch (InterruptedException ignored) {}
   }

   private void initialize(RawImage depthImage)
   {
      extractor = new RapidPlanarRegionsExtractor(openCLManager, depthImage.getIntrinsicsCopy());
      extractor.getDebugger().setEnabled(false);

      extractorParametersSync = new ROS2StoredPropertySet<>(ros2, PerceptionComms.PERSPECTIVE_RAPID_REGION_PARAMETERS, extractor.getParameters());
   }

   @Override
   public void kill()
   {
      super.kill();
      interrupt();

      extractor.destroy();
   }
}
