package us.ihmc.perception.rapidRegions;

import org.bytedeco.opencl.global.OpenCL;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.property.ROS2StoredPropertySet;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.comms.PerceptionComms;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.robotics.geometry.FramePlanarRegionsList;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.ros2.ROS2Node;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.BlockingQueue;
import java.util.function.Consumer;

public class RapidPlanarRegionsExtractionThread extends RepeatingTaskThread
{
   private static final double UPDATE_FREQUENCY = 10.0;

   private final ROS2Node ros2Node;
   private final ROS2Helper ros2Helper;

   private final OpenCLManager openCLManager;

   private final MutableReferenceFrame sensorFrame;
   private final BlockingQueue<RawImage> rawImageCollection;

   private RapidPlanarRegionsExtractor extractor;
   private ROS2StoredPropertySet<RapidRegionsExtractorParameters> extractorParametersSync;
   private final FramePlanarRegionsList framePlanarRegions = new FramePlanarRegionsList();

   private final List<Consumer<FramePlanarRegionsList>> consumers = new ArrayList<>();

   public RapidPlanarRegionsExtractionThread(ROS2Node ros2Node, OpenCLManager openCLManager, BlockingQueue<RawImage> rawImageCollection)
   {
      super(RapidPlanarRegionsExtractionThread.class.getSimpleName());
      setFrequencyLimit(UPDATE_FREQUENCY);

      this.ros2Node = ros2Node;
      this.openCLManager = openCLManager;
      this.rawImageCollection = rawImageCollection;

      ros2Helper = new ROS2Helper(ros2Node);

      sensorFrame = new MutableReferenceFrame("PlanarRegionExtractionSensorFrame", ReferenceFrame.getWorldFrame());
   }

   public void addPlanarRegionsConsumer(Consumer<FramePlanarRegionsList> planarRegionsConsumer)
   {
      consumers.add(planarRegionsConsumer);
   }

   @Override
   protected void runTask()
   {
      try
      {
         // Get an image from the sensor
         RawImage depthImage = rawImageCollection.take();

         // Initialize if not yet initialized
         if (extractor == null)
            initialize(depthImage);

         // Update parameters
         extractorParametersSync.updateAndPublishThrottledStatus();

         // Update the sensor frame using the depth image pose
         sensorFrame.update(transformToWorld -> transformToWorld.set(depthImage.getTransformToWorld()));

         // Extract the planar regions
         BytedecoImage bytedecoImage = new BytedecoImage(depthImage.getCpuImageMat().clone());
         bytedecoImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
         extractor.update(bytedecoImage, sensorFrame.getReferenceFrame(), framePlanarRegions);
         extractor.setProcessing(false);
         bytedecoImage.destroy(openCLManager);

         // Give copies to consumers
         for (Consumer<FramePlanarRegionsList> consumer : consumers)
            consumer.accept(framePlanarRegions.copy());

         // Publish the frame planar regions
         PerceptionMessageTools.publishFramePlanarRegionsList(framePlanarRegions, PerceptionAPI.PERSPECTIVE_RAPID_REGIONS, ros2Helper);

         depthImage.release();
      }
      catch (InterruptedException e)
      {
         // Do nothing
      }
   }

   private void initialize(RawImage depthImage)
   {
      extractor = new RapidPlanarRegionsExtractor(openCLManager, depthImage.getIntrinsicsCopy());
      extractor.getDebugger().setEnabled(false);

      extractorParametersSync = new ROS2StoredPropertySet<>(ros2Node, PerceptionComms.PERSPECTIVE_RAPID_REGION_PARAMETERS, extractor.getParameters());
   }

   @Override
   public void kill()
   {
      super.kill();

      if (extractor != null)
         extractor.destroy();
   }
}
