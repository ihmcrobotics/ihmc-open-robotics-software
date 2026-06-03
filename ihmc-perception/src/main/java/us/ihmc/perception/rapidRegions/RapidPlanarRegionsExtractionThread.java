package us.ihmc.perception.rapidRegions;

import org.bytedeco.opencv.opencv_core.GpuMat;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.property.ROS2StoredPropertySetGroup;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.RawImage;
import us.ihmc.sensors.CameraIntrinsics;
import us.ihmc.perception.comms.PerceptionComms;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.perception.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotics.geometry.FramePlanarRegionsList;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.jros2.ROS2Node;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.BlockingQueue;
import java.util.function.Consumer;

public class RapidPlanarRegionsExtractionThread extends RepeatingTaskThread
{
   private static final double UPDATE_FREQUENCY = 10.0;

   private final ROS2Helper ros2Helper;

   private final MutableReferenceFrame sensorFrame;
   private final BlockingQueue<RawImage> rawImageCollection;

   private RapidPlanarRegionsExtractor extractor;
   private final FramePlanarRegionsList framePlanarRegions = new FramePlanarRegionsList();
   private final ROS2StoredPropertySetGroup ros2StoredPropertySetGroup;

   private RapidRegionsExtractorParameters rapidRegionsExtractorParameters;
   private PolygonizerParameters polygonizerParameters;
   private ConcaveHullFactoryParameters concaveHullFactoryParameters;

   private final List<Consumer<FramePlanarRegionsList>> consumers = new ArrayList<>();

   public RapidPlanarRegionsExtractionThread(ROS2Node ros2Node, BlockingQueue<RawImage> rawImageCollection)
   {
      super(RapidPlanarRegionsExtractionThread.class.getSimpleName());
      setFrequencyLimit(UPDATE_FREQUENCY);

      this.rawImageCollection = rawImageCollection;
      this.ros2StoredPropertySetGroup = new ROS2StoredPropertySetGroup(ros2Node);
      this.ros2Helper = new ROS2Helper(ros2Node);
      this.sensorFrame = new MutableReferenceFrame("PlanarRegionExtractionSensorFrame", ReferenceFrame.getWorldFrame());
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
         ros2StoredPropertySetGroup.update();

         // Update the sensor frame using the depth image pose
         sensorFrame.update(transformToWorld -> transformToWorld.set(depthImage.getTransformToWorld()));

         // Extract the planar regions
         GpuMat depthImageDevice = depthImage.getGpuImageMat();
         extractor.update(depthImageDevice, sensorFrame.getReferenceFrame(), framePlanarRegions);

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
      CameraIntrinsics cameraIntrinsics = depthImage.getIntrinsicsCopy();
      extractor = new RapidPlanarRegionsExtractor(cameraIntrinsics);

      rapidRegionsExtractorParameters = extractor.getRapidRegionsExtractorParameters();
      polygonizerParameters = extractor.getPolygonizerParameters();
      concaveHullFactoryParameters = extractor.getConcaveHullFactoryParameters();

      ros2StoredPropertySetGroup.registerStoredPropertySet(PerceptionComms.PERSPECTIVE_RAPID_REGION_PARAMETERS, rapidRegionsExtractorParameters);
      ros2StoredPropertySetGroup.registerStoredPropertySet(PerceptionComms.PERSPECTIVE_POLYGONIZER_PARAMETERS, polygonizerParameters);
      ros2StoredPropertySetGroup.registerStoredPropertySet(PerceptionComms.PERSPECTIVE_CONCAVE_HULL_FACTORY_PARAMETERS, concaveHullFactoryParameters);
   }

   @Override
   public void kill()
   {
      super.kill();

      if (extractor != null)
      {
         extractor.destroy();
      }
   }

   public RapidPlanarRegionsExtractor getExtractor()
   {
      return extractor;
   }
}
