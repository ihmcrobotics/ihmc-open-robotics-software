package us.ihmc.robotEnvironmentAwareness.updaters;

import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.LidarScanMessage;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.messager.Messager;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.converters.BoundingBoxMessageConverter;
import us.ihmc.robotEnvironmentAwareness.communication.converters.OcTreeMessageConverter;
import us.ihmc.robotEnvironmentAwareness.communication.converters.REAPlanarRegionsConverter;

public class REAModuleStateReporter
{
   private final Messager reaMessager;
   private final AtomicReference<Boolean> isOcTreeRequested;
   private final AtomicReference<Boolean> isOcTreeBoundingBoxRequested;
   private final AtomicReference<Boolean> arePlanarRegionsRequested;
   private final AtomicReference<Boolean> isPlanarRegionSegmentationRequested;
   private final AtomicReference<Boolean> arePlanarRegionsIntersectionsRequested;

   public REAModuleStateReporter(Messager reaMessager)
   {
      this.reaMessager = reaMessager;
      isOcTreeRequested = reaMessager.createInput(REAModuleAPI.RequestOctree, false);
      isOcTreeBoundingBoxRequested = reaMessager.createInput(REAModuleAPI.RequestBoundingBox, false);
      arePlanarRegionsRequested = reaMessager.createInput(REAModuleAPI.RequestPlanarRegions, false);
      isPlanarRegionSegmentationRequested = reaMessager.createInput(REAModuleAPI.RequestPlanarRegionSegmentation, false);
      arePlanarRegionsIntersectionsRequested = reaMessager.createInput(REAModuleAPI.RequestPlanarRegionsIntersections, false);
   }

   public void reportOcTreeState(NormalOcTree ocTree)
   {
      if (isOcTreeRequested.getAndSet(false))
         reaMessager.submitMessage(REAModuleAPI.OcTreeState, OcTreeMessageConverter.convertToMessage(ocTree));
      if (isOcTreeBoundingBoxRequested.getAndSet(false))
         reaMessager.submitMessage(REAModuleAPI.OcTreeBoundingBoxState, BoundingBoxMessageConverter.convertToMessage(ocTree.getBoundingBox()));
   }

   public void reportPlanarRegionsState(RegionFeaturesProvider regionFeaturesProvider)
   {
      if (regionFeaturesProvider.getPlanarRegionsList() != null)
         reaMessager.submitMessage(REAModuleAPI.PlanarRegionsState,
                                   PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(regionFeaturesProvider.getPlanarRegionsList()));
      //      if (regionFeaturesProvider.getPlanarRegionsList() != null && arePlanarRegionsRequested.getAndSet(false))
      //         reaMessager.submitMessage(REAModuleAPI.PlanarRegionsState,
      //                                   PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(regionFeaturesProvider.getPlanarRegionsList()));
      if (isPlanarRegionSegmentationRequested.getAndSet(false))
         reaMessager.submitMessage(REAModuleAPI.PlanarRegionsSegmentationState,
                                   REAPlanarRegionsConverter.createPlanarRegionSegmentationMessages(regionFeaturesProvider));
      if (arePlanarRegionsIntersectionsRequested.getAndSet(false))
         reaMessager.submitMessage(REAModuleAPI.PlanarRegionsIntersectionState, REAPlanarRegionsConverter.createLineSegment3dMessages(regionFeaturesProvider));
   }

   public void registerLidarScanMessage(LidarScanMessage message)
   {
      if (reaMessager.isMessagerOpen())
         reaMessager.submitMessage(REAModuleAPI.LidarScanState, new LidarScanMessage(message));
   }

   public void registerStereoVisionPointCloudMessage(StereoVisionPointCloudMessage message)
   {
      if (reaMessager.isMessagerOpen())
         reaMessager.submitMessage(REAModuleAPI.StereoVisionPointCloudState, new StereoVisionPointCloudMessage(message));
   }
}
