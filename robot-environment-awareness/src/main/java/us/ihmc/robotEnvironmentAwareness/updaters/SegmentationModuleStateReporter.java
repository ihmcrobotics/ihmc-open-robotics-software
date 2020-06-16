package us.ihmc.robotEnvironmentAwareness.updaters;

import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.messager.Messager;
import us.ihmc.robotEnvironmentAwareness.communication.SegmentationModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.converters.BoundingBoxMessageConverter;
import us.ihmc.robotEnvironmentAwareness.communication.converters.OcTreeMessageConverter;
import us.ihmc.robotEnvironmentAwareness.communication.converters.REAPlanarRegionsConverter;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.concurrent.atomic.AtomicReference;

public class SegmentationModuleStateReporter
{
   private final Messager reaMessager;
   private final AtomicReference<Boolean> arePlanarRegionsRequested;
   private final AtomicReference<Boolean> isPlanarRegionSegmentationRequested;
   private final AtomicReference<Boolean> arePlanarRegionsIntersectionsRequested;

   public SegmentationModuleStateReporter(Messager reaMessager)
   {
      this.reaMessager = reaMessager;
      arePlanarRegionsRequested = reaMessager.createInput(SegmentationModuleAPI.RequestPlanarRegions, false);
      isPlanarRegionSegmentationRequested = reaMessager.createInput(SegmentationModuleAPI.RequestPlanarRegionSegmentation, false);
      arePlanarRegionsIntersectionsRequested = reaMessager.createInput(SegmentationModuleAPI.RequestPlanarRegionsIntersections, false);
   }

   public void reportPlanarRegionsState(RegionFeaturesProvider regionFeaturesProvider)
   {
      PlanarRegionsList planarRegionsList = regionFeaturesProvider.getPlanarRegionsList();
      if (planarRegionsList != null && planarRegionsList.getNumberOfPlanarRegions() > 0 && arePlanarRegionsRequested.getAndSet(false))
         reaMessager.submitMessage(SegmentationModuleAPI.PlanarRegionsState, PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList));
      if (isPlanarRegionSegmentationRequested.getAndSet(false))
         reaMessager.submitMessage(SegmentationModuleAPI.PlanarRegionsSegmentationState,
                                   REAPlanarRegionsConverter.createPlanarRegionSegmentationMessages(regionFeaturesProvider));
      if (arePlanarRegionsIntersectionsRequested.getAndSet(false))
         reaMessager.submitMessage(SegmentationModuleAPI.PlanarRegionsIntersectionState, REAPlanarRegionsConverter.createLineSegment3dMessages(regionFeaturesProvider));
   }
}
