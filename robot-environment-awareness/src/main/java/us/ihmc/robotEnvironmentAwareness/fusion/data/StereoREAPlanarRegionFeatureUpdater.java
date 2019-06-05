package us.ihmc.robotEnvironmentAwareness.fusion.data;

import java.util.Collections;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.IntersectionEstimationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionIntersectionCalculator;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionPolygonizer;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationNodeData;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotEnvironmentAwareness.updaters.RegionFeaturesProvider;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class StereoREAPlanarRegionFeatureUpdater implements RegionFeaturesProvider
{
   private final StereoREAPlanarRegionSegmentationCalculator planarRegionSegmentationCalculator = new StereoREAPlanarRegionSegmentationCalculator();
   private PlanarRegionsList planarRegionsList = null;

   private final AtomicReference<ConcaveHullFactoryParameters> concaveHullFactoryParameters;
   private final AtomicReference<PolygonizerParameters> polygonizerParameters;
   private final AtomicReference<IntersectionEstimationParameters> intersectionEstimationParameters;
   
   private List<LineSegment3D> planarRegionsIntersections = null;

   public StereoREAPlanarRegionFeatureUpdater(Messager reaMessager)
   {
      concaveHullFactoryParameters = reaMessager.createInput(REAModuleAPI.PlanarRegionsConcaveHullParameters, new ConcaveHullFactoryParameters());
      polygonizerParameters = reaMessager.createInput(REAModuleAPI.PlanarRegionsPolygonizerParameters, new PolygonizerParameters());
      intersectionEstimationParameters = reaMessager.createInput(REAModuleAPI.PlanarRegionsIntersectionParameters, new IntersectionEstimationParameters());
   }

   public void updateLatestLidarImageFusionData(LidarImageFusionData lidarImageFusionData)
   {
      planarRegionSegmentationCalculator.updateFusionData(lidarImageFusionData);
      planarRegionSegmentationCalculator.initialize();
   }

   public void clear()
   {
      planarRegionsList = null;
   }

   public boolean update()
   {
      boolean calculationIsDone = planarRegionSegmentationCalculator.calculate();
      if (calculationIsDone)
      {
         List<PlanarRegionSegmentationRawData> rawData = planarRegionSegmentationCalculator.getSegmentationRawData();
         updateIntersections(rawData);
         updatePolygons(rawData);
         mergeIntoCurrentPlanarRegions();
         return true;
      }

      return false;
   }

   // TODO: for the online.
   public void mergeIntoCurrentPlanarRegions()
   {
      // handle current planar regions comparing getLatestSegmentationNodeData(List of SegmentationNodeData)
      // add new planar regions.
   }

   private void updatePolygons(List<PlanarRegionSegmentationRawData> rawData)
   {
      ConcaveHullFactoryParameters concaveHullFactoryParameters = this.concaveHullFactoryParameters.get();
      PolygonizerParameters polygonizerParameters = this.polygonizerParameters.get();

      planarRegionsList = PlanarRegionPolygonizer.createPlanarRegionsList(rawData, concaveHullFactoryParameters, polygonizerParameters);
   }
   
   private void updateIntersections(List<PlanarRegionSegmentationRawData> rawData)
   {
      planarRegionsIntersections = PlanarRegionIntersectionCalculator.computeIntersections(rawData, intersectionEstimationParameters.get());
   }

   @Override
   public List<PlanarRegionSegmentationNodeData> getSegmentationNodeData()
   {
      return null;
   }

   @Override
   public PlanarRegionsList getPlanarRegionsList()
   {
      return planarRegionsList;
   }

   @Override
   public int getNumberOfPlaneIntersections()
   {
      return planarRegionsIntersections == null ? 0 : planarRegionsIntersections.size();
   }

   @Override
   public LineSegment3D getIntersection(int index)
   {
      return planarRegionsIntersections.get(index);
   }
}
