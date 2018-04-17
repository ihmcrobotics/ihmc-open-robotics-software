package us.ihmc.robotEnvironmentAwareness.updaters;

import java.util.List;

import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationNodeData;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public interface RegionFeaturesProvider
{
   List<PlanarRegionSegmentationNodeData> getSegmentationNodeData();
   
   PlanarRegionsList getPlanarRegionsList();

   int getNumberOfPlaneIntersections();

   LineSegment3D getIntersection(int index);

}