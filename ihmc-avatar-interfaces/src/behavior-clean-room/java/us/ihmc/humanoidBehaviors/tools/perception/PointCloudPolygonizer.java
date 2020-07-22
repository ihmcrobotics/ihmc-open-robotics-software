package us.ihmc.humanoidBehaviors.tools.perception;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionPolygonizer;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class PointCloudPolygonizer
{
   private final ConcaveHullFactoryParameters concaveHullFactoryParameters = new ConcaveHullFactoryParameters();
   private final PolygonizerParameters polygonizerParameters = new PolygonizerParameters();

   private volatile int id = 0;

   public PointCloudPolygonizer()
   {
   }

   public PlanarRegionsList polygonize(HashMap<PlanarRegion, List<Point3D>> pointsInRegions)
   {
      List<PlanarRegionSegmentationRawData> segmentationRawData = new ArrayList<>();
      for (PlanarRegion originalRegion : pointsInRegions.keySet())
      {
         List<Point3D> points = pointsInRegions.get(originalRegion);

         if (points.isEmpty())
            continue;

         Point3D center = new Point3D();
         originalRegion.getBoundingBox3dInWorld().getCenterPoint(center);
         PlanarRegionSegmentationRawData rawData = new PlanarRegionSegmentationRawData(id++,
                                                                                       originalRegion.getNormal(),
                                                                                       center,
                                                                                       points);
         segmentationRawData.add(rawData);
      }

      return PlanarRegionPolygonizer.createPlanarRegionsList(segmentationRawData, concaveHullFactoryParameters, polygonizerParameters);
   }
}
