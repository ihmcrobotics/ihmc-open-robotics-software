package us.ihmc.humanoidBehaviors.tools.perception;

import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionPolygonizer;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;

public class PointCloudPolygonizer
{
   private final ConcaveHullFactoryParameters concaveHullFactoryParameters = new ConcaveHullFactoryParameters();
   private final PolygonizerParameters polygonizerParameters = new PolygonizerParameters();

   private AtomicInteger id = new AtomicInteger(0);

   public PointCloudPolygonizer()
   {
   }

   public PlanarRegionsList polygonize(Map<Pair<Point3DReadOnly, Vector3DReadOnly>, List<Point3D>> pointsInRegions)
   {
      List<PlanarRegionSegmentationRawData> segmentationRawData = new ArrayList<>();
      for (Pair<Point3DReadOnly, Vector3DReadOnly> centerNormal : pointsInRegions.keySet())
      {
         List<Point3D> points = pointsInRegions.get(centerNormal);

         if (points.isEmpty())
            continue;

         PlanarRegionSegmentationRawData rawData = new PlanarRegionSegmentationRawData(id.getAndIncrement(),
                                                                                       centerNormal.getRight(),
                                                                                       centerNormal.getLeft(),
                                                                                       points);
         segmentationRawData.add(rawData);
      }

      return PlanarRegionPolygonizer.createPlanarRegionsList(segmentationRawData, concaveHullFactoryParameters, polygonizerParameters);
   }
}
