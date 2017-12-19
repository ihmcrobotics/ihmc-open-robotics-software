package us.ihmc.robotEnvironmentAwareness.planarRegion;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHull;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullCollection;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullDecomposition;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullPruningFilteringTools;
import us.ihmc.robotEnvironmentAwareness.geometry.SimpleConcaveHullFactory;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionSegmentationDataExporter;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public abstract class PlanarRegionPolygonizer
{
   public static PlanarRegionsList createPlanarRegionsList(List<PlanarRegionSegmentationRawData> rawData,
                                                           ConcaveHullFactoryParameters concaveHullFactoryParameters,
                                                           PolygonizerParameters polygonizerParameters)
   {
      return createPlanarRegionsList(rawData, concaveHullFactoryParameters, polygonizerParameters, null);
   }

   public static PlanarRegionsList createPlanarRegionsList(List<PlanarRegionSegmentationRawData> rawData,
                                                           ConcaveHullFactoryParameters concaveHullFactoryParameters,
                                                           PolygonizerParameters polygonizerParameters, PlanarRegionSegmentationDataExporter dataExporter)
   {
      return new PlanarRegionsList(createPlanarRegions(rawData, concaveHullFactoryParameters, polygonizerParameters, dataExporter));
   }

   private static List<PlanarRegion> createPlanarRegions(List<PlanarRegionSegmentationRawData> rawData,
                                                         ConcaveHullFactoryParameters concaveHullFactoryParameters, PolygonizerParameters polygonizerParameters,
                                                         PlanarRegionSegmentationDataExporter dataExporter)
   {
      List<List<PlanarRegion>> regions = rawData.parallelStream()
                                                .filter(data -> data.size() >= polygonizerParameters.getMinNumberOfNodes())
                                                .map(data -> createPlanarRegion(data, concaveHullFactoryParameters, polygonizerParameters, dataExporter))
                                                .filter(region -> region != null)
                                                .collect(Collectors.toList());

      List<PlanarRegion> flattenedRegions = new ArrayList<>();
      for (List<PlanarRegion> regionsSublist : regions)
      {
         regionsSublist.forEach(flattenedRegions::add);
      }
      return flattenedRegions;
   }

   private static List<PlanarRegion> createPlanarRegion(PlanarRegionSegmentationRawData rawData, ConcaveHullFactoryParameters concaveHullFactoryParameters,
                                                        PolygonizerParameters polygonizerParameters, PlanarRegionSegmentationDataExporter dataExporter)
   {
      try
      {
         // First compute the set of concave hulls for this region
         List<Point2D> pointCloudInPlane = rawData.getPointCloudInPlane();
         List<LineSegment2D> intersections = rawData.getIntersections();
         ConcaveHullCollection concaveHullCollection = SimpleConcaveHullFactory.createConcaveHullCollection(pointCloudInPlane, intersections,
                                                                                                            concaveHullFactoryParameters);

         // Apply some simple filtering to reduce the number of vertices and hopefully the number of convex polygons.
         double shallowAngleThreshold = polygonizerParameters.getShallowAngleThreshold();
         double peakAngleThreshold = polygonizerParameters.getPeakAngleThreshold();
         double lengthThreshold = polygonizerParameters.getLengthThreshold();

         for (int i = 0; i < 5; i++)
         {
            ConcaveHullPruningFilteringTools.filterOutPeaksAndShallowAngles(shallowAngleThreshold, peakAngleThreshold, concaveHullCollection);
            ConcaveHullPruningFilteringTools.filterOutShortEdges(lengthThreshold, concaveHullCollection);
         }

         List<PlanarRegion> planarRegions = new ArrayList<>();

         for (ConcaveHull concaveHull : concaveHullCollection)
         {
            // Decompose the concave hulls into convex polygons
            double depthThreshold = polygonizerParameters.getDepthThreshold();
            List<ConvexPolygon2D> decomposedPolygons = new ArrayList<>();
            ConcaveHullDecomposition.recursiveApproximateDecomposition(concaveHull, depthThreshold, decomposedPolygons);

            // Pack the data in PlanarRegion
            RigidBodyTransform transformToWorld = rawData.getTransformFromLocalToWorld();
            Point2D[] concaveHullsVertices = new Point2D[concaveHull.getNumberOfVertices()];
            concaveHull.getConcaveHullVertices().toArray(concaveHullsVertices);

            PlanarRegion planarRegion = new PlanarRegion(transformToWorld, concaveHullsVertices, decomposedPolygons);
            planarRegion.setRegionId(rawData.getRegionId());
            planarRegions.add(planarRegion);
         }

         return planarRegions;
      }
      catch (RuntimeException e)
      {
         if (dataExporter == null)
         {
            e.printStackTrace();
         }
         else
         {
            PrintTools.error("Caught following exception: " + e.getMessage() + ", exporting segmentation data.");
            dataExporter.exportSegmentationRawData(rawData);
         }
         return null;
      }
   }
}
