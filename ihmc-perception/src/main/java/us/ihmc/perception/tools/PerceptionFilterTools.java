package us.ihmc.perception.tools;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHull;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullDecomposition;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullPruningFilteringTools;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotics.geometry.FramePlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.ArrayList;
import java.util.List;

/**
 * This class contains tools to both create and apply filters for perception data
 */
public class PerceptionFilterTools
{
   /**
    * Applies a bounding box filter on the planar region lists by chopping off parts of the regions that are outside the bounding box. Performs
    * a plane-by-plane chopping tool for each plane on the bounding box.
    *
    * @param planarRegionsList
    * @param boundingBox
    */
   public static void filterByBoundingBox(PlanarRegionsList planarRegionsList, BoundingBox3D boundingBox)
   {
      PlanarRegionsList result = null;

      Plane3D plane = new Plane3D(new Point3D(boundingBox.getMinX(), 0, 0), new Vector3D(1, 0, 0));
      result = PlanarRegionCuttingTools.cutByPlane(plane, planarRegionsList);

      plane = new Plane3D(new Point3D(boundingBox.getMaxX(), 0, 0), new Vector3D(-1, 0, 0));
      result = PlanarRegionCuttingTools.cutByPlane(plane, result);

      plane = new Plane3D(new Point3D(0, boundingBox.getMinY(), 0), new Vector3D(0, 1, 0));
      result = PlanarRegionCuttingTools.cutByPlane(plane, result);

      plane = new Plane3D(new Point3D(0, boundingBox.getMaxY(), 0), new Vector3D(0, -1, 0));
      result = PlanarRegionCuttingTools.cutByPlane(plane, result);

      planarRegionsList.clear();
      planarRegionsList.addPlanarRegions(result.getPlanarRegionsAsList());
   }

   /**
    * Removes regions that have surface normals that are orthogonal to the line joining the camera center to the region origin.
    * Uses parallel stream for efficient CPU usage.
    *
    * @param framePlanarRegionsList
    * @param maxAngleFromNormal
    */
   public static void filterShadowRegions(FramePlanarRegionsList framePlanarRegionsList, double maxAngleFromNormal)
   {
      double angleFromNormal = Math.toRadians(maxAngleFromNormal);
      double minDot = Math.cos(Math.PI / 2.0 - angleFromNormal);

      List<PlanarRegion> filteredList = framePlanarRegionsList.getPlanarRegionsList().getPlanarRegionsAsList().parallelStream().filter(region ->
      {
         Vector3D vectorToRegion = new Vector3D(region.getPoint());
         vectorToRegion.normalize();

         return Math.abs(vectorToRegion.dot(region.getNormal())) > minDot;
      }).toList();

      framePlanarRegionsList.getPlanarRegionsList().clear();
      framePlanarRegionsList.getPlanarRegionsList().addPlanarRegions(filteredList);
   }

   /**
    * This filter reduces the concave hull vertex count by remoting peaks, shallow angles and short edges along the bounding hull for
    * every region in the provided planar regions list. Uses parallel stream for efficient CPU usage.
    *
    * @param planarRegionsList
    * @param polygonizerParameters
    */
   public static void applyConcaveHullReduction(PlanarRegionsList planarRegionsList, PolygonizerParameters polygonizerParameters)
   {
      planarRegionsList.getPlanarRegionsAsList().parallelStream().forEach((region) -> applyConcaveHullReductionSingleRegion(region, polygonizerParameters));
   }

   public static void applyConcaveHullReductionSingleRegion(PlanarRegion region, PolygonizerParameters polygonizerParameters)
   {
      // Apply some simple filtering to reduce the number of vertices and hopefully the number of convex polygons.
      double shallowAngleThreshold = polygonizerParameters.getShallowAngleThreshold();
      double peakAngleThreshold = polygonizerParameters.getPeakAngleThreshold();
      double lengthThreshold = polygonizerParameters.getLengthThreshold();
      double depthThreshold = polygonizerParameters.getDepthThreshold();

      ConcaveHull concaveHull = new ConcaveHull(region.getConcaveHull());

      ConcaveHullPruningFilteringTools.filterOutPeaksAndShallowAngles(shallowAngleThreshold, peakAngleThreshold, concaveHull);
      ConcaveHullPruningFilteringTools.filterOutShortEdges(lengthThreshold, concaveHull);

      List<ConvexPolygon2D> decomposedPolygons = new ArrayList<>();
      ConcaveHullDecomposition.recursiveApproximateDecomposition(concaveHull, depthThreshold, decomposedPolygons);

      // Pack the data in PlanarRegion
      PlanarRegion filteredRegion = new PlanarRegion(region.getTransformToWorld(), concaveHull.getConcaveHullVertices(), decomposedPolygons);
      filteredRegion.setRegionId(region.getRegionId());

      region.set(filteredRegion);
   }
}
