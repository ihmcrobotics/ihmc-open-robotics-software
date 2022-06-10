package us.ihmc.pathPlanning.visibilityGraphs.tools;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.perception.geometry.ConcaveHull;
import us.ihmc.perception.geometry.ConcaveHullCutter;
import us.ihmc.perception.geometry.ConcaveHullDecomposition;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.ArrayList;
import java.util.List;

public class PlanarRegionsListCutter
{
   /**
    * Cut a planar region map by a plane in 3D world; keeping the pert "above" the plane. (i.e. In the direction the normal points)
    *
    * @param plane
    * @param map
    * @return cut planar regions list
    */
   public static PlanarRegionsList cutByPlane(Plane3D plane, PlanarRegionsList map)
   {
      PlanarRegionsList resultingRegions = new PlanarRegionsList();

      for (PlanarRegion mapRegion : map.getPlanarRegionsAsList())
      {
         resultingRegions.addPlanarRegions(cutRegionByPlane(plane, mapRegion));
      }

      return resultingRegions;
   }

   /**
    * Cut a planar region by a plane in 3D world; keeping the part "above" the plane. (i.e. In the direction the normal points)
    *
    * @param plane
    * @param region
    * @return cut planar regions list
    */
   public static List<PlanarRegion> cutRegionByPlane(Plane3D plane, PlanarRegion region)
   {

      RigidBodyTransformReadOnly regionToWorld = region.getTransformToWorld();
      Line3D cuttingLine3D = new Line3D();
      Line2D cuttingLine2D = new Line2D();

      GeometryTools.getIntersectionBetweenTwoPlanes(plane, region.getPlane(), cuttingLine3D);
      cuttingLine3D.applyTransform(region.getTransformToLocal());
      cuttingLine2D.set(cuttingLine3D);

      // direction of cut plane 3D: project to new plane, normalize TODO correct?
      Vector3D planeNormal3D = new Vector3D(plane.getNormal());
      region.transformFromWorldToLocal(planeNormal3D);
      Vector2D planeNormal2D = new Vector2D(planeNormal3D);
      planeNormal2D.normalize();

      cuttingLine2D.getDirection().set(planeNormal2D.getY(), -planeNormal2D.getX()); // make sure left side of line is plane normal

      ConcaveHull concaveHull = new ConcaveHull(); // TODO make this one line
      for (Point2D point2D : region.getConcaveHull())
      {
         concaveHull.addVertex(point2D);
      }

      List<ConcaveHull> resultingConcaveHulls = ConcaveHullCutter.cutPolygonToLeftOfLine(concaveHull, cuttingLine2D);

      ArrayList<PlanarRegion> resultingRegions = new ArrayList<>();

      for (ConcaveHull resultingConcaveHull : resultingConcaveHulls)
      {
         List<ConvexPolygon2D> decomposedPolygons = new ArrayList<>();
         ConcaveHullDecomposition.recursiveApproximateDecomposition(resultingConcaveHull, 0.0, decomposedPolygons); // TODO: tune depth threshold?

         PlanarRegion resultingRegion = new PlanarRegion(region.getTransformToWorld(), resultingConcaveHull.getConcaveHullVertices(), decomposedPolygons);
         resultingRegion.setRegionId(region.getRegionId());
         resultingRegions.add(resultingRegion);
      }

      return resultingRegions;
   }
}
