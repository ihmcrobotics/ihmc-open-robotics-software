package us.ihmc.perception.tools;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHull;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullCutter;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullDecomposition;
import us.ihmc.robotEnvironmentAwareness.planarRegion.slam.PlanarRegionSLAMTools;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.ArrayList;
import java.util.List;

public class PlanarRegionCuttingTools
{
   /**
    * Cut a planar region map by a plane in 3D world; keeping the pert "above" the plane. (i.e. In the direction the normal points)
    *
    * @param plane
    * @param regionsListToCut
    * @return cut planar regions list
    */
   public static PlanarRegionsList cutByPlane(Plane3D plane, PlanarRegionsList regionsListToCut)
   {
      PlanarRegionsList resultingRegions = new PlanarRegionsList();

      regionsListToCut.getPlanarRegionsAsList().parallelStream().map(region -> cutRegionByPlane(plane, region)).forEach(resultingRegions::addPlanarRegions);

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
      for (Point2DReadOnly point2D : region.getConcaveHull())
      {
         concaveHull.addVertex(new Point2D(point2D));
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

   /**
    * Given two planar regions, chop off the extra part that lives outside (in the opposite direction of the normal) of the other region, for each region.
    *
    * @param regionOne First region
    * @param regionTwo Second region
    * @return resultRegions List of planar regions after the cut
    */
   public static List<PlanarRegion> chopOffExtraPartsAtIntersection(PlanarRegion regionOne, PlanarRegion regionTwo)
   {
      Vector3D oneToTwo = new Vector3D();
      oneToTwo.sub(regionTwo.getPoint(), regionOne.getPoint());

      Plane3D planeOne = regionOne.getPlane();
      // Negate plane normal if the dot product of the normal and the vector from one to two is negative
      if (oneToTwo.dot(planeOne.getNormal()) < 0.0)
      {
         planeOne.getNormal().negate();
      }

      Plane3D planeTwo = regionTwo.getPlane();
      // Negate plane normal if the dot product of the normal and the vector from two to one is negative
      if (oneToTwo.dot(planeTwo.getNormal()) > 0.0)
      {
         planeTwo.getNormal().negate();
      }

      List<PlanarRegion> resultRegions = new ArrayList<>();
      resultRegions.addAll(cutRegionByPlane(planeTwo, regionOne));
      resultRegions.addAll(cutRegionByPlane(planeOne, regionTwo));
      return resultRegions;
   }

   /**
    * Method for chopping off extra parts in pairs of planar regions within a list
    */
   public static void chopOffExtraPartsFromIntersectingPairs(PlanarRegionsList planarRegions)
   {
      for (int i = 0; i < planarRegions.getNumberOfPlanarRegions(); i++)
      {
         PlanarRegion regionA = planarRegions.getPlanarRegion(i);

         for (int j = i + 1; j < planarRegions.getNumberOfPlanarRegions(); j++)
         {
            PlanarRegion regionB = planarRegions.getPlanarRegion(j);

            if (PlanarRegionSLAMTools.checkRegionsForIntersection(regionA, regionB))
            {
               List<PlanarRegion> resultRegions = PlanarRegionCuttingTools.chopOffExtraPartsAtIntersection(regionA, regionB);

               if (resultRegions.size() == 2)
               {
                  PlanarRegion finalRegionA = resultRegions.get(0);
                  PlanarRegion finalRegionB = resultRegions.get(1);

                  if (finalRegionA.getConcaveHull() == null || finalRegionB.getConcaveHull() == null)
                     continue;
                  else if (finalRegionA.getConcaveHull().size() < 3 || finalRegionB.getConcaveHull().size() < 3)
                     continue;

                  regionA.set(finalRegionA);
                  regionA.setRegionId(finalRegionA.getRegionId());

                  regionB.set(finalRegionB);
                  regionB.setRegionId(finalRegionB.getRegionId());
               }
            }
         }
      }
   }
}
