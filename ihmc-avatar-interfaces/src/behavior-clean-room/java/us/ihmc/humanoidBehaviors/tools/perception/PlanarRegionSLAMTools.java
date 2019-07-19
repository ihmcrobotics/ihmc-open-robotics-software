package us.ihmc.humanoidBehaviors.tools.perception;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.ejml.alg.dense.linsol.svd.SolvePseudoInverseSvd;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.shape.collision.EuclidShape3DCollisionResult;
import us.ihmc.euclid.shape.collision.gjk.GilbertJohnsonKeerthiCollisionDetector;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.tools.lists.PairList;

public class PlanarRegionSLAMTools
{
   // FITTING

   /**
    * Uses the algorithm on the slides at
    * http://resources.mpi-inf.mpg.de/deformableShapeMatching/EG2011_Tutorial/slides/2.1%20Rigid%20ICP.pdf
    * pages 12-14
    * 
    * @param matchesWithReferencePoints
    * @return
    */
   public static RigidBodyTransform findDriftCorrectionTransform(Map<PlanarRegion, PairList<PlanarRegion, Point2D>> matchesWithReferencePoints)
   {
      RigidBodyTransform bigT = new RigidBodyTransform();

      SolvePseudoInverseSvd solver = new SolvePseudoInverseSvd();

      int numberOfMatches = 0;
      for (PairList<PlanarRegion, Point2D> newDataRegionWithReferencePoints : matchesWithReferencePoints.values())
      {
         numberOfMatches += newDataRegionWithReferencePoints.size();
      }

      DenseMatrix64F A = new DenseMatrix64F(numberOfMatches, 6);
      DenseMatrix64F b = new DenseMatrix64F(numberOfMatches, 1); // negative distance to planes

      int i = 0;
      for (PlanarRegion mapRegion : matchesWithReferencePoints.keySet())
      {
         Plane3D planarRegionPlane3D = mapRegion.getPlane();
         Vector3DReadOnly normal = planarRegionPlane3D.getNormal();
         Point3DReadOnly mapPoint = planarRegionPlane3D.getPoint();

         for (ImmutablePair<PlanarRegion, Point2D> newDataRegionWithReferencePoint : matchesWithReferencePoints.get(mapRegion))
         {
            PlanarRegion newPlanarRegion = newDataRegionWithReferencePoint.getLeft();
            Point2D referencePointInNewDataLocal = newDataRegionWithReferencePoint.getRight();

            Point3D referencePointInWorld = new Point3D(referencePointInNewDataLocal);
            RigidBodyTransform transformFromNewDataToWorld = new RigidBodyTransform();
            newPlanarRegion.getTransformToWorld(transformFromNewDataToWorld);
            transformFromNewDataToWorld.transform(referencePointInWorld);

            Vector3D cross = new Vector3D(referencePointInWorld);
            cross.cross(normal);

            A.set(i, 0, cross.getX());
            A.set(i, 1, cross.getY());
            A.set(i, 2, cross.getZ());
            A.set(i, 3, normal.getX());
            A.set(i, 4, normal.getY());
            A.set(i, 5, normal.getZ());

            b.set(i, 0, -planarRegionPlane3D.distance(referencePointInWorld));

            ++i;
         }
      }
      LogTools.info("numberReferencePoints: {}", i);
      LogTools.info("A: {}", A);
      LogTools.info("b: {}", b);

      DenseMatrix64F x = new DenseMatrix64F(6, 1);

      DenseMatrix64F ATranspose = new DenseMatrix64F(A);
      CommonOps.transpose(ATranspose);

      DenseMatrix64F ATransposeTimesA = new DenseMatrix64F(6, 6);
      CommonOps.mult(ATranspose, A, ATransposeTimesA);

      LogTools.info("ATransposeTimesA: {}", ATransposeTimesA);

      DenseMatrix64F ATransposeB = new DenseMatrix64F(6, 1);
      LogTools.info("ATransposeB: {}", ATransposeTimesA);
      CommonOps.mult(ATranspose, b, ATransposeB);

      solver.setA(ATransposeTimesA);
      solver.solve(ATransposeB, x);
      LogTools.info("x: {}", x);

      double rx = x.get(0, 0);
      double ry = x.get(1, 0);
      double rz = x.get(2, 0);
      double tx = x.get(3, 0);
      double ty = x.get(4, 0);
      double tz = x.get(5, 0);

      RotationMatrix rotationMatrix = new RotationMatrix(new Vector3D(rx, ry, rz));
      Vector3D translation = new Vector3D(tx, ty, tz);

      bigT.set(rotationMatrix, translation);

      return bigT;
   }

   // MERGING

   public static Map<PlanarRegion, List<PlanarRegion>> filterMatchesBasedOnNormalSimilarity(Map<PlanarRegion, List<PlanarRegion>> matchesSoFar,
                                                                                            double threshold)
   {
      HashMap<PlanarRegion, List<PlanarRegion>> normalFilteredMap = new HashMap<>();
      for (PlanarRegion mapRegion : matchesSoFar.keySet())
      {
         ArrayList<PlanarRegion> normalMatches = new ArrayList<>();

         for (PlanarRegion newDataRegion : matchesSoFar.get(mapRegion))
         {
            double dot = newDataRegion.getNormal().dot(mapRegion.getNormal());
            if (dot > threshold)
            {
               normalMatches.add(newDataRegion);
            }
         }

         if (!normalMatches.isEmpty())
         {
            normalFilteredMap.put(mapRegion, normalMatches);
         }
      }
      return normalFilteredMap;
   }

   public static Map<PlanarRegion, PairList<PlanarRegion, Point2D>> filterMatchesBasedOn2DBoundingBoxShadow(Map<PlanarRegion, List<PlanarRegion>> matchesSoFar)
   {
      HashMap<PlanarRegion, PairList<PlanarRegion, Point2D>> normalFilteredMap = new HashMap<>();
      for (PlanarRegion mapRegion : matchesSoFar.keySet())
      {
         PairList<PlanarRegion, Point2D> shadowMatches = new PairList<>();

         for (PlanarRegion newDataRegion : matchesSoFar.get(mapRegion))
         {
            RigidBodyTransform transformFromWorldToMap = PlanarRegionTools.getTransformToLocal(mapRegion);
            RigidBodyTransform transformFromNewDataToWorld = PlanarRegionTools.getTransformToWorld(newDataRegion);

            BoundingBox2D newDataRegionBoundingBoxInLocal = PlanarRegionTools.getBoundingBox2DInLocal(newDataRegion);
            Point3D newDataBoundingBoxMinPoint = new Point3D(newDataRegionBoundingBoxInLocal.getMinPoint());
            Point3D newDataBoundingBoxMaxPoint = new Point3D(newDataRegionBoundingBoxInLocal.getMaxPoint());

            transformFromNewDataToWorld.transform(newDataBoundingBoxMinPoint);
            transformFromNewDataToWorld.transform(newDataBoundingBoxMaxPoint);

            transformFromWorldToMap.transform(newDataBoundingBoxMinPoint);
            transformFromWorldToMap.transform(newDataBoundingBoxMaxPoint);

            // If the planes are perfectly aligned, then the z coordinates will be zero.
            double minX = Math.min(newDataBoundingBoxMinPoint.getX(), newDataBoundingBoxMaxPoint.getX());
            double minY = Math.min(newDataBoundingBoxMinPoint.getY(), newDataBoundingBoxMaxPoint.getY());
            double maxX = Math.max(newDataBoundingBoxMinPoint.getX(), newDataBoundingBoxMaxPoint.getX());
            double maxY = Math.max(newDataBoundingBoxMinPoint.getY(), newDataBoundingBoxMaxPoint.getY());

            Point2D newDataBoundingBoxMinPoint2D = new Point2D(minX, minY);
            Point2D newDataBoundingBoxMaxPoint2D = new Point2D(maxX, maxY);

            BoundingBox2D newDataRegionBoundingBoxProjectedToMapLocal = new BoundingBox2D(newDataBoundingBoxMinPoint2D, newDataBoundingBoxMaxPoint2D);
            BoundingBox2D mapBoundingBoxInMapLocal = PlanarRegionTools.getBoundingBox2DInLocal(mapRegion);

            //TODO: Parameterize the epsilon...
            boolean boundingBoxShadowsMapRegion = mapBoundingBoxInMapLocal.intersectsEpsilon(newDataRegionBoundingBoxProjectedToMapLocal, -0.001);
            if (boundingBoxShadowsMapRegion)
            {
               BoundingBox2D intersection = GeometryTools.intersection(mapBoundingBoxInMapLocal, newDataRegionBoundingBoxProjectedToMapLocal);

               if (intersection == null)
               {
                  LogTools.error("Woops. Should never get here!!");
                  LogTools.error("mapBoundingBoxInMapLocal = " + mapBoundingBoxInMapLocal);
                  LogTools.error("newDataRegionBoundingBoxProjectedToMapLocal = " + newDataRegionBoundingBoxProjectedToMapLocal);
                  continue;
               }

               Point2D intersectionCentroid = new Point2D();
               intersection.getCenterPoint(intersectionCentroid);

               Point3D newDataReferencePoint = new Point3D(intersectionCentroid);
               newDataReferencePoint.applyInverseTransform(transformFromWorldToMap);
               newDataReferencePoint.applyInverseTransform(transformFromNewDataToWorld);

               Point2D newDataReferencePointInNewDataLocal = new Point2D(newDataReferencePoint.getX(), newDataReferencePoint.getY());

               shadowMatches.add(newDataRegion, newDataReferencePointInNewDataLocal);
            }
         }

         if (!shadowMatches.isEmpty())
         {
            normalFilteredMap.put(mapRegion, shadowMatches);
         }
      }
      return normalFilteredMap;
   }

   public static Map<PlanarRegion, List<PlanarRegion>> detectLocalBoundingBox3DCollisions(PlanarRegionsList map, PlanarRegionsList newData)
   {
      HashMap<PlanarRegion, List<PlanarRegion>> newDataCollisions = new HashMap<>();
      for (PlanarRegion planarRegion : map.getPlanarRegionsAsList())
      {
         ArrayList<PlanarRegion> localCollisions = new ArrayList<>();

         for (PlanarRegion newRegion : newData.getPlanarRegionsAsList())
         {
            if (boxesIn3DIntersect(planarRegion, newRegion, 0.1))
            {
               localCollisions.add(newRegion);
            }
         }

         if (!localCollisions.isEmpty())
         {
            newDataCollisions.put(planarRegion, localCollisions);
         }
      }
      return newDataCollisions;
   }

   public static boolean boxesIn3DIntersect(PlanarRegion a, PlanarRegion b, double boxHeight)
   {
      Box3D boxA = PlanarRegionTools.getLocalBoundingBox3DInWorld(a, boxHeight);
      Box3D boxB = PlanarRegionTools.getLocalBoundingBox3DInWorld(b, boxHeight);

      GilbertJohnsonKeerthiCollisionDetector gjkCollisionDetector = new GilbertJohnsonKeerthiCollisionDetector();

      EuclidShape3DCollisionResult collisionResult = gjkCollisionDetector.evaluateCollision(boxA, boxB);

      return collisionResult.areShapesColliding();
   }

   public static boolean boundingBoxesIntersect(PlanarRegion a, PlanarRegion b)
   {
      Box3D boxA = GeometryTools.convertBoundingBoxToBox(a.getBoundingBox3dInWorld());
      Box3D boxB = GeometryTools.convertBoundingBoxToBox(b.getBoundingBox3dInWorld());

      GilbertJohnsonKeerthiCollisionDetector gjkCollisionDetector = new GilbertJohnsonKeerthiCollisionDetector();

      EuclidShape3DCollisionResult collisionResult = gjkCollisionDetector.evaluateCollision(boxA, boxB);

      return collisionResult.areShapesColliding();
   }
}
