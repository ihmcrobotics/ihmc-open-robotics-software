package us.ihmc.humanoidBehaviors.tools.perception;

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
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.tools.lists.PairList;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class PlanarRegionSLAMTools
{
   // FITTING

   public static RigidBodyTransform findDriftCorrectionTransform(Map<PlanarRegion, PairList<PlanarRegion, Point3D>> matchesWithReferencePoints)
   {
      RigidBodyTransform bigT = new RigidBodyTransform();

      SolvePseudoInverseSvd solver = new SolvePseudoInverseSvd();

      int numberOfMatches = 0;
      for (PairList<PlanarRegion, Point3D> newDataRegionWithReferencePoints : matchesWithReferencePoints.values())
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

         for (ImmutablePair<PlanarRegion, Point3D> newDataRegionWithReferencePoint : matchesWithReferencePoints.get(mapRegion))
         {
            Vector3D cross = new Vector3D(mapPoint);
            cross.cross(normal);

            A.set(i, 0, cross.getX());
            A.set(i, 1, cross.getY());
            A.set(i, 2, cross.getZ());
            A.set(i, 3, normal.getX());
            A.set(i, 4, normal.getY());
            A.set(i, 5, normal.getZ());

            b.set(i, 0, -planarRegionPlane3D.distance(newDataRegionWithReferencePoint.getRight()));

            ++i;
         }
      }

      DenseMatrix64F x = new DenseMatrix64F(6, 1);

      DenseMatrix64F ATranspose = new DenseMatrix64F(A);
      CommonOps.transpose(ATranspose);

      DenseMatrix64F ATransposeTimesA = new DenseMatrix64F(6, 6);
      CommonOps.mult(ATranspose, A, ATransposeTimesA);

      DenseMatrix64F ATransposeB = new DenseMatrix64F(6, 1);
      CommonOps.mult(ATranspose, b, ATransposeB);

      solver.setA(ATransposeTimesA);
      solver.solve(ATransposeB, x);

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

   public static Map<PlanarRegion, PairList<PlanarRegion, Point3D>> filterMatchesBasedOn2DBoundingBoxShadow(Map<PlanarRegion, List<PlanarRegion>> matchesSoFar)
   {
      HashMap<PlanarRegion, PairList<PlanarRegion, Point3D>> normalFilteredMap = new HashMap<>();
      for (PlanarRegion mapRegion : matchesSoFar.keySet())
      {
         PairList<PlanarRegion, Point3D> shadowMatches = new PairList<>();

         for (PlanarRegion newDataRegion : matchesSoFar.get(mapRegion))
         {

            RigidBodyTransform mapTransformToWorld = PlanarRegionTools.getTransformToWorld(mapRegion);
            RigidBodyTransform worldToNewData = PlanarRegionTools.getTransformToWorld(newDataRegion);
            worldToNewData.invert();

            RigidBodyTransform transformFromNewDataToMapOrViceVersa = new RigidBodyTransform(); // TODO: fix this transform
            transformFromNewDataToMapOrViceVersa.set(mapTransformToWorld);
            transformFromNewDataToMapOrViceVersa.multiply(worldToNewData);

            PlanarRegion transformedToMapRegion = new PlanarRegion();
            transformedToMapRegion.set(newDataRegion);
            transformedToMapRegion.transform(transformFromNewDataToMapOrViceVersa);

            BoundingBox2D mapBoundingBox = PlanarRegionTools.getBoundingBox2DInLocal(mapRegion);
            BoundingBox2D projectedNewDataBoundingBox = PlanarRegionTools.getBoundingBox2DInLocal(transformedToMapRegion);

            boolean boundingBoxShadowsMapRegion = mapBoundingBox.intersectsEpsilon(projectedNewDataBoundingBox, 0.0);
            if (boundingBoxShadowsMapRegion)
            {

               BoundingBox2D intersection = GeometryTools.intersection(mapBoundingBox, projectedNewDataBoundingBox);
               Point2D intersectionCentroid = new Point2D();
               intersection.getCenterPoint(intersectionCentroid);

               Point3D newDataReferencePoint = new Point3D(intersectionCentroid);
               newDataReferencePoint.applyTransform(mapTransformToWorld);  // TODO: check

               shadowMatches.add(newDataRegion, newDataReferencePoint);
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
         newDataCollisions.put(planarRegion, localCollisions);

         for (PlanarRegion newRegion : newData.getPlanarRegionsAsList())
         {
            if (boxesIn3DIntersect(planarRegion, newRegion, 0.1))
            {
               localCollisions.add(newRegion);
            }
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
