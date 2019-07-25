package us.ihmc.humanoidBehaviors.tools.perception;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.ejml.alg.dense.linsol.svd.SolvePseudoInverseSvd;
import org.ejml.data.DenseMatrix64F;
import org.ejml.interfaces.decomposition.SingularValueDecomposition;
import org.ejml.ops.CommonOps;

import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.shape.collision.EuclidShape3DCollisionResult;
import us.ihmc.euclid.shape.collision.gjk.GilbertJohnsonKeerthiCollisionDetector;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.tools.lists.PairList;

public class PlanarRegionSLAMTools
{
   private static boolean verbose = false;

   /**
    * Uses the algorithm on the slides at
    * http://resources.mpi-inf.mpg.de/deformableShapeMatching/EG2011_Tutorial/slides/2.1%20Rigid%20ICP.pdf
    * pages 12-14
    * 
    * @param matchesWithReferencePoints
    * @return
    */
   public static RigidBodyTransform findDriftCorrectionTransform(Map<PlanarRegion, PairList<PlanarRegion, Point2D>> matchesWithReferencePoints,
                                                                 PlanarRegionSLAMParameters parameters)
   {
      RigidBodyTransform bigT = new RigidBodyTransform();

      SolvePseudoInverseSvd solver = new SolvePseudoInverseSvd();

      int numberOfMatches = 0;
      for (PairList<PlanarRegion, Point2D> newDataRegionWithReferencePoints : matchesWithReferencePoints.values())
      {
         numberOfMatches += newDataRegionWithReferencePoints.size();
      }

      if (numberOfMatches == 0)
         return new RigidBodyTransform();

      DenseMatrix64F A = new DenseMatrix64F(numberOfMatches, 6);
      DenseMatrix64F b = new DenseMatrix64F(numberOfMatches, 1); // negative distance to planes

      int i = 0;
      for (PlanarRegion mapRegion : matchesWithReferencePoints.keySet())
      {
         Plane3D planarRegionPlane3D = mapRegion.getPlane();
         Vector3DReadOnly normal = planarRegionPlane3D.getNormal();

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

            double signedDistanceFromPointToPlane = planarRegionPlane3D.signedDistance(referencePointInWorld);

            //TODO: Reject outliers that have a large distance to the plane.
            if (verbose && Math.abs(signedDistanceFromPointToPlane) > 0.05)
            {
               System.err.println("\n\n*******************\nsignedDistanceFromPointToPlane = " + signedDistanceFromPointToPlane);
               System.err.println("referencePointInWorld = " + referencePointInWorld);
               System.err.println("planarRegionPlane3D = " + planarRegionPlane3D);
               System.err.println("mapRegion = " + mapRegion);
               System.err.println("newPlanarRegion = " + newPlanarRegion);
               System.err.println("normal = " + normal);
               System.err.println("normalOfNewPlanarRegion = " + newPlanarRegion.getNormal());
            }

            b.set(i, 0, -signedDistanceFromPointToPlane);

            ++i;
         }
      }
      if (verbose)
      {
         LogTools.info("numberReferencePoints: {}", i);
         LogTools.info("A: {}", A);
         LogTools.info("b: {}", b);
      }

      DenseMatrix64F x = new DenseMatrix64F(6, 1);

      DenseMatrix64F ATransposeTimesA = new DenseMatrix64F(6, 6);
      CommonOps.multInner(A, ATransposeTimesA);

      DenseMatrix64F ATransposeB = new DenseMatrix64F(6, 1);
      CommonOps.multTransA(A, b, ATransposeB);

      // Use damped least squares (also called regularized least squares) to prevent blow up when data is sparse.
      // See https://www2.math.uconn.edu/~leykekhman/courses/MARN_5898/Lectures/Linear_least_squares_reg.pdf
      DenseMatrix64F lambdaI = CommonOps.identity(6);
      CommonOps.scale(parameters.getDampedLeastSquaresLambda(), lambdaI);

      DenseMatrix64F ATransposeTimesAPlusLambdaI = new DenseMatrix64F(6, 6);
      CommonOps.add(ATransposeTimesA, lambdaI, ATransposeTimesAPlusLambdaI);

      solver.setA(ATransposeTimesAPlusLambdaI);

      solver.solve(ATransposeB, x);

      if (verbose)
      {
         SingularValueDecomposition<DenseMatrix64F> decomposition = solver.getDecomposition();
         double[] singularValues = decomposition.getSingularValues();
         LogTools.info("singularValues = " + doubleArrayToString(singularValues));
         LogTools.info("ATransposeTimesA: {}", ATransposeTimesA);
         LogTools.info("ATransposeB: {}", ATransposeTimesA);
         LogTools.info("x: {}", x);
      }

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

   private static String doubleArrayToString(double[] singularValues)
   {
      String returnString = "(";

      for (int i = 0; i < singularValues.length; i++)
      {
         returnString += singularValues[i];
         if (i != singularValues.length - 1)
         {
            returnString = returnString + ", ";
         }
      }

      returnString += ")";
      return returnString;
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

   /**
    * Filters the match set down further.
    *
    * For each match so far, find newData regions that "shadow" the corresponding map region.
    * By "shadow" we mean as if shining an orthogonal light source at the map region and seeing the shadow cast upon it
    * by a newData region. Shadow must overlap more than minimumRegionOverlapDistance.
    *
    * Then, for each shadow match, add four or more reference points, like the vertices of the shadow.
    * These points are the "fit" points that are put into the fit optimization algorithm.
    *
    * @param minimumRegionOverlapDistance
    * @param matchesSoFar
    * @return mapToShadowMatchAndFitPoints
    */
   public static Map<PlanarRegion, PairList<PlanarRegion, Point2D>>
   filterMatchesBasedOn2DBoundingBoxShadow(double minimumRegionOverlapDistance, Map<PlanarRegion, List<PlanarRegion>> matchesSoFar)
   {
      HashMap<PlanarRegion, PairList<PlanarRegion, Point2D>> mapToShadowMatchAndFitPoints = new HashMap<>();
      for (PlanarRegion mapRegion : matchesSoFar.keySet())
      {
         PairList<PlanarRegion, Point2D> shadowMatches = new PairList<>();

         for (PlanarRegion newDataRegion : matchesSoFar.get(mapRegion))
         {
            BoundingBox2D newDataRegionBoundingBoxProjectedToMapLocal
                  = computeNewDataRegionBoundingBoxProjectedToMapLocal(newDataRegion, mapRegion.getTransformToLocal(), newDataRegion.getTransformToWorld());

            BoundingBox2D mapBoundingBoxInMapLocal = PlanarRegionTools.getLocalBoundingBox2DInLocal(mapRegion);

            //TODO: Test case for minimumRegionOverlapDistance
            boolean boundingBoxShadowsMapRegion
                  = mapBoundingBoxInMapLocal.intersectsEpsilon(newDataRegionBoundingBoxProjectedToMapLocal, -minimumRegionOverlapDistance);
            if (boundingBoxShadowsMapRegion)
            {
               addCornerPointsOfBoundingBoxIntersectionToMatches(mapBoundingBoxInMapLocal,
                                                                 newDataRegionBoundingBoxProjectedToMapLocal,
                                                                 newDataRegion,
                                                                 mapRegion.getTransformToLocal(),
                                                                 newDataRegion.getTransformToWorld(),
                                                                 shadowMatches);
            }
         }

         if (!shadowMatches.isEmpty())
         {
            mapToShadowMatchAndFitPoints.put(mapRegion, shadowMatches);
         }
      }
      return mapToShadowMatchAndFitPoints;
   }

   private static void addCornerPointsOfBoundingBoxIntersectionToMatches(BoundingBox2D mapBoundingBoxInMapLocal,
                                                                         BoundingBox2D newDataRegionBoundingBoxProjectedToMapLocal,
                                                                         PlanarRegion newDataRegion,
                                                                         RigidBodyTransformReadOnly transformFromWorldToMap,
                                                                         RigidBodyTransformReadOnly transformFromNewDataToWorld,
                                                                         PairList<PlanarRegion, Point2D> shadowMatches)
   {
      BoundingBox2D intersection = GeometryTools.getIntersectionOfTwoBoundingBoxes(mapBoundingBoxInMapLocal, newDataRegionBoundingBoxProjectedToMapLocal);

      if (intersection == null)
      {
         LogTools.error("Woops. Should never get here!!");
         LogTools.error("mapBoundingBoxInMapLocal = " + mapBoundingBoxInMapLocal);
         LogTools.error("newDataRegionBoundingBoxProjectedToMapLocal = " + newDataRegionBoundingBoxProjectedToMapLocal);
         return;
      }

      Point2DBasics centerPoint = new Point2D();
      Point2DBasics minPoint = new Point2D();
      Point2DBasics maxPoint = new Point2D();

      intersection.getCenterPoint(centerPoint);
      intersection.getMinPoint(minPoint);
      intersection.getMaxPoint(maxPoint);

      Point2D newDataReferencePointInNewDataLocal = createNewDataReferencePointInNewDataLocal(centerPoint,
                                                                                              transformFromWorldToMap,
                                                                                              transformFromNewDataToWorld);
      shadowMatches.add(newDataRegion, newDataReferencePointInNewDataLocal);

      newDataReferencePointInNewDataLocal = createNewDataReferencePointInNewDataLocal(minPoint, transformFromWorldToMap, transformFromNewDataToWorld);
      shadowMatches.add(newDataRegion, newDataReferencePointInNewDataLocal);

      newDataReferencePointInNewDataLocal = createNewDataReferencePointInNewDataLocal(maxPoint, transformFromWorldToMap, transformFromNewDataToWorld);
      shadowMatches.add(newDataRegion, newDataReferencePointInNewDataLocal);

      Point2D otherCorner = new Point2D(minPoint.getX(), maxPoint.getY());
      newDataReferencePointInNewDataLocal = createNewDataReferencePointInNewDataLocal(otherCorner,
                                                                                      transformFromWorldToMap,
                                                                                      transformFromNewDataToWorld);
      shadowMatches.add(newDataRegion, newDataReferencePointInNewDataLocal);

      otherCorner = new Point2D(maxPoint.getX(), minPoint.getY());
      newDataReferencePointInNewDataLocal = createNewDataReferencePointInNewDataLocal(otherCorner,
                                                                                      transformFromWorldToMap,
                                                                                      transformFromNewDataToWorld);
      shadowMatches.add(newDataRegion, newDataReferencePointInNewDataLocal);
      
   }

   private static BoundingBox2D computeNewDataRegionBoundingBoxProjectedToMapLocal(PlanarRegion newDataRegion,
                                                                                   RigidBodyTransformReadOnly transformFromWorldToMap,
                                                                                   RigidBodyTransformReadOnly transformFromNewDataToWorld)
   {
      BoundingBox2D newDataRegionBoundingBoxInLocal = PlanarRegionTools.getLocalBoundingBox2DInLocal(newDataRegion);
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
      return newDataRegionBoundingBoxProjectedToMapLocal;
   }

   private static Point2D createNewDataReferencePointInNewDataLocal(Point2DReadOnly pointInMapLocal,
                                                                    RigidBodyTransformReadOnly transformFromWorldToMap,
                                                                    RigidBodyTransformReadOnly transformFromNewDataToWorld)
   {
      Point3D newDataReferencePoint = new Point3D(pointInMapLocal);
      newDataReferencePoint.applyInverseTransform(transformFromWorldToMap);
      newDataReferencePoint.applyInverseTransform(transformFromNewDataToWorld);

      return new Point2D(newDataReferencePoint);
   }

   public static Map<PlanarRegion, List<PlanarRegion>> detectLocalBoundingBox3DCollisions(PlanarRegionsList map, PlanarRegionsList newData,
                                                                                          double boundingBoxHeight)
   {
      HashMap<PlanarRegion, List<PlanarRegion>> newDataCollisions = new HashMap<>();
      for (PlanarRegion planarRegion : map.getPlanarRegionsAsList())
      {
         ArrayList<PlanarRegion> localCollisions = new ArrayList<>();

         for (PlanarRegion newRegion : newData.getPlanarRegionsAsList())
         {
            if (boxesIn3DIntersect(planarRegion, newRegion, boundingBoxHeight))
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
      return a.getBoundingBox3dInWorld().intersectsEpsilon(b.getBoundingBox3dInWorld(), 1e-8);
   }
}
