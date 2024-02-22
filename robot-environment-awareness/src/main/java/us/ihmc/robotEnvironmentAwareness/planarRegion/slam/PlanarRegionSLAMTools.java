package us.ihmc.robotEnvironmentAwareness.planarRegion.slam;

import java.util.*;

import gnu.trove.list.array.TIntArrayList;
import org.apache.commons.lang3.tuple.ImmutablePair;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.linsol.svd.SolvePseudoInverseSvd_DDRM;
import org.ejml.interfaces.decomposition.SingularValueDecomposition_F64;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.shape.collision.EuclidShape3DCollisionResult;
import us.ihmc.euclid.shape.collision.gjk.GilbertJohnsonKeerthiCollisionDetector;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.log.LogTools;
import us.ihmc.robotEnvironmentAwareness.tools.ConcaveHullMerger;
import us.ihmc.robotics.geometry.*;
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
                                                                 PlanarRegionSLAMParameters parameters,
                                                                 RigidBodyTransform referenceTransform)
   {
      RigidBodyTransform bigT = new RigidBodyTransform();

      SolvePseudoInverseSvd_DDRM solver = new SolvePseudoInverseSvd_DDRM();

      int numberOfMatches = 0;
      for (PairList<PlanarRegion, Point2D> newDataRegionWithReferencePoints : matchesWithReferencePoints.values())
      {
         numberOfMatches += newDataRegionWithReferencePoints.size();
      }

      if (numberOfMatches == 0)
         return new RigidBodyTransform();

      DMatrixRMaj A = new DMatrixRMaj(numberOfMatches, 6);
      DMatrixRMaj b = new DMatrixRMaj(numberOfMatches, 1); // negative distance to planes

      int i = 0;
      for (PlanarRegion mapRegion : matchesWithReferencePoints.keySet())
      {
         Plane3D planarRegionPlane3D = mapRegion.getPlane();
         Vector3DReadOnly normal = planarRegionPlane3D.getNormal();
         Vector3D normalInReferenceFrame = null;

         if (referenceTransform != null)
         {
            normalInReferenceFrame = new Vector3D(normal);
            referenceTransform.inverseTransform(normalInReferenceFrame);
         }

         for (ImmutablePair<PlanarRegion, Point2D> newDataRegionWithReferencePoint : matchesWithReferencePoints.get(mapRegion))
         {
            PlanarRegion newPlanarRegion = newDataRegionWithReferencePoint.getLeft();
            Point2D referencePointInNewDataLocal = newDataRegionWithReferencePoint.getRight();

            Point3D referencePointInWorld = new Point3D(referencePointInNewDataLocal);
            RigidBodyTransform transformFromNewDataToWorld = new RigidBodyTransform();
            newPlanarRegion.getTransformToWorld(transformFromNewDataToWorld);
            transformFromNewDataToWorld.transform(referencePointInWorld);

            Vector3D cross = new Vector3D();

            if (referenceTransform != null)
            {
               Point3D referencePointInReferenceFrame = new Point3D(referencePointInWorld);
               referenceTransform.inverseTransform(referencePointInReferenceFrame);
               cross.cross(referencePointInReferenceFrame, normalInReferenceFrame);
            }
            else
            {
               cross.cross(referencePointInWorld, normal);
            }

            A.set(i, 0, cross.getX());
            A.set(i, 1, cross.getY());
            A.set(i, 2, cross.getZ());

            if (referenceTransform != null)
            {
               A.set(i, 3, normalInReferenceFrame.getX());
               A.set(i, 4, normalInReferenceFrame.getY());
               A.set(i, 5, normalInReferenceFrame.getZ());
            }
            else
            {
               A.set(i, 3, normal.getX());
               A.set(i, 4, normal.getY());
               A.set(i, 5, normal.getZ());
            }

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

      DMatrixRMaj x = new DMatrixRMaj(6, 1);

      DMatrixRMaj ATransposeTimesA = new DMatrixRMaj(6, 6);
      CommonOps_DDRM.multInner(A, ATransposeTimesA);

      DMatrixRMaj ATransposeB = new DMatrixRMaj(6, 1);
      CommonOps_DDRM.multTransA(A, b, ATransposeB);

      // Use damped least squares (also called regularized least squares) to prevent blow up when data is sparse.
      // See https://www2.math.uconn.edu/~leykekhman/courses/MARN_5898/Lectures/Linear_least_squares_reg.pdf
      DMatrixRMaj lambdaI = CommonOps_DDRM.identity(6);
      CommonOps_DDRM.scale(parameters.getDampedLeastSquaresLambda(), lambdaI);

      DMatrixRMaj ATransposeTimesAPlusLambdaI = new DMatrixRMaj(6, 6);
      CommonOps_DDRM.add(ATransposeTimesA, lambdaI, ATransposeTimesAPlusLambdaI);

      solver.setA(ATransposeTimesAPlusLambdaI);

      solver.solve(ATransposeB, x);

      if (verbose)
      {
         SingularValueDecomposition_F64<DMatrixRMaj> decomposition = solver.getDecomposition();
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

      if (referenceTransform != null)
      {
         RigidBodyTransform transformToReturn = new RigidBodyTransform(bigT);
         transformToReturn.preMultiply(referenceTransform);
         transformToReturn.multiplyInvertOther(referenceTransform);
         return transformToReturn;
      }
      else
      {
         return bigT;
      }
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

   public static class IncomingRegionMatchData
   {
      private final int incomingRegionId;
      private final List<MatchData> goodMatches = new ArrayList<>();
      private final List<MatchData> badMatches = new ArrayList<>();

      public IncomingRegionMatchData(int incomingRegionId)
      {
         this.incomingRegionId = incomingRegionId;
      }

      public void reset()
      {
         goodMatches.clear();
         badMatches.clear();
      }

      public int getIncomingRegionId()
      {
         return incomingRegionId;
      }

      public void addMatchData(MatchData matchData)
      {
         if (matchData.getMatchScore() > 50.0)
            goodMatches.add(matchData);
         else
            badMatches.add(matchData);
      }

      public int getNumberOfGoodMatches()
      {
         return goodMatches.size();
      }

      public int getNumberOfBadMatches()
      {
         return badMatches.size();
      }

      public int getNumberOfMatches()
      {
         return getNumberOfGoodMatches() + getNumberOfBadMatches();
      }

      public MatchData getGoodMatch(int goodMatchNumber)
      {
         return goodMatches.get(goodMatchNumber);
      }

      public MatchData getBadMatch(int badMatchNumber)
      {
         return badMatches.get(badMatchNumber);
      }
   }

   public static class MatchData
   {
      private final int matchingMapRegionId;

      public MatchData(int matchingMapRegionId)
      {
         this.matchingMapRegionId = matchingMapRegionId;
      }

      public int getMatchingMapRegionId()
      {
         return matchingMapRegionId;
      }

      public double getMatchScore()
      {
         return 100.0;
      }
   }

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
    * Filters the match set down further. For each match so far, find newData regions that "shadow" the
    * corresponding map region. By "shadow" we mean as if shining an orthogonal light source at the map
    * region and seeing the shadow cast upon it by a newData region. Shadow must overlap more than
    * minimumRegionOverlapDistance. Then, for each shadow match, add several reference points, like the
    * vertices of the shadow. These points are the "fit" points that are put into the fit optimization
    * algorithm.
    *
    * @param minimumRegionOverlapDistance   The minimum amount that two regions must overlap in order
    *                                       to consider them part of the same region.
    * @param maximumPointProjectionDistance The maximum distance that a point will be projected from
    *                                       one region to the other when slam happens.
    * @param matchesSoFar
    * @return A map from a map PlanarRegion to a PairList of new data PlanarRegions and Point2Ds that
    *       are points on the new region that we would like to move towards the plane of the map
    *       region.
    */
   public static Map<PlanarRegion, PairList<PlanarRegion, Point2D>> filterMatchesBasedOn2DBoundingBoxShadow(double minimumRegionOverlapDistance,
                                                                                                            double maximumPointProjectionDistance,
                                                                                                            Map<PlanarRegion, List<PlanarRegion>> matchesSoFar)
   {
      HashMap<PlanarRegion, PairList<PlanarRegion, Point2D>> mapToShadowMatchAndFitPoints = new HashMap<>();
      for (PlanarRegion mapRegion : matchesSoFar.keySet())
      {
         PairList<PlanarRegion, Point2D> shadowMatches = new PairList<>();

         for (PlanarRegion newDataRegion : matchesSoFar.get(mapRegion))
         {
            BoundingBox2D newDataRegionBoundingBoxProjectedToMapLocal = computeNewDataRegionBoundingBoxProjectedToMapLocal(newDataRegion,
                                                                                                                           mapRegion.getTransformToLocal(),
                                                                                                                           newDataRegion.getTransformToWorld());

            BoundingBox2D mapBoundingBoxInMapLocal = PlanarRegionTools.getLocalBoundingBox2DInLocal(mapRegion);

            boolean boundingBoxShadowsMapRegion = mapBoundingBoxInMapLocal.intersectsEpsilon(newDataRegionBoundingBoxProjectedToMapLocal,
                                                                                             -minimumRegionOverlapDistance);
            if (boundingBoxShadowsMapRegion)
            {
               addCornerPointsOfBoundingBoxIntersectionToMatches(mapBoundingBoxInMapLocal,
                                                                 newDataRegionBoundingBoxProjectedToMapLocal,
                                                                 newDataRegion,
                                                                 mapRegion.getTransformToLocal(),
                                                                 newDataRegion.getTransformToWorld(),
                                                                 maximumPointProjectionDistance,
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
                                                                         double maximumPointProjectionDistance,
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
      minPoint.set(intersection.getMinPoint());
      maxPoint.set(intersection.getMaxPoint());

      Point2D newCenterPointInNewDataLocal = createNewDataReferencePointInNewDataLocal(centerPoint,
                                                                                       transformFromWorldToMap,
                                                                                       transformFromNewDataToWorld,
                                                                                       maximumPointProjectionDistance);

      Point2D newMinimumPointInNewDataLocal = createNewDataReferencePointInNewDataLocal(minPoint,
                                                                                        transformFromWorldToMap,
                                                                                        transformFromNewDataToWorld,
                                                                                        maximumPointProjectionDistance);

      Point2D newMaximumPointInNewDataLocal = createNewDataReferencePointInNewDataLocal(maxPoint,
                                                                                        transformFromWorldToMap,
                                                                                        transformFromNewDataToWorld,
                                                                                        maximumPointProjectionDistance);

      Point2D otherCorner = new Point2D(minPoint.getX(), maxPoint.getY());
      Point2D newDataOtherCornerPointInNewDataLocal = createNewDataReferencePointInNewDataLocal(otherCorner,
                                                                                                transformFromWorldToMap,
                                                                                                transformFromNewDataToWorld,
                                                                                                maximumPointProjectionDistance);

      Point2D finalCorner = new Point2D(maxPoint.getX(), minPoint.getY());
      Point2D newFinalCornerPointInNewDataLocal = createNewDataReferencePointInNewDataLocal(finalCorner,
                                                                                            transformFromWorldToMap,
                                                                                            transformFromNewDataToWorld,
                                                                                            maximumPointProjectionDistance);

      addAllIfAllAreNotNull(shadowMatches,
                            newDataRegion,
                            newCenterPointInNewDataLocal,
                            newMinimumPointInNewDataLocal,
                            newMaximumPointInNewDataLocal,
                            newDataOtherCornerPointInNewDataLocal,
                            newFinalCornerPointInNewDataLocal);
   }

   private static void addAllIfAllAreNotNull(PairList<PlanarRegion, Point2D> shadowMatches, PlanarRegion planarRegion, Point2D... pointsToAdd)
   {
      boolean areAnyNull = areAnyNull(pointsToAdd);

      if (areAnyNull)
      {
         return;
      }

      for (Point2D point : pointsToAdd)
      {
         shadowMatches.add(planarRegion, point);
      }
   }

   private static boolean areAnyNull(Point2D[] points)
   {
      for (Point2D point : points)
      {
         if (point == null)
            return true;
      }
      return false;
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
                                                                    RigidBodyTransformReadOnly transformFromNewDataToWorld,
                                                                    double maximumPointProjectionDistance)
   {
      Point3D newDataReferencePoint = new Point3D(pointInMapLocal);
      newDataReferencePoint.applyInverseTransform(transformFromWorldToMap);
      newDataReferencePoint.applyInverseTransform(transformFromNewDataToWorld);

      double distanceProjectedToLocal = Math.abs(newDataReferencePoint.getZ());
      if (distanceProjectedToLocal > maximumPointProjectionDistance)
         return null;

      return new Point2D(newDataReferencePoint);
   }

   public static Map<PlanarRegion, List<PlanarRegion>> detectLocalBoundingBox3DCollisions(PlanarRegionsList map,
                                                                                          PlanarRegionsList newData,
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

   public static void updateParentPlaneUsingComplementaryFilter(PlanarRegion parentRegion, PlanarRegion childRegion, double updateTowardsChildAlpha)
   {
      // Update Map Region Normal and Origin
      UnitVector3DReadOnly mapNormal = parentRegion.getNormal();
      Point3DReadOnly mapOrigin = parentRegion.getPoint();

      UnitVector3DReadOnly regionNormal = childRegion.getNormal();
      Point3DReadOnly regionOrigin = childRegion.getPoint();

      Vector3D futureNormal = new Vector3D();
      futureNormal.interpolate(mapNormal, regionNormal, updateTowardsChildAlpha);

      double futureHeightZ = EuclidCoreTools.interpolate(mapOrigin.getZ(), regionOrigin.getZ(), updateTowardsChildAlpha);

      Vector3D normalVector = new Vector3D(mapNormal);
      Vector3D axis = new Vector3D();
      axis.cross(normalVector, futureNormal);
      double angle = normalVector.angle(futureNormal);

      Point3D futureOrigin = new Point3D(mapOrigin.getX(), mapOrigin.getY(), futureHeightZ);
      AxisAngle rotationToFutureRegion = new AxisAngle(axis, angle);
      Vector3D translationToFutureRegion = new Vector3D();
      translationToFutureRegion.sub(futureOrigin, mapOrigin);

      RigidBodyTransform transform = new RigidBodyTransform(rotationToFutureRegion, translationToFutureRegion);
      transform.normalizeRotationPart();

      parentRegion.applyTransform(transform);
   }

   public static boolean mergeRegionIntoParentUsingFilter(PlanarRegion parentRegion, PlanarRegion childRegion, double updateTowardsChildAlpha)
   {
      updateParentPlaneUsingComplementaryFilter(parentRegion, childRegion, updateTowardsChildAlpha);
      return mergeRegionHulls(parentRegion, childRegion);
   }

   public static boolean mergeRegionIntoParentUsingOptimalPlane(PlanarRegion parentRegion, PlanarRegion childRegion, Vector4D optimalPlane)
   {
      // TODO: Fix this to use the optimal plane from factor graph optimized results
      // Jira: https://jira.ihmc.us/browse/HS-434

      //      PlanarRegionTools.projectInZToPlanarRegion()
      //      (parentRegion, childRegion, optimalPlane);
      return mergeRegionHulls(parentRegion, childRegion);
   }

   public static boolean mergeRegionHulls(PlanarRegion parentRegion, PlanarRegion childRegion)
   {
      ArrayList<PlanarRegion> mergedRegion = ConcaveHullMerger.mergePlanarRegions(parentRegion, childRegion, 1.0f, null);

      if (mergedRegion != null)
      {
         if (mergedRegion.size() > 0)
         {
            parentRegion.set(mergedRegion.get(0));
            return true;
         }
         else
         {
            // check if the child region is fully contained by the parent region
            if (parentRegion.getConvexHull().isPointInside(childRegion.getConvexHull().getVertex(0)))
            {
               return true;
            }
            // check if the parent region is fully contained by the child region
            else if (childRegion.getConvexHull().isPointInside(parentRegion.getConvexHull().getVertex(0)))
            {
               parentRegion.set(childRegion);
               return true;
            }

            return false;
         }
      }
      return false;
   }

   private static void updateRegionPlaneTowardsReference(PlanarRegion regionToModify, PlanarRegion regionToRefer, double updateTowardsSecondAlpha)
   {
      // Update Map Region Normal and Origin
      UnitVector3DReadOnly firstNormal = regionToModify.getNormal();
      Point3DReadOnly firstOrigin = regionToModify.getPoint();

      UnitVector3DReadOnly secondNormal = regionToRefer.getNormal();
      Point3DReadOnly secondOrigin = regionToRefer.getPoint();

      Vector3D futureNormal = new Vector3D();
      futureNormal.interpolate(firstNormal, secondNormal, updateTowardsSecondAlpha);

      double futureHeightZ = EuclidCoreTools.interpolate(firstOrigin.getZ(), secondOrigin.getZ(), updateTowardsSecondAlpha);

      Vector3D normalVector = new Vector3D(firstNormal);
      Vector3D axis = new Vector3D();
      axis.cross(normalVector, futureNormal);
      double angle = normalVector.angle(futureNormal);

      Point3D futureOrigin = new Point3D(firstOrigin.getX(), firstOrigin.getY(), futureHeightZ);
      AxisAngle rotationToFutureRegion = new AxisAngle(axis, angle);
      Vector3D translationToFutureRegion = new Vector3D();
      translationToFutureRegion.sub(futureOrigin, firstOrigin);

      RigidBodyTransform transform = new RigidBodyTransform();
      transform.appendOrientation(rotationToFutureRegion);
      transform.appendTranslation(translationToFutureRegion);

      regionToModify.applyTransform(transform);
   }
}
