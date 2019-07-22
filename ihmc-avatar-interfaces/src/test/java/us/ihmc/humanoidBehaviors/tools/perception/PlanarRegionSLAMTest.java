package us.ihmc.humanoidBehaviors.tools.perception;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.Set;
import java.util.concurrent.CountDownLatch;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.junit.jupiter.api.Test;

import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidBehaviors.ui.graphics.PlanarRegionsGraphic;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.tools.lists.PairList;

class PlanarRegionSLAMTest
{
   private static boolean visualize = false;

   //test commit.
   @Test
   public void testSLAMWithThreeNiceWalls()
   {
      PlanarRegionsList map;
      PlanarRegionsList newData;

      PlanarRegionSLAMResult slamResult;
      PlanarRegionsList mergedMap;
      RigidBodyTransform transformResult;

      // No transform. Exactly the same walls.
      map = createSomeRightAngledWalls(false, new RigidBodyTransform(), true, true, true);
      newData = createSomeRightAngledWalls(false, new RigidBodyTransform(), true, true, true);

      PlanarRegionSLAMParameters parameters = new PlanarRegionSLAMParameters();
      parameters.setBoundingBoxHeight(0.1);
      parameters.setDampedLeastSquaresLambda(0.0);
      parameters.setIterations(1);

      slamResult = PlanarRegionSLAM.slam(map, newData, parameters);
      mergedMap = slamResult.getMergedMap();
      transformResult = slamResult.getTransformFromIncomingToMap();

      assertTrue(transformResult.epsilonEquals(new RigidBodyTransform(), 1e-7));
      //      assertEquals(3, mergedMap.getNumberOfPlanarRegions()); //TODO: Fix

      
//      visualizePlanarRegions(map);
//      visualizePlanarRegions(mergedMap);
//      ThreadTools.sleepForever();
      
      // Small translation transform with all six walls.
      RigidBodyTransform smallTranslationTransform = new RigidBodyTransform();
      double delta = 0.05;
      smallTranslationTransform.setTranslation(delta * 0.5, delta * 1.3, -delta * 0.6);

      map = createSomeRightAngledWalls(true, new RigidBodyTransform(), true, true, true);
      newData = createSomeRightAngledWalls(true, smallTranslationTransform, true, true, true);

      Map<PlanarRegion, PairList<PlanarRegion, Point2D>> matchesAndReferencePoints = PlanarRegionSLAM.findHighConfidenceRegionMatchesAndReferencePoints(map,
                                                                                                                                                        newData,
                                                                                                                                                        parameters);
      assertEquals(6, matchesAndReferencePoints.size());
      Set<PlanarRegion> keySet = matchesAndReferencePoints.keySet();
      for (PlanarRegion key : keySet)
      {
         PairList<PlanarRegion, Point2D> pairList = matchesAndReferencePoints.get(key);
         //         assertEquals(3, pairList.size());//TODO: Fix

         ImmutablePair<PlanarRegion, Point2D> immutablePair = pairList.get(0);
         PlanarRegion planarRegionMatch = immutablePair.getLeft();
         Point2D referencePoint = immutablePair.getRight();
      }

      parameters.setIterations(1);
      parameters.setBoundingBoxHeight(0.1);

      slamResult = PlanarRegionSLAM.slam(map, newData, parameters);
      mergedMap = slamResult.getMergedMap();
      transformResult = slamResult.getTransformFromIncomingToMap();

      assertTransformsAreInverses(transformResult, smallTranslationTransform, 1e-7);
      //      assertEquals(3, mergedMap.getNumberOfPlanarRegions()); //TODO: Fix

      // Small rotation transform with all six walls.
      RigidBodyTransform smallRotationTransform = new RigidBodyTransform();

      smallRotationTransform.setRotationYawPitchRoll(delta * 0.25, delta * 1.13, -delta * 0.7);
      map = createSomeRightAngledWalls(true, new RigidBodyTransform(), true, true, true);
      newData = createSomeRightAngledWalls(true, smallRotationTransform, true, true, true);

      parameters.setIterations(2);
      slamResult = PlanarRegionSLAM.slam(map, newData, parameters);
      mergedMap = slamResult.getMergedMap();
      transformResult = slamResult.getTransformFromIncomingToMap();

      assertTransformsAreInverses(transformResult, smallRotationTransform, 1e-5);
      //      assertEquals(3, mergedMap.getNumberOfPlanarRegions()); //TODO: Fix

      // Small translation and rotation transform with all six walls.
      RigidBodyTransform smallRotationAndTranslationTransform = new RigidBodyTransform();

      smallRotationAndTranslationTransform.setTranslation(delta * 0.17, -delta * 0.33, delta * 0.117);
      smallRotationAndTranslationTransform.setRotationYawPitchRoll(delta * 0.25, delta * 1.13, -delta * 0.7);
      map = createSomeRightAngledWalls(true, new RigidBodyTransform(), true, true, true);
      newData = createSomeRightAngledWalls(true, smallRotationAndTranslationTransform, true, true, true);

      slamResult = PlanarRegionSLAM.slam(map, newData, parameters);
      mergedMap = slamResult.getMergedMap();
      transformResult = slamResult.getTransformFromIncomingToMap();

      assertTransformsAreInverses(transformResult, smallRotationAndTranslationTransform, 1e-5);
      //      assertEquals(3, mergedMap.getNumberOfPlanarRegions()); //TODO: Fix

      // With only two walls, will only get two translations and two rotations. Use no floor here.
      map = createSomeRightAngledWalls(true, new RigidBodyTransform(), false, true, true);
      newData = createSomeRightAngledWalls(true, smallTranslationTransform, false, true, true);

      slamResult = PlanarRegionSLAM.slam(map, newData, parameters);
      mergedMap = slamResult.getMergedMap();
      transformResult = slamResult.getTransformFromIncomingToMap();

      Vector3D rotationResult = new Vector3D();
      Vector3D translationResult = new Vector3D();

      transformResult.get(rotationResult, translationResult);
      assertTrue(rotationResult.epsilonEquals(new Vector3D(), 1e-7));
      assertTrue(translationResult.epsilonEquals(new Vector3D(-smallTranslationTransform.getTranslationX(), -smallTranslationTransform.getTranslationY(), 0.0),
                                                 1e-7));
      //      assertEquals(2, mergedMap.getNumberOfPlanarRegions()); //TODO: Fix

      // Use only floor here. The other regions each have one wall.
      map = createSomeRightAngledWalls(true, new RigidBodyTransform(), true, true, false);
      newData = createSomeRightAngledWalls(true, smallTranslationTransform, true, false, true);

      slamResult = PlanarRegionSLAM.slam(map, newData, parameters);
      mergedMap = slamResult.getMergedMap();
      transformResult = slamResult.getTransformFromIncomingToMap();

      transformResult.get(rotationResult, translationResult);
      assertTrue(rotationResult.epsilonEquals(new Vector3D(), 1e-7));
      assertTrue(translationResult.epsilonEquals(new Vector3D(0.0, 0.0, -smallTranslationTransform.getTranslationZ()), 1e-7));
      //      assertEquals(2, mergedMap.getNumberOfPlanarRegions()); //TODO: Fix

      // The floor alone should not give yaw.
      RigidBodyTransform smallYawTransform = new RigidBodyTransform();
      smallYawTransform.setRotationYaw(delta);

      map = createSomeRightAngledWalls(true, new RigidBodyTransform(), true, false, false);
      newData = createSomeRightAngledWalls(true, smallYawTransform, true, false, false);

      slamResult = PlanarRegionSLAM.slam(map, newData, parameters);
      mergedMap = slamResult.getMergedMap();
      transformResult = slamResult.getTransformFromIncomingToMap();

      assertTrue(transformResult.epsilonEquals(new RigidBodyTransform(), 1e-7));
      //      assertEquals(1, mergedMap.getNumberOfPlanarRegions()); //TODO: Fix

      // The floor alone should give pitch and roll, but might have some yaw in it.
      RigidBodyTransform smallPitchAndRollTransforms = new RigidBodyTransform();
      smallPitchAndRollTransforms.setRotationYawPitchRoll(0.0, delta * 1.3, -delta * 0.22);

      map = createSomeRightAngledWalls(true, new RigidBodyTransform(), true, false, false);
      newData = createSomeRightAngledWalls(true, smallPitchAndRollTransforms, true, false, false);

      slamResult = PlanarRegionSLAM.slam(map, newData, parameters);
      mergedMap = slamResult.getMergedMap();
      transformResult = slamResult.getTransformFromIncomingToMap();

      assertTransformsAreInverses(transformResult, smallPitchAndRollTransforms, 1e-3);
      //      assertEquals(1, mergedMap.getNumberOfPlanarRegions()); //TODO: Fix

      // Two walls should give yaw, pitch, and roll
      map = createSomeRightAngledWalls(true, new RigidBodyTransform(), true, true, false);
      newData = createSomeRightAngledWalls(true, smallRotationTransform, true, true, false);

      slamResult = PlanarRegionSLAM.slam(map, newData, parameters);
      mergedMap = slamResult.getMergedMap();
      transformResult = slamResult.getTransformFromIncomingToMap();

      assertTransformsAreInverses(transformResult, smallRotationTransform, 1e-5);
      //      assertEquals(2, mergedMap.getNumberOfPlanarRegions()); //TODO: Fix
   }

   private void assertTransformsAreInverses(RigidBodyTransform expectedTransform, RigidBodyTransform transform, double epsilon)
   {
      RigidBodyTransform inverseTransform = new RigidBodyTransform(transform);
      inverseTransform.invert();

      assertTrue(inverseTransform.epsilonEquals(expectedTransform, epsilon),
                 "\ntransform = " + inverseTransform + ", \nexpectedTransform = " + expectedTransform);
   }

   @Test
   public void testSLAMWithRandomPolygonsAndSmallExactTransforms()
   {
      PlanarRegionsList map = new PlanarRegionsList();
      PlanarRegionsList newData = new PlanarRegionsList();

      PlanarRegionsList perfectCombinedMap = new PlanarRegionsList();

      Random random = new Random(1776L);

      double maxAbsoluteXY = 1.0;

      int numberOfPolygonPlanarRegions = 100;

      int numberInMap = 0;
      int numberInNewData = 0;
      int numberInBoth = 0;

      RigidBodyTransform smallTransform = new RigidBodyTransform();
      smallTransform.setTranslation(new Vector3D(0.011, -0.0137, 0.012));
      smallTransform.setRotation(new Vector3D(0.002, -0.03, -0.19));

      for (int i = 0; i < numberOfPolygonPlanarRegions; i++)
      {
         int numberOfPossiblePoints = 3 + random.nextInt(8);

         ConvexPolygon2D convexPolygon = EuclidGeometryRandomTools.nextConvexPolygon2D(random, maxAbsoluteXY, numberOfPossiblePoints);
         Vector3D minMaxTranslation = new Vector3D(10.0, 10.0, 10.0);

         Vector3D translation = EuclidCoreRandomTools.nextVector3D(random, minMaxTranslation);
         RotationMatrix rotation = EuclidCoreRandomTools.nextRotationMatrix(random);
         RigidBodyTransform transform = new RigidBodyTransform(rotation, translation);

         boolean addToNewData = random.nextBoolean();
         boolean addToMap = !addToNewData || random.nextBoolean();

         PlanarRegion combinedPlanarRegion = new PlanarRegion(transform, convexPolygon);
         combinedPlanarRegion.setRegionId(random.nextInt(1000000));

         if (addToMap)
         {
            PlanarRegion planarRegion = new PlanarRegion(transform, convexPolygon);
            planarRegion.setRegionId(random.nextInt(1000000));
            map.addPlanarRegion(planarRegion);
            numberInMap++;
         }
         if (addToNewData)
         {
            RigidBodyTransform newTransform = new RigidBodyTransform(transform);
            newTransform.preMultiply(smallTransform);

            PlanarRegion planarRegion = new PlanarRegion(newTransform, convexPolygon);
            planarRegion.setRegionId(random.nextInt(1000000));

            newData.addPlanarRegion(planarRegion);
            numberInNewData++;
         }

         perfectCombinedMap.addPlanarRegion(combinedPlanarRegion);

         if (addToMap && addToNewData)
         {
            numberInBoth++;
         }
      }

      PlanarRegionSLAMParameters parameters = new PlanarRegionSLAMParameters();
      parameters.setIterations(3);
      parameters.setBoundingBoxHeight(0.1);
      parameters.setDampedLeastSquaresLambda(0.0);

      PlanarRegionSLAMResult slamResult = PlanarRegionSLAM.slam(map, newData, parameters);
      PlanarRegionsList mergedMap = slamResult.getMergedMap();
      RigidBodyTransform transformResult = slamResult.getTransformFromIncomingToMap();

      assertTransformsAreInverses(transformResult, smallTransform, 1e-5);
      assertEquals(numberInMap + numberInNewData - numberInBoth, perfectCombinedMap.getNumberOfPlanarRegions());
      //      assertEquals(perfectCombinedMap.getNumberOfPlanarRegions(), mergedMap.getNumberOfPlanarRegions());

      if (visualize)
      {
         visualizePlanarRegions(mergedMap);
         ThreadTools.sleepForever();
      }
      assertPlanarRegionsListAreEquivalentThroughPointProjections(random, perfectCombinedMap, mergedMap);
   }

   private void assertPlanarRegionsListAreEquivalentThroughPointProjections(Random random, PlanarRegionsList regionsOne, PlanarRegionsList regionsTwo)
   {
      BoundingBox3D boundingBoxOne = computePlanarRegionsListBoundingBox3D(regionsOne);
      BoundingBox3D boundingBoxTwo = computePlanarRegionsListBoundingBox3D(regionsTwo);

      double epsilon = 0.003;

      assertTrue(boundingBoxOne.epsilonEquals(boundingBoxTwo, epsilon), "boundingBoxOne = " + boundingBoxOne + "boundingBoxTwo = " + boundingBoxTwo);

      int numberOfPointsToTest = 1000;

      for (int i = 0; i < numberOfPointsToTest; i++)
      {
         Point3D randomPoint = randomPointInsideBoundingBox(random, boundingBoxOne);
         Point3D projectionOne = PlanarRegionTools.projectPointToPlanes(randomPoint, regionsOne);
         Point3D projectionTwo = PlanarRegionTools.projectPointToPlanes(randomPoint, regionsTwo);
         assertTrue(projectionOne.epsilonEquals(projectionTwo, epsilon));
      }
   }

   private Point3D randomPointInsideBoundingBox(Random random, BoundingBox3D boundingBox)
   {
      return EuclidCoreRandomTools.nextPoint3D(random, boundingBox.getMinX(), boundingBox.getMaxX(), boundingBox.getMinY(), boundingBox.getMaxY(),
                                               boundingBox.getMinZ(), boundingBox.getMaxZ());
   }

   private static BoundingBox3D computePlanarRegionsListBoundingBox3D(PlanarRegionsList planarRegionsList)
   {
      BoundingBox3D combinedBoundingBox = null;

      List<PlanarRegion> planarRegions = planarRegionsList.getPlanarRegionsAsList();

      for (PlanarRegion planarRegion : planarRegions)
      {
         BoundingBox3D boundingBox = planarRegion.getBoundingBox3dInWorldCopy();
         if (combinedBoundingBox == null)
         {
            combinedBoundingBox = boundingBox;
         }
         else
         {
            combinedBoundingBox = BoundingBox3D.union(combinedBoundingBox, boundingBox);
         }
      }

      return combinedBoundingBox;
   }

   private PlanarRegionsList createSomeRightAngledWalls(boolean includeOppositeSide, RigidBodyTransform transform, boolean includeFloor, boolean includeWallOne,
                                                        boolean includeWallTwo)
   {
      PlanarRegionsList planarRegionsList = new PlanarRegionsList();

      if (includeFloor)
      {
         PlanarRegion floor = createASingleSquare(new Vector3D(), 0.0, 0.0, 0.0, new Point2D(0.0, 0.0), new Point2D(1.0, 1.0));

         floor.transformByPreMultiply(transform);
         floor.setRegionId(1);
         planarRegionsList.addPlanarRegion(floor);

         if (includeOppositeSide)
         {
            PlanarRegion ceiling = createASingleSquare(new Vector3D(), 0.0, 0.0, 0.0, new Point2D(0.0, 0.0), new Point2D(1.0, 1.0));

            RigidBodyTransform floorToCeiling = new RigidBodyTransform();
            floorToCeiling.setTranslation(0.0, 0.0, 1.0);
            ceiling.transformByPreMultiply(floorToCeiling);
            ceiling.transformByPreMultiply(transform);

            ceiling.setRegionId(4);
            planarRegionsList.addPlanarRegion(ceiling);
         }
      }

      if (includeWallOne)
      {
         PlanarRegion wallOne = createASingleSquare(new Vector3D(), 0.0, -Math.PI / 2.0, 0.0, new Point2D(0.0, 0.0), new Point2D(1.0, 1.0));

         wallOne.transformByPreMultiply(transform);
         wallOne.setRegionId(2);
         planarRegionsList.addPlanarRegion(wallOne);

         if (includeOppositeSide)
         {
            PlanarRegion oppositeWallOne = createASingleSquare(new Vector3D(), 0.0, -Math.PI / 2.0, 0.0, new Point2D(0.0, 0.0), new Point2D(1.0, 1.0));

            RigidBodyTransform wallToWall = new RigidBodyTransform();
            wallToWall.setTranslation(1.0, 0.0, 0.0);
            oppositeWallOne.transformByPreMultiply(wallToWall);
            oppositeWallOne.transformByPreMultiply(transform);

            oppositeWallOne.setRegionId(5);
            planarRegionsList.addPlanarRegion(oppositeWallOne);
         }
      }

      if (includeWallTwo)
      {
         PlanarRegion wallTwo = createASingleSquare(new Vector3D(), 0.0, 0.0, Math.PI / 2.0, new Point2D(0.0, 0.0), new Point2D(1.0, 1.0));

         wallTwo.transformByPreMultiply(transform);
         wallTwo.setRegionId(3);
         planarRegionsList.addPlanarRegion(wallTwo);
         if (includeOppositeSide)
         {
            PlanarRegion oppositeWallTwo = createASingleSquare(new Vector3D(), 0.0, 0.0, Math.PI / 2.0, new Point2D(0.0, 0.0), new Point2D(1.0, 1.0));

            RigidBodyTransform wallToWall = new RigidBodyTransform();
            wallToWall.setTranslation(0.0, 1.0, 0.0);
            oppositeWallTwo.transformByPreMultiply(wallToWall);
            oppositeWallTwo.transformByPreMultiply(transform);

            oppositeWallTwo.setRegionId(5);
            planarRegionsList.addPlanarRegion(oppositeWallTwo);
         }
      }

      return planarRegionsList;
   }

   @Test
   public void testFindHighConfidencePairsWithSomeSimpleSquares()
   {
      // Two identical squares.
      PlanarRegion squareOne = createASingleSquareCenteredAtOrigin(1.0, 1.0);
      PlanarRegion squareTwo = createASingleSquareCenteredAtOrigin(1.0, 1.0);
      assertHighConfidencePairingBothWays(squareOne, squareTwo);

      // One small square inside a larger square.
      squareOne = createASingleSquareCenteredAtOrigin(1.0, 1.0);
      squareTwo = createASingleSquareCenteredAtOrigin(0.5, 0.5);
      assertHighConfidencePairingBothWays(squareOne, squareTwo);

      Vector3D translationOne = new Vector3D(10.1, -20.3, 7.6);

      // One large, one small translated same amounts.
      squareOne = createASingleTranslatedSquare(translationOne, 1.0, 1.0);
      squareTwo = createASingleTranslatedSquare(translationOne, 0.5, 0.5);
      assertHighConfidencePairingBothWays(squareOne, squareTwo);

      // One large, one small, but translated a significant amount in z, so should not be a pair.
      Vector3D largeZOffset = new Vector3D(0.0, 0.0, 0.2);
      Vector3D translationTwo = new Vector3D(translationOne);
      translationTwo.add(largeZOffset);
      squareOne = createASingleTranslatedSquare(translationOne, 1.0, 1.0);
      squareTwo = createASingleTranslatedSquare(translationTwo, 0.5, 0.5);
      assertNotAHighConfidencePairing(squareOne, squareTwo);

      // One large, one small, shifted a bit but still overlapping, with a small z delta.
      double epsilon = 1e-3;
      Vector3D smallZOffset = new Vector3D(0.0, 0.0, epsilon);
      Vector3D overlappingHorizontalOffset = new Vector3D(0.1, 0.2, 0.0);

      translationTwo = new Vector3D(translationOne);
      translationTwo.add(smallZOffset);
      translationTwo.add(overlappingHorizontalOffset);
      squareOne = createASingleTranslatedSquare(translationOne, 1.0, 1.0);
      squareTwo = createASingleTranslatedSquare(translationTwo, 0.5, 0.5);
      assertHighConfidencePairingBothWays(squareOne, squareTwo);

      translationTwo = new Vector3D(translationOne);
      translationTwo.sub(smallZOffset);
      translationTwo.sub(overlappingHorizontalOffset);
      squareOne = createASingleTranslatedSquare(translationOne, 1.0, 1.0);
      squareTwo = createASingleTranslatedSquare(translationTwo, 0.5, 0.5);
      assertHighConfidencePairingBothWays(squareOne, squareTwo);

      // One large, one small, both rotated differently in yaw should still overlap.
      translationTwo = new Vector3D(translationOne);
      squareOne = createASingleTranslatedAndYawedSquare(translationOne, Math.PI / 2.0, 1.0, 1.0);
      squareTwo = createASingleTranslatedAndYawedSquare(translationTwo, -Math.PI / 3.0, 0.5, 0.5);
      assertHighConfidencePairingBothWays(squareOne, squareTwo);

      // Two pitched squares
      double yaw = 0.0;
      double pitch = Math.PI;
      double roll = 0.0;

      squareOne = createASingleSquare(translationOne, yaw, pitch, roll, 1.0, 1.0);
      squareTwo = createASingleSquare(translationOne, yaw, pitch, roll, 1.0, 1.0);
      assertHighConfidencePairingBothWays(squareOne, squareTwo);

      // Two randomly oriented squares
      yaw = Math.PI * 0.5;
      pitch = Math.PI * 0.3;
      roll = -Math.PI * 0.7;

      squareOne = createASingleSquare(translationOne, yaw, pitch, roll, 0.5, 0.5);
      squareTwo = createASingleSquare(translationOne, yaw, pitch, roll, 1.0, 1.0);
      assertHighConfidencePairingBothWays(squareOne, squareTwo);

      // Two squares with different surface normals should not have high confidence
      yaw = 0.0;
      pitch = 0.0;
      roll = 0.0;
      double largePitchDelta = 0.2;

      squareOne = createASingleSquare(translationOne, yaw, pitch, roll, 0.5, 0.5);
      squareOne = createASingleSquare(translationOne, yaw, pitch + largePitchDelta, roll, 1.0, 1.0);
      assertNotAHighConfidencePairing(squareOne, squareTwo);

      yaw = 0.0;
      pitch = 0.0;
      roll = 0.0;
      double largeRollDelta = 0.2;

      squareOne = createASingleSquare(translationOne, yaw, pitch, roll, 0.5, 0.5);
      squareOne = createASingleSquare(translationOne, yaw, pitch, roll + largeRollDelta, 1.0, 1.0);
      assertNotAHighConfidencePairing(squareOne, squareTwo);

   }

   private void assertHighConfidencePairingBothWays(PlanarRegion regionOne, PlanarRegion regionTwo)
   {
      assertHighConfidencePairing(regionOne, regionTwo);
      assertHighConfidencePairing(regionTwo, regionOne);
   }

   private void assertHighConfidencePairing(PlanarRegion mapRegion, PlanarRegion newDataRegion)
   {
      PlanarRegionsList map = new PlanarRegionsList();
      PlanarRegionsList newData = new PlanarRegionsList();

      map.addPlanarRegion(mapRegion);
      newData.addPlanarRegion(newDataRegion);

      PlanarRegionSLAMParameters parameters = new PlanarRegionSLAMParameters();
      parameters.setBoundingBoxHeight(0.02);

      Map<PlanarRegion, PairList<PlanarRegion, Point2D>> highConfidenceMatches = PlanarRegionSLAM.findHighConfidenceRegionMatchesAndReferencePoints(map,
                                                                                                                                                    newData,
                                                                                                                                                    parameters);

      assertEquals(1, highConfidenceMatches.size());
      PairList<PlanarRegion, Point2D> pairList = highConfidenceMatches.get(mapRegion);
      //      assertEquals(1, pairList.size());

      ImmutablePair<PlanarRegion, Point2D> regionAndReferencePoint = pairList.get(0);

      assertTrue(newDataRegion == regionAndReferencePoint.getLeft());
      Point2D referencePoint2D = regionAndReferencePoint.getRight();

      // Make sure the reference point is on and inside the new data region.
      assertTrue(newDataRegion.isPointInside(referencePoint2D));
   }

   private void assertNotAHighConfidencePairing(PlanarRegion regionOne, PlanarRegion regionTwo)
   {
      PlanarRegionsList map = new PlanarRegionsList();
      PlanarRegionsList newData = new PlanarRegionsList();

      map.addPlanarRegion(regionOne);
      newData.addPlanarRegion(regionTwo);

      PlanarRegionSLAMParameters parameters = new PlanarRegionSLAMParameters();
      parameters.setBoundingBoxHeight(0.02);

      Map<PlanarRegion, PairList<PlanarRegion, Point2D>> highConfidenceMatches = PlanarRegionSLAM.findHighConfidenceRegionMatchesAndReferencePoints(map,
                                                                                                                                                    newData,
                                                                                                                                                    parameters);

      assertTrue(highConfidenceMatches.isEmpty());
   }

   private PlanarRegion createASingleTranslatedAndYawedSquare(Vector3D translation, double yaw, double xSize, double ySize)
   {
      Point2D minimumPoint = new Point2D(-xSize / 2.0, -ySize / 2.0);
      Point2D maximumPoint = new Point2D(xSize / 2.0, ySize / 2.0);

      return createASingleSquare(translation, yaw, 0.0, 0.0, minimumPoint, maximumPoint);
   }

   private PlanarRegion createASingleTranslatedSquare(Vector3D translation, double xSize, double ySize)
   {
      Point2D minimumPoint = new Point2D(-xSize / 2.0, -ySize / 2.0);
      Point2D maximumPoint = new Point2D(xSize / 2.0, ySize / 2.0);

      return createASingleSquare(translation, 0.0, 0.0, 0.0, minimumPoint, maximumPoint);
   }

   private PlanarRegion createASingleSquareCenteredAtOrigin(double xSize, double ySize)
   {
      Point2D minimumPoint = new Point2D(-xSize / 2.0, -ySize / 2.0);
      Point2D maximumPoint = new Point2D(xSize / 2.0, ySize / 2.0);

      return createASingleSquare(new Vector3D(), 0.0, 0.0, 0.0, minimumPoint, maximumPoint);
   }

   private PlanarRegion createASingleSquare(Vector3D translation, double yaw, double pitch, double roll, double xSize, double ySize)
   {
      Point2D minimumPoint = new Point2D(-xSize / 2.0, -ySize / 2.0);
      Point2D maximumPoint = new Point2D(xSize / 2.0, ySize / 2.0);

      return createASingleSquare(translation, yaw, pitch, roll, minimumPoint, maximumPoint);
   }

   private PlanarRegion createASingleSquare(Vector3D translation, double yaw, double pitch, double roll, Point2D minimumPoint, Point2D maximumPoint)
   {
      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      transformToWorld.setTranslation(translation);
      transformToWorld.setRotationYawPitchRoll(yaw, pitch, roll);

      ArrayList<Point2D> vertices = new ArrayList<Point2D>();
      vertices.add(new Point2D(minimumPoint));
      vertices.add(new Point2D(minimumPoint.getX(), maximumPoint.getY()));
      vertices.add(new Point2D(maximumPoint));
      vertices.add(new Point2D(maximumPoint.getX(), minimumPoint.getY()));

      ConvexPolygon2D convexPolygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(vertices));
      PlanarRegion square = new PlanarRegion(transformToWorld, convexPolygon);
      return square;
   }

   private void visualizePlanarRegions(PlanarRegionsList planarRegions)
   {
      JavaFXApplicationCreator.createAJavaFXApplication();

      PlanarRegionsGraphic regionsGraphic = new PlanarRegionsGraphic(false);
      regionsGraphic.generateMeshes(planarRegions);
      regionsGraphic.update();

      final CountDownLatch countDownLatch = new CountDownLatch(1);

      Platform.runLater(new Runnable()
      {
         @Override
         public void run()
         {
            //            double preferredWidth = 1000.0;
            //            double preferredHeight = 1000.0;

            View3DFactory view3dFactory = new View3DFactory(1200, 800);
            view3dFactory.addCameraController(0.05, 2000.0, true);
            view3dFactory.addWorldCoordinateSystem(0.3);
            view3dFactory.addDefaultLighting();
            view3dFactory.addNodeToView(regionsGraphic);

            Stage stage = new Stage();
            stage.setTitle(getClass().getSimpleName());
            stage.setMaximized(false);
            stage.setScene(view3dFactory.getScene());

            stage.centerOnScreen();

            stage.show();

            countDownLatch.countDown();
         }
      });

      try
      {
         countDownLatch.await();
      }
      catch (InterruptedException e)
      {
      }

   }
}
