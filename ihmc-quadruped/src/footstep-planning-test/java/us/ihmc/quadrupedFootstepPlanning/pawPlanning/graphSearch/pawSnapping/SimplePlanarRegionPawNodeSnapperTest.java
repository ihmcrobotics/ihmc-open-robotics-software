package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.pawSnapping;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.DefaultPawStepPlannerParameters;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.Random;

public class SimplePlanarRegionPawNodeSnapperTest
{
   @Test
   public void testSnapToFlatSquare()
   {
      double squareHeight = 4.6994;
      double epsilon = 1e-6;

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.translate(0.0, 0.0, squareHeight);
      generator.addRectangle(10.0, 10.0);
      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();

      DefaultPawStepPlannerParameters parameters = new DefaultPawStepPlannerParameters();
      SimplePlanarRegionPawNodeSnapper snapper = new SimplePlanarRegionPawNodeSnapper(parameters, true);
      snapper.setPlanarRegions(planarRegionsList);

      int xIndex = -3;
      int yIndex = 5;

      double x = xIndex * PawNode.gridSizeXY;
      double y = yIndex * PawNode.gridSizeXY;
      RobotQuadrant robotQuadrant = RobotQuadrant.FRONT_LEFT;
      double yaw = 0.0;

      PawNodeSnapData snapData = snapper.snapPawNode(robotQuadrant, xIndex, yIndex, yaw);
      RigidBodyTransform snapTransform = snapData.getSnapTransform();

      Point3D snappedPoint = new Point3D(x, y, 0.0);
      Point3D expectedPoint = new Point3D(x, y, squareHeight);
      snappedPoint.applyTransform(snapTransform);
      QuaternionReadOnly expectedSnapRotation = new Quaternion();
      Quaternion snapRotation = new Quaternion();
      snapRotation.set(snapTransform.getRotation());
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedPoint, snappedPoint, epsilon);
      EuclidCoreTestTools.assertEquals(expectedSnapRotation, snapRotation, epsilon);
   }

   @Test
   public void testSnapToAngledSquare()
   {
      double squareHeightAtOrigin = 3.909;
      double squareRoll = 0.33;
      double squarePitch = -0.2;
      double epsilon = 1e-6;

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      Quaternion rotation = new Quaternion(0.0, squareRoll, squarePitch);
      generator.translate(0.0, 0.0, squareHeightAtOrigin);
      generator.rotate(rotation);
      generator.addRectangle(10.0, 10.0);

      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();

      DefaultPawStepPlannerParameters parameters = new DefaultPawStepPlannerParameters();
      SimplePlanarRegionPawNodeSnapper snapper = new SimplePlanarRegionPawNodeSnapper(parameters,true);
      snapper.setPlanarRegions(planarRegionsList);

      int xIndex = 4;
      int yIndex = -2;
      double x = xIndex * PawNode.gridSizeXY;
      double y = yIndex * PawNode.gridSizeXY;

      PawNodeSnapData snapData = snapper.snapPawNode(RobotQuadrant.FRONT_LEFT, xIndex, yIndex, 0.0);
      RigidBodyTransform snapTransform = snapData.getSnapTransform();

      PlanarRegion planarRegion = planarRegionsList.getPlanarRegion(0);
      double heightAtPoint = planarRegion.getPlaneZGivenXY(x, y);

      Vector3D translation = new Vector3D(0.0, 0.0, heightAtPoint);
      rotation.transform(translation);

      Point3D snappedPoint = new Point3D(x, y, 0.0);
      Point3D expectedPoint = new Point3D(x, y, heightAtPoint);
      snappedPoint.applyTransform(snapTransform);
      Quaternion snapRotation = new Quaternion();
      snapRotation.set(snapTransform.getRotation());
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedPoint, snappedPoint, epsilon);
      EuclidCoreTestTools.assertEquals(rotation, snapRotation, epsilon);
   }

   @Test
   public void testSnapperSnapsToHighestRegion()
   {
      Random random = new Random(399);
      double epsilon = 1e-6;
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();

      double highestRegionHeight = 100.22;
      double highestRegionPitch = 0.2;
      double highestRegionRoll = -0.15;

      generator.translate(0.0, 0.0, highestRegionHeight);
      Quaternion expectedSnapOrientation = new Quaternion(0.0, highestRegionPitch, highestRegionRoll);
      generator.rotate(expectedSnapOrientation);
      generator.addRectangle(10.0, 10.0);

      for (int i = 0; i < 10; i++)
      {
         generator.identity();
         generator.translate(0.0, 0.0, EuclidCoreRandomTools.nextDouble(random, -50.0, highestRegionHeight - 10.0));
         generator.rotate(new Quaternion(0.0, EuclidCoreRandomTools.nextDouble(random, 0.2), EuclidCoreRandomTools.nextDouble(random, 0.2)));
         generator.addRectangle(10.0, 10.0);
      }

      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();

      DefaultPawStepPlannerParameters parameters = new DefaultPawStepPlannerParameters();
      SimplePlanarRegionPawNodeSnapper snapper = new SimplePlanarRegionPawNodeSnapper(parameters,true);
      snapper.setPlanarRegions(planarRegionsList);

      int xIndex = 7;
      int yIndex = -10;
      double x = xIndex * PawNode.gridSizeXY;
      double y = yIndex * PawNode.gridSizeXY;

      PawNodeSnapData snapData = snapper.snapPawNode(RobotQuadrant.FRONT_LEFT, xIndex, yIndex, 0.0);
      RigidBodyTransform snapTransform = snapData.getSnapTransform();

      PlanarRegion planarRegion = planarRegionsList.getPlanarRegion(0);
      double heightAtPoint = planarRegion.getPlaneZGivenXY(x, y);

      Point3D snappedPoint = new Point3D(x, y, 0.0);
      Point3D expectedPoint = new Point3D(x, y, heightAtPoint);
      snappedPoint.applyTransform(snapTransform);
      Quaternion snapRotation = new Quaternion();
      snapRotation.set(snapTransform.getRotation());

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedPoint, snappedPoint, epsilon);
      EuclidCoreTestTools.assertEquals(expectedSnapOrientation, snapRotation, epsilon);
   }

   @Test
   public void testTryingToProjectFromFarAway()
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.addRectangle(1.0, 1.0);
      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();

      DefaultPawStepPlannerParameters parameters = new DefaultPawStepPlannerParameters();
      SimplePlanarRegionPawNodeSnapper snapper = new SimplePlanarRegionPawNodeSnapper(parameters,true);
      snapper.setPlanarRegions(planarRegionsList);

      RobotQuadrant robotQuadrant = RobotQuadrant.FRONT_LEFT;
      double yaw = 0.0;

      PawNodeSnapData snapData = snapper.snapPawNode(robotQuadrant, 1000, 1, yaw);
      Assertions.assertTrue(snapData == PawNodeSnapData.emptyData());

      snapData = snapper.snapPawNode(robotQuadrant, 1, 1000, yaw);
      Assertions.assertTrue(snapData == PawNodeSnapData.emptyData());
   }

   @Test
   public void testProjectingIntoFlatSquare()
   {
      double epsilon = 1e-6;
      int squareCellHalfWidth = 10;

      double extraSquareWidth = 0.001;
      double squareWidth = 2 * (squareCellHalfWidth * PawNode.gridSizeXY + extraSquareWidth);

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.addRectangle(squareWidth, squareWidth);
      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();

      DefaultPawStepPlannerParameters parameters = new DefaultPawStepPlannerParameters();
      SimplePlanarRegionPawNodeSnapper snapper = new SimplePlanarRegionPawNodeSnapper(parameters,true);
      snapper.setPlanarRegions(planarRegionsList);

      double projectionDistance = new DefaultPawStepPlannerParameters().getProjectInsideDistance();
      double expectedTranslation = projectionDistance - extraSquareWidth;

      RobotQuadrant robotQuadrant = RobotQuadrant.FRONT_LEFT;
      double yaw = 0.0;
      // test snapping on front edge
      PawNodeSnapData snapData = snapper.snapPawNode(robotQuadrant, squareCellHalfWidth, 2, yaw);
      Vector3DReadOnly translationVector = snapData.getSnapTransform().getTranslation();
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new Vector3D(-expectedTranslation, 0.0, 0.0), translationVector, epsilon);

      // test snapping on back edge
      snapData = snapper.snapPawNode(robotQuadrant, -squareCellHalfWidth, 3, yaw);
      translationVector = snapData.getSnapTransform().getTranslation();
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new Vector3D(expectedTranslation, 0.0, 0.0), translationVector, epsilon);

      // test snapping on left edge
      snapData = snapper.snapPawNode(robotQuadrant, 1, squareCellHalfWidth, yaw);
      translationVector = snapData.getSnapTransform().getTranslation();
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new Vector3D(0.0, -expectedTranslation, 0.0), translationVector, epsilon);

      // test snapping on right edge
      snapData = snapper.snapPawNode(robotQuadrant, 2, -squareCellHalfWidth, yaw);
      translationVector = snapData.getSnapTransform().getTranslation();
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new Vector3D(0.0, expectedTranslation, 0.0), translationVector, epsilon);

      // test snapping on front-left corner
      snapData = snapper.snapPawNode(robotQuadrant, squareCellHalfWidth, squareCellHalfWidth, yaw);
      translationVector = snapData.getSnapTransform().getTranslation();
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new Vector3D(-expectedTranslation, -expectedTranslation, 0.0), translationVector, epsilon);

      // test snapping on front-right corner
      snapData = snapper.snapPawNode(robotQuadrant, squareCellHalfWidth, -squareCellHalfWidth, yaw);
      translationVector = snapData.getSnapTransform().getTranslation();
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new Vector3D(-expectedTranslation, expectedTranslation, 0.0), translationVector, epsilon);

      // test snapping on back-left corner
      snapData = snapper.snapPawNode(robotQuadrant, -squareCellHalfWidth, squareCellHalfWidth, yaw);
      translationVector = snapData.getSnapTransform().getTranslation();
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new Vector3D(expectedTranslation, -expectedTranslation, 0.0), translationVector, epsilon);

      // test snapping on back-right corner
      snapData = snapper.snapPawNode(robotQuadrant, -squareCellHalfWidth, -squareCellHalfWidth, yaw);
      translationVector = snapData.getSnapTransform().getTranslation();
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new Vector3D(expectedTranslation, expectedTranslation, 0.0), translationVector, epsilon);
   }

   @Test
   public void testSnapFromOutsideOfRegion()
   {
      double epsilon = 1e-6;
      int squareCellHalfWidth = 10;

      double widthShrinkAmount = -0.5 * PawNode.gridSizeXY + 0.001;
      double squareWidth = 2 * (squareCellHalfWidth * PawNode.gridSizeXY + widthShrinkAmount);

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.addRectangle(squareWidth, squareWidth);
      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();

      double projectionDistance = 0.0;
      TestParameters parameters = new TestParameters(projectionDistance);
      SimplePlanarRegionPawNodeSnapper snapper = new SimplePlanarRegionPawNodeSnapper(parameters,true);
      snapper.setPlanarRegions(planarRegionsList);

      double expectedTranslation = -widthShrinkAmount;

      RobotQuadrant robotQuadrant = RobotQuadrant.FRONT_LEFT;
      double yaw = 0.0;

      // test snapping on front edge
      PawNodeSnapData snapData = snapper.snapPawNode(robotQuadrant, squareCellHalfWidth, 2, yaw);
      Vector3DReadOnly translationVector = snapData.getSnapTransform().getTranslation();
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new Vector3D(-expectedTranslation, 0.0, 0.0), translationVector, epsilon);

      // test snapping on back edge
      snapData = snapper.snapPawNode(robotQuadrant, -squareCellHalfWidth, 3, yaw);
      translationVector = snapData.getSnapTransform().getTranslation();
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new Vector3D(expectedTranslation, 0.0, 0.0), translationVector, epsilon);

      // test snapping on left edge
      snapData = snapper.snapPawNode(robotQuadrant, 1, squareCellHalfWidth, yaw);
      translationVector = snapData.getSnapTransform().getTranslation();
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new Vector3D(0.0, -expectedTranslation, 0.0), translationVector, epsilon);

      // test snapping on right edge
      snapData = snapper.snapPawNode(robotQuadrant, 2, -squareCellHalfWidth, yaw);
      translationVector = snapData.getSnapTransform().getTranslation();
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new Vector3D(0.0, expectedTranslation, 0.0), translationVector, epsilon);

      // test snapping on front-left corner
      snapData = snapper.snapPawNode(robotQuadrant, squareCellHalfWidth, squareCellHalfWidth, yaw);
      translationVector = snapData.getSnapTransform().getTranslation();
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new Vector3D(-expectedTranslation, -expectedTranslation, 0.0), translationVector, epsilon);

      // test snapping on front-right corner
      snapData = snapper.snapPawNode(robotQuadrant, squareCellHalfWidth, -squareCellHalfWidth, yaw);
      translationVector = snapData.getSnapTransform().getTranslation();
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new Vector3D(-expectedTranslation, expectedTranslation, 0.0), translationVector, epsilon);

      // test snapping on back-left corner
      snapData = snapper.snapPawNode(robotQuadrant, -squareCellHalfWidth, squareCellHalfWidth, yaw);
      translationVector = snapData.getSnapTransform().getTranslation();
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new Vector3D(expectedTranslation, -expectedTranslation, 0.0), translationVector, epsilon);

      // test snapping on back-right corner
      snapData = snapper.snapPawNode(robotQuadrant, -squareCellHalfWidth, -squareCellHalfWidth, yaw);
      translationVector = snapData.getSnapTransform().getTranslation();
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new Vector3D(expectedTranslation, expectedTranslation, 0.0), translationVector, epsilon);
   }

   @Test
   public void testSnapAndProjectFromOutsideOfRegion()
   {
      double epsilon = 1e-6;
      int squareCellHalfWidth = 10;
      double projectionDistance = 0.015;

      // NOTE: this test will likely fail if PawNode.gridSizeXY is changed to be smaller than projectionDistance
      double widthShrinkAmount = - 0.5 * PawNode.gridSizeXY + projectionDistance + 0.001;
      double expectedTranslation = - widthShrinkAmount + projectionDistance;
      double squareWidth = 2 * (squareCellHalfWidth * PawNode.gridSizeXY + widthShrinkAmount);

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.addRectangle(squareWidth, squareWidth);
      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();

      TestParameters parameters = new TestParameters(projectionDistance);
      SimplePlanarRegionPawNodeSnapper snapper = new SimplePlanarRegionPawNodeSnapper(parameters,true);
      snapper.setPlanarRegions(planarRegionsList);

      RobotQuadrant robotQuadrant = RobotQuadrant.FRONT_LEFT;
      double yaw = 0.0;

      // test snapping on front edge
      PawNodeSnapData snapData = snapper.snapPawNode(robotQuadrant, squareCellHalfWidth, 2, yaw);
      Point3D snappedPoint = new Point3D(squareCellHalfWidth * PawNode.gridSizeXY, 2 * PawNode.gridSizeXY, 0.0);
      Point3D expectedSnappedPoint = new Point3D(squareCellHalfWidth * PawNode.gridSizeXY - expectedTranslation, 2 * PawNode.gridSizeXY, 0.0);
      snappedPoint.applyTransform(snapData.getSnapTransform());
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedSnappedPoint, snappedPoint, epsilon);
      //      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new Vector3D(-expectedTranslation, 0.0, 0.0), translationVector, epsilon);

      // test snapping on back edge
      snapData = snapper.snapPawNode(robotQuadrant, -squareCellHalfWidth, 3, yaw);
      snappedPoint = new Point3D(-squareCellHalfWidth * PawNode.gridSizeXY, 3 * PawNode.gridSizeXY, 0.0);
      expectedSnappedPoint = new Point3D(-squareCellHalfWidth * PawNode.gridSizeXY + expectedTranslation, 3 * PawNode.gridSizeXY, 0.0);
      snappedPoint.applyTransform(snapData.getSnapTransform());
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedSnappedPoint, snappedPoint, epsilon);

      // test snapping on left edge
      snapData = snapper.snapPawNode(robotQuadrant, 1, squareCellHalfWidth, yaw);

      snappedPoint = new Point3D(PawNode.gridSizeXY, squareCellHalfWidth * PawNode.gridSizeXY, 0.0);
      expectedSnappedPoint = new Point3D(PawNode.gridSizeXY, squareCellHalfWidth * PawNode.gridSizeXY - expectedTranslation, 0.0);
      snappedPoint.applyTransform(snapData.getSnapTransform());
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedSnappedPoint, snappedPoint, epsilon);

      // test snapping on right edge
      snapData = snapper.snapPawNode(robotQuadrant, 2, -squareCellHalfWidth, yaw);
      snappedPoint = new Point3D(2 * PawNode.gridSizeXY, -squareCellHalfWidth * PawNode.gridSizeXY, 0.0);
      expectedSnappedPoint = new Point3D(2 * PawNode.gridSizeXY, -squareCellHalfWidth * PawNode.gridSizeXY + expectedTranslation, 0.0);
      snappedPoint.applyTransform(snapData.getSnapTransform());
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedSnappedPoint, snappedPoint, epsilon);

      // test snapping on right edge
      snapData = snapper.snapPawNode(robotQuadrant, 2, -squareCellHalfWidth, yaw);
      snappedPoint = new Point3D(2 * PawNode.gridSizeXY, -squareCellHalfWidth * PawNode.gridSizeXY, 0.0);
      expectedSnappedPoint = new Point3D(2 * PawNode.gridSizeXY, -squareCellHalfWidth * PawNode.gridSizeXY + expectedTranslation, 0.0);
      snappedPoint.applyTransform(snapData.getSnapTransform());
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedSnappedPoint, snappedPoint, epsilon);

      // test snapping on front-left corner
      snapData = snapper.snapPawNode(robotQuadrant, squareCellHalfWidth, squareCellHalfWidth, yaw);
      snappedPoint = new Point3D(squareCellHalfWidth * PawNode.gridSizeXY, squareCellHalfWidth * PawNode.gridSizeXY, 0.0);
      expectedSnappedPoint = new Point3D(squareCellHalfWidth * PawNode.gridSizeXY - expectedTranslation,
                                         squareCellHalfWidth * PawNode.gridSizeXY - expectedTranslation,
                                         0.0);
      snappedPoint.applyTransform(snapData.getSnapTransform());
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedSnappedPoint, snappedPoint, epsilon);

      // test snapping on front-right corner
      snapData = snapper.snapPawNode(robotQuadrant, squareCellHalfWidth, -squareCellHalfWidth, yaw);
      snappedPoint = new Point3D(squareCellHalfWidth * PawNode.gridSizeXY, -squareCellHalfWidth * PawNode.gridSizeXY, 0.0);
      expectedSnappedPoint = new Point3D(squareCellHalfWidth * PawNode.gridSizeXY - expectedTranslation,
                                         -squareCellHalfWidth * PawNode.gridSizeXY + expectedTranslation,
                                         0.0);
      snappedPoint.applyTransform(snapData.getSnapTransform());
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedSnappedPoint, snappedPoint, epsilon);

      // test snapping on back-left corner
      snapData = snapper.snapPawNode(robotQuadrant, -squareCellHalfWidth, squareCellHalfWidth, yaw);
      snappedPoint = new Point3D(-squareCellHalfWidth * PawNode.gridSizeXY, squareCellHalfWidth * PawNode.gridSizeXY, 0.0);
      expectedSnappedPoint = new Point3D(-squareCellHalfWidth * PawNode.gridSizeXY + expectedTranslation,
                                         squareCellHalfWidth * PawNode.gridSizeXY - expectedTranslation,
                                         0.0);
      snappedPoint.applyTransform(snapData.getSnapTransform());
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedSnappedPoint, snappedPoint, epsilon);

      // test snapping on back-right corner
      snapData = snapper.snapPawNode(robotQuadrant, -squareCellHalfWidth, -squareCellHalfWidth, yaw);
      snappedPoint = new Point3D(-squareCellHalfWidth * PawNode.gridSizeXY, -squareCellHalfWidth * PawNode.gridSizeXY, 0.0);
      expectedSnappedPoint = new Point3D(-squareCellHalfWidth * PawNode.gridSizeXY + expectedTranslation,
                                         -squareCellHalfWidth * PawNode.gridSizeXY + expectedTranslation,
                                         0.0);
      snappedPoint.applyTransform(snapData.getSnapTransform());
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedSnappedPoint, snappedPoint, epsilon);
   }

   @Test
   public void testProjectingIntoRolledSquare()
   {
      double epsilon = 1e-6;
      double rollAngle = Math.toRadians(30.0);
      int squareCellPlanarHalfWidth = 10;
      double extraSquareWidth = 0.001;
      double squareWidth = 2.0 * extraSquareWidth + 2.0 * (squareCellPlanarHalfWidth * PawNode.gridSizeXY) / Math.cos(rollAngle);

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.rotate(rollAngle, Axis3D.X);
      generator.addRectangle(squareWidth, squareWidth);
      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();

      DefaultPawStepPlannerParameters parameters = new DefaultPawStepPlannerParameters();
      SimplePlanarRegionPawNodeSnapper snapper = new SimplePlanarRegionPawNodeSnapper(parameters,true);
      snapper.setPlanarRegions(planarRegionsList);

      double projectionDistance = new DefaultPawStepPlannerParameters().getProjectInsideDistance();
      double expectedTranslation = (projectionDistance - extraSquareWidth) * Math.cos(rollAngle);

      PlanarRegion region = planarRegionsList.getPlanarRegion(0);

      // test snapping on left edge
      int xIndex = 1;
      int yIndex = squareCellPlanarHalfWidth;
      double x = xIndex * PawNode.gridSizeXY;
      double y = yIndex * PawNode.gridSizeXY;

      RobotQuadrant robotQuadrant = RobotQuadrant.FRONT_LEFT;
      double yaw = 0.0;

      PawNodeSnapData snapData = snapper.snapPawNode(robotQuadrant, xIndex, yIndex, yaw);

      Point3D snappedPoint = new Point3D(x, y, 0.0);
      snappedPoint.applyTransform(snapData.getSnapTransform());
      double regionHeight = region.getPlaneZGivenXY(x, snappedPoint.getY());
      Point3D expectedSnappedPoint = new Point3D(x, snappedPoint.getY(), regionHeight);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedSnappedPoint, snappedPoint, epsilon);

      // test snapping on right edge
      xIndex = 3;
      yIndex = -squareCellPlanarHalfWidth;
      x = xIndex * PawNode.gridSizeXY;
      y = yIndex * PawNode.gridSizeXY;

      snapData = snapper.snapPawNode(robotQuadrant, xIndex, yIndex, yaw);

      snappedPoint = new Point3D(x, y, 0.0);
      snappedPoint.applyTransform(snapData.getSnapTransform());
      regionHeight = region.getPlaneZGivenXY(x, snappedPoint.getY());
      expectedSnappedPoint = new Point3D(x, snappedPoint.getY(), regionHeight);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedSnappedPoint, snappedPoint, epsilon);
   }

   @Test
   public void testProjectingIntoPitchedSquare()
   {
      double epsilon = 1e-6;
      double pitchAngle = Math.toRadians(30.0);
      int squareCellPlanarHalfWidth = 10;
      double extraSquareWidth = 0.001;
      double squareWidth = 2.0 * extraSquareWidth + 2.0 * (squareCellPlanarHalfWidth * PawNode.gridSizeXY) / Math.cos(pitchAngle);

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.rotate(pitchAngle, Axis3D.Y);
      generator.addRectangle(squareWidth, squareWidth);
      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();

      DefaultPawStepPlannerParameters parameters = new DefaultPawStepPlannerParameters();
      SimplePlanarRegionPawNodeSnapper snapper = new SimplePlanarRegionPawNodeSnapper(parameters,true);
      snapper.setPlanarRegions(planarRegionsList);

      double projectionDistance = new DefaultPawStepPlannerParameters().getProjectInsideDistance();
      double expectedTranslation = (projectionDistance - extraSquareWidth) * Math.cos(pitchAngle);

      PlanarRegion region = planarRegionsList.getPlanarRegion(0);

      // test snapping on front edge
      int xIndex = squareCellPlanarHalfWidth;
      int yIndex = -2;
      double x = xIndex * PawNode.gridSizeXY;
      double y = yIndex * PawNode.gridSizeXY;

      RobotQuadrant robotQuadrant = RobotQuadrant.FRONT_LEFT;
      double yaw = 0.0;
      PawNodeSnapData snapData = snapper.snapPawNode(robotQuadrant, xIndex, yIndex, yaw);

      Point3D snappedPoint = new Point3D(x, y, 0.0);
      snappedPoint.applyTransform(snapData.getSnapTransform());
      double regionHeight = region.getPlaneZGivenXY(snappedPoint.getX(), y);
      Point3D expectedSnappedPoint = new Point3D(snappedPoint.getX(), y, regionHeight);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedSnappedPoint, snappedPoint, epsilon);

      // test snapping on back edge
      xIndex = -squareCellPlanarHalfWidth;
      yIndex = 0;
      x = xIndex * PawNode.gridSizeXY;
      y = yIndex * PawNode.gridSizeXY;

      snapData = snapper.snapPawNode(robotQuadrant, xIndex, yIndex, yaw);

      snappedPoint = new Point3D(x, y, 0.0);
      snappedPoint.applyTransform(snapData.getSnapTransform());
      regionHeight = region.getPlaneZGivenXY(snappedPoint.getX(), y);
      expectedSnappedPoint = new Point3D(snappedPoint.getX(), y, regionHeight);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedSnappedPoint, snappedPoint, epsilon);
   }

   private class TestParameters extends DefaultPawStepPlannerParameters
   {
      final double projectionDistance;

      TestParameters(double projectionDistance)
      {
         this.projectionDistance = projectionDistance;
      }

      @Override
      public double getProjectInsideDistance()
      {
         return projectionDistance;
      }
   }
}