package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;

import java.util.Random;

public class SimplePlanarRegionFootstepNodeSnapperTest
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

      SimplePlanarRegionFootstepNodeSnapper snapper = new SimplePlanarRegionFootstepNodeSnapper();
      snapper.setPlanarRegions(planarRegionsList);

      int xIndex = -3;
      int yIndex = 5;

      FootstepNodeSnapData snapData = snapper.snapFootstepNode(xIndex, yIndex);
      RigidBodyTransform snapTransform = snapData.getSnapTransform();

      Assertions.assertFalse(snapTransform.hasRotation());

      Vector3D expectedSnapTransform = new Vector3D();
      expectedSnapTransform.set(0.0, 0.0, squareHeight);
      Assertions.assertTrue(snapTransform.getTranslationVector().epsilonEquals(expectedSnapTransform, epsilon));
   }

   @Test
   public void testSnapToAngledSquare()
   {
      double squareHeightAtOrigin = 3.909;
      double squareRoll = 0.33;
      double squarePitch = -0.2;
      double epsilon = 1e-6;

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.translate(0.0, 0.0, squareHeightAtOrigin);
      generator.rotate(new Quaternion(0.0, squareRoll, squarePitch));
      generator.addRectangle(10.0, 10.0);
      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();

      SimplePlanarRegionFootstepNodeSnapper snapper = new SimplePlanarRegionFootstepNodeSnapper();
      snapper.setPlanarRegions(planarRegionsList);

      int xIndex = 4;
      int yIndex = -2;
      double x = xIndex * FootstepNode.gridSizeXY;
      double y = yIndex * FootstepNode.gridSizeXY;

      FootstepNodeSnapData snapData = snapper.snapFootstepNode(xIndex, yIndex);
      RigidBodyTransform snapTransform = snapData.getSnapTransform();

      Assertions.assertFalse(snapTransform.hasRotation());

      PlanarRegion planarRegion = planarRegionsList.getPlanarRegion(0);
      double heightAtPoint = planarRegion.getPlaneZGivenXY(x, y);

      Vector3D expectedSnapTransform = new Vector3D();
      expectedSnapTransform.set(0.0, 0.0, heightAtPoint);
      Assertions.assertTrue(snapTransform.getTranslationVector().epsilonEquals(expectedSnapTransform, epsilon));
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
      generator.rotate(new Quaternion(0.0, highestRegionPitch, highestRegionRoll));
      generator.addRectangle(10.0, 10.0);

      for (int i = 0; i < 10; i++)
      {
         generator.identity();
         generator.translate(0.0, 0.0, EuclidCoreRandomTools.nextDouble(random, -50.0, highestRegionHeight - 10.0));
         generator.rotate(new Quaternion(0.0, EuclidCoreRandomTools.nextDouble(random, 0.2), EuclidCoreRandomTools.nextDouble(random, 0.2)));
         generator.addRectangle(10.0, 10.0);
      }

      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();

      SimplePlanarRegionFootstepNodeSnapper snapper = new SimplePlanarRegionFootstepNodeSnapper();
      snapper.setPlanarRegions(planarRegionsList);

      int xIndex = 7;
      int yIndex = -10;
      double x = xIndex * FootstepNode.gridSizeXY;
      double y = yIndex * FootstepNode.gridSizeXY;

      FootstepNodeSnapData snapData = snapper.snapFootstepNode(xIndex, yIndex);
      RigidBodyTransform snapTransform = snapData.getSnapTransform();

      Assertions.assertFalse(snapTransform.hasRotation());

      PlanarRegion planarRegion = planarRegionsList.getPlanarRegion(0);
      double heightAtPoint = planarRegion.getPlaneZGivenXY(x, y);

      Vector3D expectedSnapTransform = new Vector3D();
      expectedSnapTransform.set(0.0, 0.0, heightAtPoint);
      Assertions.assertTrue(snapTransform.getTranslationVector().epsilonEquals(expectedSnapTransform, epsilon));
   }

   @Test
   public void testTryingToProjectFromFarAway()
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.addRectangle(1.0, 1.0);
      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();

      SimplePlanarRegionFootstepNodeSnapper snapper = new SimplePlanarRegionFootstepNodeSnapper();
      snapper.setPlanarRegions(planarRegionsList);

      FootstepNodeSnapData snapData = snapper.snapFootstepNode(1000, 1);
      Assertions.assertTrue(snapData == FootstepNodeSnapData.emptyData());

      snapData = snapper.snapFootstepNode(1, 1000);
      Assertions.assertTrue(snapData == FootstepNodeSnapData.emptyData());
   }

   @Test
   public void testProjectingIntoFlatSquare()
   {
      double epsilon = 1e-6;
      int squareCellHalfWidth = 10;

      double extraSquareWidth = 0.001;
      double squareWidth = 2 * (squareCellHalfWidth * FootstepNode.gridSizeXY + extraSquareWidth);

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.addRectangle(squareWidth, squareWidth);
      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();

      SimplePlanarRegionFootstepNodeSnapper snapper = new SimplePlanarRegionFootstepNodeSnapper();
      snapper.setPlanarRegions(planarRegionsList);

      double projectionDistance = new DefaultFootstepPlannerParameters().getProjectInsideDistance();
      double expectedTranslation = projectionDistance - extraSquareWidth;

      // test snapping on front edge
      FootstepNodeSnapData snapData = snapper.snapFootstepNode(squareCellHalfWidth, 2);
      Vector3DReadOnly translationVector = snapData.getSnapTransform().getTranslationVector();
      Assertions.assertTrue(translationVector.epsilonEquals(new Vector3D(-expectedTranslation, 0.0, 0.0), epsilon));

      // test snapping on back edge
      snapData = snapper.snapFootstepNode(-squareCellHalfWidth, 3);
      translationVector = snapData.getSnapTransform().getTranslationVector();
      Assertions.assertTrue(translationVector.epsilonEquals(new Vector3D(expectedTranslation, 0.0, 0.0), epsilon));

      // test snapping on left edge
      snapData = snapper.snapFootstepNode(1, squareCellHalfWidth);
      translationVector = snapData.getSnapTransform().getTranslationVector();
      Assertions.assertTrue(translationVector.epsilonEquals(new Vector3D(0.0, -expectedTranslation, 0.0), epsilon));

      // test snapping on right edge
      snapData = snapper.snapFootstepNode(2, -squareCellHalfWidth);
      translationVector = snapData.getSnapTransform().getTranslationVector();
      Assertions.assertTrue(translationVector.epsilonEquals(new Vector3D(0.0, expectedTranslation, 0.0), epsilon));

      // test snapping on front-left corner
      snapData = snapper.snapFootstepNode(squareCellHalfWidth, squareCellHalfWidth);
      translationVector = snapData.getSnapTransform().getTranslationVector();
      Assertions.assertTrue(translationVector.epsilonEquals(new Vector3D(-expectedTranslation, -expectedTranslation, 0.0), epsilon));

      // test snapping on front-right corner
      snapData = snapper.snapFootstepNode(squareCellHalfWidth, -squareCellHalfWidth);
      translationVector = snapData.getSnapTransform().getTranslationVector();
      Assertions.assertTrue(translationVector.epsilonEquals(new Vector3D(-expectedTranslation, expectedTranslation, 0.0), epsilon));

      // test snapping on back-left corner
      snapData = snapper.snapFootstepNode(-squareCellHalfWidth, squareCellHalfWidth);
      translationVector = snapData.getSnapTransform().getTranslationVector();
      Assertions.assertTrue(translationVector.epsilonEquals(new Vector3D(expectedTranslation, -expectedTranslation, 0.0), epsilon));

      // test snapping on back-right corner
      snapData = snapper.snapFootstepNode(-squareCellHalfWidth, -squareCellHalfWidth);
      translationVector = snapData.getSnapTransform().getTranslationVector();
      Assertions.assertTrue(translationVector.epsilonEquals(new Vector3D(expectedTranslation, expectedTranslation, 0.0), epsilon));
   }

   @Test
   public void testSnapFromOutsideOfRegion()
   {
      double epsilon = 1e-6;
      int squareCellHalfWidth = 10;

      double widthShrinkAmount = - 0.5 * FootstepNode.gridSizeXY + 0.001;
      double squareWidth = 2 * (squareCellHalfWidth * FootstepNode.gridSizeXY + widthShrinkAmount);

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.addRectangle(squareWidth, squareWidth);
      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();

      double projectionDistance = 0.0;
      TestParameters parameters = new TestParameters(projectionDistance);
      SimplePlanarRegionFootstepNodeSnapper snapper = new SimplePlanarRegionFootstepNodeSnapper(parameters);
      snapper.setPlanarRegions(planarRegionsList);

      double expectedTranslation = - widthShrinkAmount;

      // test snapping on front edge
      FootstepNodeSnapData snapData = snapper.snapFootstepNode(squareCellHalfWidth, 2);
      Vector3DReadOnly translationVector = snapData.getSnapTransform().getTranslationVector();
      Assertions.assertTrue(translationVector.epsilonEquals(new Vector3D(-expectedTranslation, 0.0, 0.0), epsilon));

      // test snapping on back edge
      snapData = snapper.snapFootstepNode(-squareCellHalfWidth, 3);
      translationVector = snapData.getSnapTransform().getTranslationVector();
      Assertions.assertTrue(translationVector.epsilonEquals(new Vector3D(expectedTranslation, 0.0, 0.0), epsilon));

      // test snapping on left edge
      snapData = snapper.snapFootstepNode(1, squareCellHalfWidth);
      translationVector = snapData.getSnapTransform().getTranslationVector();
      Assertions.assertTrue(translationVector.epsilonEquals(new Vector3D(0.0, -expectedTranslation, 0.0), epsilon));

      // test snapping on right edge
      snapData = snapper.snapFootstepNode(2, -squareCellHalfWidth);
      translationVector = snapData.getSnapTransform().getTranslationVector();
      Assertions.assertTrue(translationVector.epsilonEquals(new Vector3D(0.0, expectedTranslation, 0.0), epsilon));

      // test snapping on front-left corner
      snapData = snapper.snapFootstepNode(squareCellHalfWidth, squareCellHalfWidth);
      translationVector = snapData.getSnapTransform().getTranslationVector();
      Assertions.assertTrue(translationVector.epsilonEquals(new Vector3D(-expectedTranslation, -expectedTranslation, 0.0), epsilon));

      // test snapping on front-right corner
      snapData = snapper.snapFootstepNode(squareCellHalfWidth, -squareCellHalfWidth);
      translationVector = snapData.getSnapTransform().getTranslationVector();
      Assertions.assertTrue(translationVector.epsilonEquals(new Vector3D(-expectedTranslation, expectedTranslation, 0.0), epsilon));

      // test snapping on back-left corner
      snapData = snapper.snapFootstepNode(-squareCellHalfWidth, squareCellHalfWidth);
      translationVector = snapData.getSnapTransform().getTranslationVector();
      Assertions.assertTrue(translationVector.epsilonEquals(new Vector3D(expectedTranslation, -expectedTranslation, 0.0), epsilon));

      // test snapping on back-right corner
      snapData = snapper.snapFootstepNode(-squareCellHalfWidth, -squareCellHalfWidth);
      translationVector = snapData.getSnapTransform().getTranslationVector();
      Assertions.assertTrue(translationVector.epsilonEquals(new Vector3D(expectedTranslation, expectedTranslation, 0.0), epsilon));
   }

   @Test
   public void testSnapAndProjectFromOutsideOfRegion()
   {
      double epsilon = 1e-6;
      int squareCellHalfWidth = 10;
      double projectionDistance = 0.015;

      // NOTE: this test will likely fail if FootstepNode.gridSizeXY is changed to be smaller than projectionDistance
      double widthShrinkAmount = - 0.5 * FootstepNode.gridSizeXY + projectionDistance + 0.001;
      double expectedTranslation = - widthShrinkAmount + projectionDistance;
      double squareWidth = 2 * (squareCellHalfWidth * FootstepNode.gridSizeXY + widthShrinkAmount);

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.addRectangle(squareWidth, squareWidth);
      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();

      TestParameters parameters = new TestParameters(projectionDistance);
      SimplePlanarRegionFootstepNodeSnapper snapper = new SimplePlanarRegionFootstepNodeSnapper(parameters);
      snapper.setPlanarRegions(planarRegionsList);

      // test snapping on front edge
      FootstepNodeSnapData snapData = snapper.snapFootstepNode(squareCellHalfWidth, 2);
      Vector3DReadOnly translationVector = snapData.getSnapTransform().getTranslationVector();
      Assertions.assertTrue(translationVector.epsilonEquals(new Vector3D(-expectedTranslation, 0.0, 0.0), epsilon));

      // test snapping on back edge
      snapData = snapper.snapFootstepNode(-squareCellHalfWidth, 3);
      translationVector = snapData.getSnapTransform().getTranslationVector();
      Assertions.assertTrue(translationVector.epsilonEquals(new Vector3D(expectedTranslation, 0.0, 0.0), epsilon));

      // test snapping on left edge
      snapData = snapper.snapFootstepNode(1, squareCellHalfWidth);
      translationVector = snapData.getSnapTransform().getTranslationVector();
      Assertions.assertTrue(translationVector.epsilonEquals(new Vector3D(0.0, -expectedTranslation, 0.0), epsilon));

      // test snapping on right edge
      snapData = snapper.snapFootstepNode(2, -squareCellHalfWidth);
      translationVector = snapData.getSnapTransform().getTranslationVector();
      Assertions.assertTrue(translationVector.epsilonEquals(new Vector3D(0.0, expectedTranslation, 0.0), epsilon));

      // test snapping on right edge
      snapData = snapper.snapFootstepNode(2, -squareCellHalfWidth);
      translationVector = snapData.getSnapTransform().getTranslationVector();
      Assertions.assertTrue(translationVector.epsilonEquals(new Vector3D(0.0, expectedTranslation, 0.0), epsilon));

      // test snapping on front-left corner
      snapData = snapper.snapFootstepNode(squareCellHalfWidth, squareCellHalfWidth);
      translationVector = snapData.getSnapTransform().getTranslationVector();
      Assertions.assertTrue(translationVector.epsilonEquals(new Vector3D(-expectedTranslation, -expectedTranslation, 0.0), epsilon));

      // test snapping on front-right corner
      snapData = snapper.snapFootstepNode(squareCellHalfWidth, -squareCellHalfWidth);
      translationVector = snapData.getSnapTransform().getTranslationVector();
      Assertions.assertTrue(translationVector.epsilonEquals(new Vector3D(-expectedTranslation, expectedTranslation, 0.0), epsilon));

      // test snapping on back-left corner
      snapData = snapper.snapFootstepNode(-squareCellHalfWidth, squareCellHalfWidth);
      translationVector = snapData.getSnapTransform().getTranslationVector();
      Assertions.assertTrue(translationVector.epsilonEquals(new Vector3D(expectedTranslation, -expectedTranslation, 0.0), epsilon));

      // test snapping on back-right corner
      snapData = snapper.snapFootstepNode(-squareCellHalfWidth, -squareCellHalfWidth);
      translationVector = snapData.getSnapTransform().getTranslationVector();
      Assertions.assertTrue(translationVector.epsilonEquals(new Vector3D(expectedTranslation, expectedTranslation, 0.0), epsilon));
   }

   @Test
   public void testProjectingIntoRolledSquare()
   {
      double epsilon = 1e-6;
      double rollAngle = Math.toRadians(30.0);
      int squareCellPlanarHalfWidth = 10;
      double extraSquareWidth = 0.001;
      double squareWidth = 2.0 * extraSquareWidth + 2.0 * (squareCellPlanarHalfWidth * FootstepNode.gridSizeXY) / Math.cos(rollAngle);

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.rotate(rollAngle, Axis.X);
      generator.addRectangle(squareWidth, squareWidth);
      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();

      SimplePlanarRegionFootstepNodeSnapper snapper = new SimplePlanarRegionFootstepNodeSnapper();
      snapper.setPlanarRegions(planarRegionsList);

      double projectionDistance = new DefaultFootstepPlannerParameters().getProjectInsideDistance();
      double expectedTranslation = (projectionDistance - extraSquareWidth) * Math.cos(rollAngle);

      PlanarRegion region = planarRegionsList.getPlanarRegion(0);

      // test snapping on left edge
      int xIndex = 1;
      int yIndex = squareCellPlanarHalfWidth;
      double x = xIndex * FootstepNode.gridSizeXY;
      double y = yIndex * FootstepNode.gridSizeXY;

      FootstepNodeSnapData snapData = snapper.snapFootstepNode(xIndex, yIndex);
      Vector3DReadOnly translationVector = snapData.getSnapTransform().getTranslationVector();
      double regionHeight = region.getPlaneZGivenXY(x + translationVector.getX(), y + translationVector.getY());
      Assertions.assertTrue(translationVector.epsilonEquals(new Vector3D(0.0, - expectedTranslation, regionHeight), epsilon));

      // test snapping on left edge
      xIndex = 3;
      yIndex = - squareCellPlanarHalfWidth;
      x = xIndex * FootstepNode.gridSizeXY;
      y = yIndex * FootstepNode.gridSizeXY;

      snapData = snapper.snapFootstepNode(xIndex, yIndex);
      translationVector = snapData.getSnapTransform().getTranslationVector();
      regionHeight = region.getPlaneZGivenXY(x + translationVector.getX(), y + translationVector.getY());
      Assertions.assertTrue(translationVector.epsilonEquals(new Vector3D(0.0, expectedTranslation, regionHeight), epsilon));
   }

   @Test
   public void testProjectingIntoPitchedSquare()
   {
      double epsilon = 1e-6;
      double pitchAngle = Math.toRadians(30.0);
      int squareCellPlanarHalfWidth = 10;
      double extraSquareWidth = 0.001;
      double squareWidth = 2.0 * extraSquareWidth + 2.0 * (squareCellPlanarHalfWidth * FootstepNode.gridSizeXY) / Math.cos(pitchAngle);

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.rotate(pitchAngle, Axis.Y);
      generator.addRectangle(squareWidth, squareWidth);
      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();

      SimplePlanarRegionFootstepNodeSnapper snapper = new SimplePlanarRegionFootstepNodeSnapper();
      snapper.setPlanarRegions(planarRegionsList);

      double projectionDistance = new DefaultFootstepPlannerParameters().getProjectInsideDistance();
      double expectedTranslation = (projectionDistance - extraSquareWidth) * Math.cos(pitchAngle);

      PlanarRegion region = planarRegionsList.getPlanarRegion(0);

      // test snapping on front edge
      int xIndex = squareCellPlanarHalfWidth;
      int yIndex = -2;
      double x = xIndex * FootstepNode.gridSizeXY;
      double y = yIndex * FootstepNode.gridSizeXY;

      FootstepNodeSnapData snapData = snapper.snapFootstepNode(xIndex, yIndex);
      Vector3DReadOnly translationVector = snapData.getSnapTransform().getTranslationVector();
      double regionHeight = region.getPlaneZGivenXY(x + translationVector.getX(), y + translationVector.getY());
      Assertions.assertTrue(translationVector.epsilonEquals(new Vector3D(- expectedTranslation, 0.0, regionHeight), epsilon));

      // test snapping on back edge
      xIndex = - squareCellPlanarHalfWidth;
      yIndex = 0;
      x = xIndex * FootstepNode.gridSizeXY;
      y = yIndex * FootstepNode.gridSizeXY;

      snapData = snapper.snapFootstepNode(xIndex, yIndex);
      translationVector = snapData.getSnapTransform().getTranslationVector();
      regionHeight = region.getPlaneZGivenXY(x + translationVector.getX(), y + translationVector.getY());
      Assertions.assertTrue(translationVector.epsilonEquals(new Vector3D(expectedTranslation, 0.0, regionHeight), epsilon));
   }

   private class TestParameters extends DefaultFootstepPlannerParameters
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