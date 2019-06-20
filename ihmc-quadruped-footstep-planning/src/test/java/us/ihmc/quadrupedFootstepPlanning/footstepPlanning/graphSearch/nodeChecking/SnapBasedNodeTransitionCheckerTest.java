package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeChecking;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.QuadrupedFootstepPlannerNodeRejectionReason;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping.SimplePlanarRegionFootstepNodeSnapper;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.listeners.QuadrupedFootstepPlannerListener;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.List;
import java.util.Random;
import java.util.concurrent.atomic.AtomicReference;

import static org.junit.jupiter.api.Assertions.*;

public class SnapBasedNodeTransitionCheckerTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @Test
   public void testStepInPlace()
   {
      FootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters()
      {
         @Override
         public double getMinimumFrontStepLength()
         {
            return -0.3;
         }

         @Override
         public double getProjectInsideDistanceForExpansion()
         {
            return 0.0;
         }
      };
      SimplePlanarRegionFootstepNodeSnapper snapper = new SimplePlanarRegionFootstepNodeSnapper(parameters, parameters::getProjectInsideDistanceForExpansion,
                                                                                                parameters::getProjectInsideUsingConvexHullDuringExpansion,
                                                                                                true);
      SnapBasedNodeTransitionChecker nodeChecker = new SnapBasedNodeTransitionChecker(parameters, snapper);

      TestListener testListener = new TestListener();
      nodeChecker.addPlannerListener(testListener);

      PoseReferenceFrame footstepFrame = new PoseReferenceFrame("footstepFrame", worldFrame);
      footstepFrame.setPoseAndUpdate(new Point3D(0.6, 0.9, 0.0), new AxisAngle(-Math.PI / 4, 0.0, 0.0));

      // have to set this really small, so that we don't get a ton of rounding issues
      FootstepNode.gridSizeXY = 0.001;
      RobotQuadrant robotQuadrant = RobotQuadrant.FRONT_LEFT;

      FramePoint2D frontLeft = new FramePoint2D(footstepFrame, 1.57, 0.67);
      FramePoint2D frontRight = new FramePoint2D(footstepFrame, 1.63, 0.12);
      FramePoint2D hindLeft = new FramePoint2D(footstepFrame, 0.67, 0.72);
      FramePoint2D otherHindLeft = new FramePoint2D(footstepFrame, 0.69, 0.72);
      FramePoint2D hindRight = new FramePoint2D(footstepFrame, 0.73, 0.05);

      frontLeft.changeFrame(worldFrame);
      frontRight.changeFrame(worldFrame);
      hindLeft.changeFrame(worldFrame);
      otherHindLeft.changeFrame(worldFrame);
      hindRight.changeFrame(worldFrame);

      double yaw = FootstepNode
            .computeNominalYaw(frontLeft.getX(), frontLeft.getY(), frontRight.getX(), frontRight.getY(), hindLeft.getX(), hindLeft.getY(), hindRight.getX(),
                               hindRight.getY());
      // check stepping on exactly the same spot
      FootstepNode previousNode = new FootstepNode(robotQuadrant, frontLeft, frontRight, otherHindLeft, hindRight, yaw, 1.0, 0.5);
      FootstepNode node = new FootstepNode(robotQuadrant, frontLeft, frontRight, hindLeft, hindRight, yaw, 1.0, 0.5);

      PoseReferenceFrame nodeFrame = new PoseReferenceFrame("nodeFrame", worldFrame);
      nodeFrame.setPoseAndUpdate(new Point3D(previousNode.getOrComputeXGaitCenterPoint().getX(), previousNode.getOrComputeXGaitCenterPoint().getY(), 0.0),
                                 new AxisAngle(yaw, 0.0, 0.0));

      String message = "Stepping from " + previousNode + " to " + node;

      //      assertFalse(nodeChecking.isNodeValid(node, previousNode));
      //      testListener.assertCorrectRejection(message, node, previousNode, QuadrupedFootstepPlannerNodeRejectionReason.STEP_IN_PLACE);

      FrameVector2D clearanceVector = new FrameVector2D(nodeFrame, parameters.getMinXClearanceFromFoot(), parameters.getMinYClearanceFromFoot());
      clearanceVector.changeFrame(worldFrame);

      // check stepping on not quite the same, but well within the clearance vector
      Random random = new Random(1738L);
      for (int iter = 0; iter < 50; iter++)
      {
         Vector2D offsetVector = new Vector2D(clearanceVector);
         double scaleFactor = RandomNumbers.nextDouble(random, 0.0, 0.8);
         offsetVector.scale(scaleFactor);
         if (RandomNumbers.nextBoolean(random, 0.5))
            offsetVector.setX(-offsetVector.getX());
         if (RandomNumbers.nextBoolean(random, 0.5))
            offsetVector.setY(-offsetVector.getY());

         Point2D shiftedFrontLeft = new Point2D(frontLeft);
         shiftedFrontLeft.add(offsetVector);

         FootstepNode newNode = new FootstepNode(robotQuadrant, shiftedFrontLeft, frontRight, hindLeft, hindRight, yaw, 1.0, 0.5);

         if (newNode.getXIndex(robotQuadrant) == previousNode.getXIndex(robotQuadrant) && newNode.getYIndex(robotQuadrant) == previousNode
               .getYIndex(robotQuadrant))
            continue;

         message = "iter = " + iter + ". Stepping from " + previousNode + "\nTo " + newNode + "\n, clearance amount in the moving foot is only " + offsetVector
               + "\n clearance required is " + clearanceVector;
         assertFalse(nodeChecker.isNodeValid(newNode, previousNode), message);
         testListener.assertCorrectRejection(message, newNode, previousNode, QuadrupedFootstepPlannerNodeRejectionReason.STEP_IN_PLACE);
      }

      // check stepping near the edge of the clearance vector
      for (int iter = 0; iter < 10; iter++)
      {
         FrameVector2D offsetVector = new FrameVector2D(clearanceVector);
         double scaleFactor = RandomNumbers.nextDouble(random, 0.8, 0.95);
         offsetVector.scale(scaleFactor);
         if (RandomNumbers.nextBoolean(random, 0.5))
            offsetVector.setX(-offsetVector.getX());
         if (RandomNumbers.nextBoolean(random, 0.5))
            offsetVector.setY(-offsetVector.getY());

         FramePoint2D shiftedFrontLeft = new FramePoint2D(frontLeft);
         shiftedFrontLeft.add(offsetVector);

         FootstepNode newNode = new FootstepNode(robotQuadrant, shiftedFrontLeft, frontRight, hindLeft, hindRight, yaw, 1.0, 0.5);

         offsetVector.changeFrame(nodeFrame);
         message = "Stepping from " + previousNode + "\n To " + newNode + "\n, clearance amount in the moving foot is only " + offsetVector + "\n";

         assertFalse(nodeChecker.isNodeValid(newNode, previousNode));
         testListener.assertCorrectRejection(message, newNode, previousNode, QuadrupedFootstepPlannerNodeRejectionReason.STEP_IN_PLACE);
      }

      // check stepping with enough clearance
      for (int iter = 0; iter < 10; iter++)
      {
         frontLeft.changeFrame(worldFrame);
         frontRight.changeFrame(worldFrame);

         clearanceVector.changeFrame(nodeFrame);
         FrameVector2D offsetVector = new FrameVector2D(clearanceVector);
         double scaleFactor = RandomNumbers.nextDouble(random, 1.1, 1.15);
         offsetVector.scale(scaleFactor);
         if (RandomNumbers.nextBoolean(random, 0.5))
            offsetVector.setX(-offsetVector.getX());

         offsetVector.changeFrame(worldFrame);

         FramePoint2D shiftedFrontLeft = new FramePoint2D(frontLeft);
         shiftedFrontLeft.add(offsetVector);

         FootstepNode newNode = new FootstepNode(robotQuadrant, shiftedFrontLeft, frontRight, hindLeft, hindRight, yaw, 1.0, 0.5);

         frontLeft.changeFrame(nodeFrame);
         shiftedFrontLeft.changeFrame(nodeFrame);
         frontRight.changeFrame(nodeFrame);
         clearanceVector.changeFrame(nodeFrame);
         offsetVector.changeFrame(nodeFrame);

         message = "Stepping from " + previousNode + "\n To " + newNode + "\n";
         message += "Front left = " + frontLeft + "\nShifted front left = " + shiftedFrontLeft + "\nFront right = " + frontRight + "\nOffset = " + offsetVector
               + "\nRequired clearance = " + clearanceVector + "\n";

         boolean isValid = nodeChecker.isNodeValid(newNode, previousNode);
         testListener.assertCorrectRejection(message, null, null, null);
         assertTrue(isValid);
      }
   }

   @Test
   public void testStepTooFarInwardFrontLeft()
   {
      Random random = new Random(1738L);

      double stanceLength = 1.0;
      double stanceWidth = 0.5;
      FootstepPlannerParameters parameters = new TestParameters();
      SimplePlanarRegionFootstepNodeSnapper snapper = new SimplePlanarRegionFootstepNodeSnapper(parameters, parameters::getProjectInsideDistanceForExpansion,
                                                                                                parameters::getProjectInsideUsingConvexHullDuringExpansion,
                                                                                                true);
      FootstepNodeTransitionChecker nodeChecker = new SnapBasedNodeTransitionChecker(parameters, snapper);

      TestListener testListener = new TestListener();
      nodeChecker.addPlannerListener(testListener);

      double stepYaw = -Math.PI / 4.0;
      PoseReferenceFrame previousNodeXGaitFrame = new PoseReferenceFrame("nodeFrame", worldFrame);
      previousNodeXGaitFrame.setPoseAndUpdate(new Point3D(0.6, 0.9, 0.0), new AxisAngle(stepYaw, 0.0, 0.0));

      // have to set this really small, so that we don't get a ton of rounding issues
      FootstepNode.gridSizeXY = 0.001;
      RobotQuadrant robotQuadrant = RobotQuadrant.FRONT_LEFT;

      FramePoint2D frontLeft = new FramePoint2D(previousNodeXGaitFrame, 0.57, 0.27);
      FramePoint2D frontRight = new FramePoint2D(previousNodeXGaitFrame, 0.63, -0.42);
      FramePoint2D hindLeft = new FramePoint2D(previousNodeXGaitFrame, -0.67, 0.32);
      FramePoint2D hindRight = new FramePoint2D(previousNodeXGaitFrame, -0.73, -0.35); // way further out

      frontLeft.changeFrame(worldFrame);
      frontRight.changeFrame(worldFrame);
      hindLeft.changeFrame(worldFrame);
      hindRight.changeFrame(worldFrame);


      FootstepNode previousNode = new FootstepNode(robotQuadrant, frontLeft, frontRight, hindLeft, hindRight, stepYaw, stanceLength, stanceWidth);

      previousNodeXGaitFrame.setPoseAndUpdate(new Point3D(previousNode.getOrComputeXGaitCenterPoint()), previousNode.getStepOrientation());

      FramePoint2D nominalFrontLeft = new FramePoint2D(previousNodeXGaitFrame, 0.5 * previousNode.getNominalStanceLength(),
                                                       0.5 * previousNode.getNominalStanceWidth());

      // check stepping within the reach
      for (int iter = 0; iter < 50; iter++)
      {
         FrameVector2D offsetVector = new FrameVector2D(previousNodeXGaitFrame, 0.0, parameters.getMaximumStepInward());
         double scaleFactor = RandomNumbers.nextDouble(random, 0.05, 0.95);
         offsetVector.scale(scaleFactor);

         FramePoint2D shiftedFrontLeft = new FramePoint2D(nominalFrontLeft);
         shiftedFrontLeft.add(offsetVector);
         shiftedFrontLeft.changeFrameAndProjectToXYPlane(worldFrame);

         FootstepNode newNode = new FootstepNode(robotQuadrant, shiftedFrontLeft, frontRight, hindLeft, hindRight, stepYaw, stanceLength, stanceWidth);

         frontLeft.changeFrame(previousNodeXGaitFrame);
         shiftedFrontLeft.changeFrame(previousNodeXGaitFrame);
         offsetVector.changeFrame(previousNodeXGaitFrame);
         String message = "\nIteration " + iter + ".\nStepping from " + previousNode + " to " + newNode + "\n";
         boolean isValid = nodeChecker.isNodeValid(newNode, previousNode);
         testListener.assertCorrectRejection(message, null, null, null);
         assertTrue(isValid);
      }

      // check stepping around reach max
      for (int iter = 0; iter < 50; iter++)
      {
         FrameVector2D offsetVector = new FrameVector2D(previousNodeXGaitFrame, 0.0, parameters.getMaximumStepInward());
         double scaleFactor = RandomNumbers.nextDouble(random, 0.95, 0.99);
         offsetVector.scale(scaleFactor);

         FramePoint2D shiftedFrontLeft = new FramePoint2D(nominalFrontLeft);
         shiftedFrontLeft.add(offsetVector);
         shiftedFrontLeft.changeFrameAndProjectToXYPlane(worldFrame);

         FootstepNode newNode = new FootstepNode(robotQuadrant, shiftedFrontLeft, frontRight, hindLeft, hindRight, stepYaw, stanceLength, stanceWidth);

         frontLeft.changeFrame(previousNodeXGaitFrame);
         shiftedFrontLeft.changeFrame(previousNodeXGaitFrame);
         offsetVector.changeFrame(previousNodeXGaitFrame);
         String message = "\nIteration " + iter + ".\nStepping from " + previousNode + " to " + newNode + "\n";
         boolean isValid = nodeChecker.isNodeValid(newNode, previousNode);
         testListener.assertCorrectRejection(message, null, null, null);
         assertTrue(isValid);
      }

      // check stepping past the reach
      for (int iter = 0; iter < 50; iter++)
      {
         FrameVector2D offsetVector = new FrameVector2D(previousNodeXGaitFrame, 0.0, parameters.getMaximumStepInward());
         double scaleFactor = RandomNumbers.nextDouble(random, 1.01, 1.2);
         offsetVector.scale(scaleFactor);

         FramePoint2D shiftedFrontLeft = new FramePoint2D(nominalFrontLeft);
         shiftedFrontLeft.add(offsetVector);
         shiftedFrontLeft.changeFrameAndProjectToXYPlane(worldFrame);

         FootstepNode newNode = new FootstepNode(robotQuadrant, shiftedFrontLeft, frontRight, hindLeft, hindRight, stepYaw, stanceLength, stanceWidth);

         frontLeft.changeFrame(previousNodeXGaitFrame);
         shiftedFrontLeft.changeFrame(previousNodeXGaitFrame);
         offsetVector.changeFrame(previousNodeXGaitFrame);
         String message = "\nIteration " + iter + ".\nStepping from " + previousNode + " to " + newNode + "\n";
         boolean isValid = nodeChecker.isNodeValid(newNode, previousNode);
         testListener.assertCorrectRejection(message, newNode, previousNode, QuadrupedFootstepPlannerNodeRejectionReason.STEP_TOO_FAR_RIGHT);
         assertFalse(isValid);
      }
   }

   @Test
   public void testStepTooFarInwardHindRight()
   {
      double stanceLength = 1.0;
      double stanceWidth = 0.5;
      FootstepPlannerParameters parameters = new TestParameters();
      SimplePlanarRegionFootstepNodeSnapper snapper = new SimplePlanarRegionFootstepNodeSnapper(parameters, parameters::getProjectInsideDistanceForExpansion,
                                                                                                parameters::getProjectInsideUsingConvexHullDuringExpansion,
                                                                                                true);
      FootstepNodeTransitionChecker nodeChecker = new SnapBasedNodeTransitionChecker(parameters, snapper);

      TestListener testListener = new TestListener();
      nodeChecker.addPlannerListener(testListener);

      double stepYaw = -Math.PI / 4.0;
      PoseReferenceFrame previousNodeXGaitFrame = new PoseReferenceFrame("nodeFrame", worldFrame);
      previousNodeXGaitFrame.setPoseAndUpdate(new Point3D(0.6, 0.9, 0.0), new AxisAngle(stepYaw, 0.0, 0.0));

      // have to set this really small, so that we don't get a ton of rounding issues
      FootstepNode.gridSizeXY = 0.001;
      RobotQuadrant robotQuadrant = RobotQuadrant.HIND_RIGHT;

      FramePoint2D frontLeft = new FramePoint2D(previousNodeXGaitFrame, 1.57, 0.67);
      FramePoint2D frontRight = new FramePoint2D(previousNodeXGaitFrame, 1.63, 0.12);
      FramePoint2D hindLeft = new FramePoint2D(previousNodeXGaitFrame, 0.67, 0.72);
      FramePoint2D hindRight = new FramePoint2D(previousNodeXGaitFrame, 0.73, 0.05);

      frontLeft.changeFrame(worldFrame);
      frontRight.changeFrame(worldFrame);
      hindLeft.changeFrame(worldFrame);
      hindRight.changeFrame(worldFrame);

      FootstepNode previousNode = new FootstepNode(robotQuadrant, frontLeft, frontRight, hindLeft, hindRight, stepYaw, stanceLength, stanceWidth);

      previousNodeXGaitFrame.setPoseAndUpdate(new Point3D(previousNode.getOrComputeXGaitCenterPoint()), previousNode.getStepOrientation());

      FramePoint2D nominalFrontLeft = new FramePoint2D(previousNodeXGaitFrame, -0.5 * previousNode.getNominalStanceLength(),
                                                       -0.5 * previousNode.getNominalStanceWidth());
      // check stepping within the reach
      Random random = new Random(1738L);
      for (int iter = 0; iter < 50; iter++)
      {
         FrameVector2D offsetVector = new FrameVector2D(previousNodeXGaitFrame, 0.0, -parameters.getMaximumStepInward());
         double scaleFactor = RandomNumbers.nextDouble(random, 0.05, 0.95);
         offsetVector.scale(scaleFactor);

         FramePoint2D shiftedHindRight = new FramePoint2D(nominalFrontLeft);
         shiftedHindRight.add(offsetVector);
         shiftedHindRight.changeFrameAndProjectToXYPlane(worldFrame);

         FootstepNode newNode = new FootstepNode(robotQuadrant, frontLeft, frontRight, hindLeft, shiftedHindRight, stepYaw, stanceLength, stanceWidth);

         hindRight.changeFrame(previousNodeXGaitFrame);
         shiftedHindRight.changeFrame(previousNodeXGaitFrame);
         offsetVector.changeFrame(previousNodeXGaitFrame);
         String message = "Stepping from " + previousNode + " to " + newNode + "\n";
         boolean isValid = nodeChecker.isNodeValid(newNode, previousNode);
         testListener.assertCorrectRejection(message, null, null, null);
         assertTrue(isValid);
      }

      // check stepping around reach max
      for (int iter = 0; iter < 50; iter++)
      {
         FrameVector2D offsetVector = new FrameVector2D(previousNodeXGaitFrame, 0.0, -parameters.getMaximumStepInward());
         double scaleFactor = RandomNumbers.nextDouble(random, 0.95, 0.99);
         offsetVector.scale(scaleFactor);

         FramePoint2D shiftedHindRight = new FramePoint2D(nominalFrontLeft);
         shiftedHindRight.add(offsetVector);
         shiftedHindRight.changeFrameAndProjectToXYPlane(worldFrame);

         FootstepNode newNode = new FootstepNode(robotQuadrant, frontLeft, frontRight, hindLeft, shiftedHindRight, stepYaw, stanceLength, stanceWidth);

         hindRight.changeFrame(previousNodeXGaitFrame);
         shiftedHindRight.changeFrame(previousNodeXGaitFrame);
         offsetVector.changeFrame(previousNodeXGaitFrame);
         String message = "Stepping from " + previousNode + " to " + newNode + "\n";
         boolean isValid = nodeChecker.isNodeValid(newNode, previousNode);
         testListener.assertCorrectRejection(message, null, null, null);
         assertTrue(isValid);
      }

      // check stepping past the reach
      for (int iter = 0; iter < 50; iter++)
      {
         FrameVector2D offsetVector = new FrameVector2D(previousNodeXGaitFrame, 0.0, -parameters.getMaximumStepInward());
         double scaleFactor = RandomNumbers.nextDouble(random, 1.01, 1.2);
         offsetVector.scale(scaleFactor);

         FramePoint2D shiftedHindRight = new FramePoint2D(nominalFrontLeft);
         shiftedHindRight.add(offsetVector);
         shiftedHindRight.changeFrameAndProjectToXYPlane(worldFrame);

         FootstepNode newNode = new FootstepNode(robotQuadrant, frontLeft, frontRight, hindLeft, shiftedHindRight, stepYaw, stanceLength, stanceWidth);


         hindRight.changeFrame(previousNodeXGaitFrame);
         shiftedHindRight.changeFrame(previousNodeXGaitFrame);
         offsetVector.changeFrame(previousNodeXGaitFrame);
         String message = "Stepping from " + previousNode + " to " + newNode + "\n";
         boolean isValid = nodeChecker.isNodeValid(newNode, previousNode);
         testListener.assertCorrectRejection(message, newNode, previousNode, QuadrupedFootstepPlannerNodeRejectionReason.STEP_TOO_FAR_LEFT);
         assertFalse(isValid);
      }
   }

   @Test
   public void testStepTooFarForwardFrontLeft()
   {
      Random random = new Random(1738L);

      double stanceLength = 1.0;
      double stanceWidth = 0.5;
      FootstepPlannerParameters parameters = new TestParameters();
      SimplePlanarRegionFootstepNodeSnapper snapper = new SimplePlanarRegionFootstepNodeSnapper(parameters, parameters::getProjectInsideDistanceForExpansion,
                                                                                                parameters::getProjectInsideUsingConvexHullDuringExpansion,
                                                                                                true);
      FootstepNodeTransitionChecker nodeChecker = new SnapBasedNodeTransitionChecker(parameters, snapper);

      TestListener testListener = new TestListener();
      nodeChecker.addPlannerListener(testListener);

      double stepYaw = -Math.PI / 4.0;
      PoseReferenceFrame previousNodeXGaitFrame = new PoseReferenceFrame("nodeFrame", worldFrame);
      previousNodeXGaitFrame.setPoseAndUpdate(new Point3D(0.6, 0.9, 0.0), new AxisAngle(stepYaw, 0.0, 0.0));

      // have to set this really small, so that we don't get a ton of rounding issues
      FootstepNode.gridSizeXY = 0.001;
      RobotQuadrant robotQuadrant = RobotQuadrant.FRONT_LEFT;

      FramePoint2D frontLeft = new FramePoint2D(previousNodeXGaitFrame, 0.57, 0.27);
      FramePoint2D frontRight = new FramePoint2D(previousNodeXGaitFrame, 0.63, -0.42);
      FramePoint2D hindLeft = new FramePoint2D(previousNodeXGaitFrame, -0.67, 0.32);
      FramePoint2D hindRight = new FramePoint2D(previousNodeXGaitFrame, -0.73, -0.35); // way further out

      frontLeft.changeFrame(worldFrame);
      frontRight.changeFrame(worldFrame);
      hindLeft.changeFrame(worldFrame);
      hindRight.changeFrame(worldFrame);


      FootstepNode previousNode = new FootstepNode(robotQuadrant, frontLeft, frontRight, hindLeft, hindRight, stepYaw, stanceLength, stanceWidth);

      previousNodeXGaitFrame.setPoseAndUpdate(new Point3D(previousNode.getOrComputeXGaitCenterPoint()), previousNode.getStepOrientation());

      FramePoint2D nominalFrontLeft = new FramePoint2D(previousNodeXGaitFrame, 0.5 * previousNode.getNominalStanceLength(),
                                                       0.5 * previousNode.getNominalStanceWidth());

      // check stepping within the reach
      for (int iter = 0; iter < 50; iter++)
      {
         FrameVector2D offsetVector = new FrameVector2D(previousNodeXGaitFrame, parameters.getMaximumFrontStepLength(), 0.0);
         double scaleFactor = RandomNumbers.nextDouble(random, 0.05, 0.95);
         offsetVector.scale(scaleFactor);

         FramePoint2D shiftedFrontLeft = new FramePoint2D(nominalFrontLeft);
         shiftedFrontLeft.add(offsetVector);
         shiftedFrontLeft.changeFrameAndProjectToXYPlane(worldFrame);

         FootstepNode newNode = new FootstepNode(robotQuadrant, shiftedFrontLeft, frontRight, hindLeft, hindRight, stepYaw, stanceLength, stanceWidth);

         frontLeft.changeFrame(previousNodeXGaitFrame);
         shiftedFrontLeft.changeFrame(previousNodeXGaitFrame);
         offsetVector.changeFrame(previousNodeXGaitFrame);
         String message = "\nIteration " + iter + ".\nStepping from " + previousNode + " to " + newNode + "\n";
         boolean isValid = nodeChecker.isNodeValid(newNode, previousNode);
         testListener.assertCorrectRejection(message, null, null, null);
         assertTrue(isValid);
      }

      // check stepping around reach max
      for (int iter = 0; iter < 50; iter++)
      {
         FrameVector2D offsetVector = new FrameVector2D(previousNodeXGaitFrame, parameters.getMaximumFrontStepLength(), 0.0);
         double scaleFactor = RandomNumbers.nextDouble(random, 0.95, 0.99);
         offsetVector.scale(scaleFactor);

         FramePoint2D shiftedFrontLeft = new FramePoint2D(nominalFrontLeft);
         shiftedFrontLeft.add(offsetVector);
         shiftedFrontLeft.changeFrameAndProjectToXYPlane(worldFrame);

         FootstepNode newNode = new FootstepNode(robotQuadrant, shiftedFrontLeft, frontRight, hindLeft, hindRight, stepYaw, stanceLength, stanceWidth);

         frontLeft.changeFrame(previousNodeXGaitFrame);
         shiftedFrontLeft.changeFrame(previousNodeXGaitFrame);
         offsetVector.changeFrame(previousNodeXGaitFrame);
         String message = "\nIteration " + iter + ".\nStepping from " + previousNode + " to " + newNode + "\n";
         boolean isValid = nodeChecker.isNodeValid(newNode, previousNode);
         testListener.assertCorrectRejection(message, null, null, null);
         assertTrue(isValid);
      }

      double maxScale = parameters.getMaximumFrontStepReach() / parameters.getMaximumFrontStepLength() - 1e-3;
      // check stepping past the reach
      for (int iter = 0; iter < 50; iter++)
      {
         FrameVector2D offsetVector = new FrameVector2D(previousNodeXGaitFrame, parameters.getMaximumFrontStepLength(), 0.0);
         double scaleFactor = RandomNumbers.nextDouble(random, 1.01, maxScale);
         offsetVector.scale(scaleFactor);

         FramePoint2D shiftedFrontLeft = new FramePoint2D(nominalFrontLeft);
         shiftedFrontLeft.add(offsetVector);
         shiftedFrontLeft.changeFrameAndProjectToXYPlane(worldFrame);

         FootstepNode newNode = new FootstepNode(robotQuadrant, shiftedFrontLeft, frontRight, hindLeft, hindRight, stepYaw, stanceLength, stanceWidth);

         frontLeft.changeFrame(previousNodeXGaitFrame);
         shiftedFrontLeft.changeFrame(previousNodeXGaitFrame);
         offsetVector.changeFrame(previousNodeXGaitFrame);
         String message = "\nIteration " + iter + ".\nStepping from " + previousNode + " to " + newNode + "\n";
         boolean isValid = nodeChecker.isNodeValid(newNode, previousNode);
         testListener.assertCorrectRejection(message, newNode, previousNode, QuadrupedFootstepPlannerNodeRejectionReason.STEP_TOO_FAR_FORWARD);
         assertFalse(isValid);
      }
   }

   @Test
   public void testStepTooFarBackwardFrontLeft()
   {
      Random random = new Random(1738L);

      double stanceLength = 1.0;
      double stanceWidth = 0.5;
      FootstepPlannerParameters parameters = new TestParameters();
      SimplePlanarRegionFootstepNodeSnapper snapper = new SimplePlanarRegionFootstepNodeSnapper(parameters, parameters::getProjectInsideDistanceForExpansion,
                                                                                                parameters::getProjectInsideUsingConvexHullDuringExpansion,
                                                                                                true);
      FootstepNodeTransitionChecker nodeChecker = new SnapBasedNodeTransitionChecker(parameters, snapper);

      TestListener testListener = new TestListener();
      nodeChecker.addPlannerListener(testListener);

      double stepYaw = -Math.PI / 4.0;
      PoseReferenceFrame previousNodeXGaitFrame = new PoseReferenceFrame("nodeFrame", worldFrame);
      previousNodeXGaitFrame.setPoseAndUpdate(new Point3D(0.6, 0.9, 0.0), new AxisAngle(stepYaw, 0.0, 0.0));

      // have to set this really small, so that we don't get a ton of rounding issues
      FootstepNode.gridSizeXY = 0.001;
      RobotQuadrant robotQuadrant = RobotQuadrant.FRONT_LEFT;

      FramePoint2D frontLeft = new FramePoint2D(previousNodeXGaitFrame, 0.57, 0.27);
      FramePoint2D frontRight = new FramePoint2D(previousNodeXGaitFrame, 0.63, -0.42);
      FramePoint2D hindLeft = new FramePoint2D(previousNodeXGaitFrame, -0.67, 0.32);
      FramePoint2D hindRight = new FramePoint2D(previousNodeXGaitFrame, -0.73, -0.35); // way further out

      frontLeft.changeFrame(worldFrame);
      frontRight.changeFrame(worldFrame);
      hindLeft.changeFrame(worldFrame);
      hindRight.changeFrame(worldFrame);


      FootstepNode previousNode = new FootstepNode(robotQuadrant, frontLeft, frontRight, hindLeft, hindRight, stepYaw, stanceLength, stanceWidth);

      previousNodeXGaitFrame.setPoseAndUpdate(new Point3D(previousNode.getOrComputeXGaitCenterPoint()), previousNode.getStepOrientation());

      FramePoint2D nominalFrontLeft = new FramePoint2D(previousNodeXGaitFrame, 0.5 * previousNode.getNominalStanceLength(),
                                                       0.5 * previousNode.getNominalStanceWidth());

      // check stepping within the reach
      for (int iter = 0; iter < 50; iter++)
      {
         FrameVector2D offsetVector = new FrameVector2D(previousNodeXGaitFrame, parameters.getMinimumFrontStepLength(), 0.0);
         double scaleFactor = RandomNumbers.nextDouble(random, 0.05, 0.95);
         offsetVector.scale(scaleFactor);

         FramePoint2D shiftedFrontLeft = new FramePoint2D(nominalFrontLeft);
         shiftedFrontLeft.add(offsetVector);
         shiftedFrontLeft.changeFrameAndProjectToXYPlane(worldFrame);

         FootstepNode newNode = new FootstepNode(robotQuadrant, shiftedFrontLeft, frontRight, hindLeft, hindRight, stepYaw, stanceLength, stanceWidth);

         frontLeft.changeFrame(previousNodeXGaitFrame);
         shiftedFrontLeft.changeFrame(previousNodeXGaitFrame);
         offsetVector.changeFrame(previousNodeXGaitFrame);
         String message = "\nIteration " + iter + ".\nStepping from " + previousNode + " to " + newNode + "\n";
         boolean isValid = nodeChecker.isNodeValid(newNode, previousNode);
         testListener.assertCorrectRejection(message, null, null, null);
         assertTrue(isValid);
      }

      // check stepping around reach max
      for (int iter = 0; iter < 50; iter++)
      {
         FrameVector2D offsetVector = new FrameVector2D(previousNodeXGaitFrame, parameters.getMinimumFrontStepLength(), 0.0);
         double scaleFactor = RandomNumbers.nextDouble(random, 0.95, 0.99);
         offsetVector.scale(scaleFactor);

         FramePoint2D shiftedFrontLeft = new FramePoint2D(nominalFrontLeft);
         shiftedFrontLeft.add(offsetVector);
         shiftedFrontLeft.changeFrameAndProjectToXYPlane(worldFrame);

         FootstepNode newNode = new FootstepNode(robotQuadrant, shiftedFrontLeft, frontRight, hindLeft, hindRight, stepYaw, stanceLength, stanceWidth);

         frontLeft.changeFrame(previousNodeXGaitFrame);
         shiftedFrontLeft.changeFrame(previousNodeXGaitFrame);
         offsetVector.changeFrame(previousNodeXGaitFrame);
         String message = "\nIteration " + iter + ".\nStepping from " + previousNode + " to " + newNode + "\n";
         boolean isValid = nodeChecker.isNodeValid(newNode, previousNode);
         testListener.assertCorrectRejection(message, null, null, null);
         assertTrue(isValid);
      }

      double maxScale = parameters.getMaximumFrontStepReach() /  Math.abs(parameters.getMinimumFrontStepLength()) - 1e-3;
      // check stepping past the reach
      for (int iter = 0; iter < 50; iter++)
      {
         FrameVector2D offsetVector = new FrameVector2D(previousNodeXGaitFrame, parameters.getMinimumFrontStepLength(), 0.0);
         double scaleFactor = RandomNumbers.nextDouble(random, 1.01, maxScale);
         offsetVector.scale(scaleFactor);

         FramePoint2D shiftedFrontLeft = new FramePoint2D(nominalFrontLeft);
         shiftedFrontLeft.add(offsetVector);
         shiftedFrontLeft.changeFrameAndProjectToXYPlane(worldFrame);

         FootstepNode newNode = new FootstepNode(robotQuadrant, shiftedFrontLeft, frontRight, hindLeft, hindRight, stepYaw, stanceLength, stanceWidth);

         frontLeft.changeFrame(previousNodeXGaitFrame);
         shiftedFrontLeft.changeFrame(previousNodeXGaitFrame);
         offsetVector.changeFrame(previousNodeXGaitFrame);
         String message = "\nIteration " + iter + ".\nStepping from " + previousNode + " to " + newNode + "\n";
         boolean isValid = nodeChecker.isNodeValid(newNode, previousNode);
         testListener.assertCorrectRejection(message, newNode, previousNode, QuadrupedFootstepPlannerNodeRejectionReason.STEP_TOO_FAR_BACKWARD);
         assertFalse(isValid);
      }
   }


   private class TestListener implements QuadrupedFootstepPlannerListener
   {
      private final AtomicReference<QuadrupedFootstepPlannerNodeRejectionReason> reason = new AtomicReference<>();
      private final AtomicReference<FootstepNode> rejectedNode = new AtomicReference<>();
      private final AtomicReference<FootstepNode> rejectedParentNode = new AtomicReference<>();

      @Override
      public void addNode(FootstepNode node, FootstepNode previousNode)
      {

      }

      public void assertCorrectRejection(String message, FootstepNode node, FootstepNode parentNode,
                                         QuadrupedFootstepPlannerNodeRejectionReason rejectionReason)
      {
         assertEquals(rejectionReason, reason.getAndSet(null), message);
         assertEquals(node, rejectedNode.getAndSet(null), message);
         assertEquals(parentNode, rejectedParentNode.getAndSet(null), message);
      }

      @Override
      public void rejectNode(FootstepNode node, FootstepNode parentNode, QuadrupedFootstepPlannerNodeRejectionReason rejectionReason)
      {
         rejectedNode.set(node);
         rejectedParentNode.set(parentNode);
         reason.set(rejectionReason);
      }

      @Override
      public void rejectNode(FootstepNode node, QuadrupedFootstepPlannerNodeRejectionReason rejectionReason)
      {
         rejectedNode.set(node);
         reason.set(rejectionReason);
      }

      @Override
      public void plannerFinished(List<FootstepNode> plan)
      {

      }

      @Override
      public void reportLowestCostNodeList(List<FootstepNode> plan)
      {

      }

      @Override
      public void tickAndUpdate()
      {

      }
   }

   private class TestParameters extends DefaultFootstepPlannerParameters
   {
      @Override
      public double getMinXClearanceFromFoot()
      {
         return 0.0;
      }

      @Override
      public double getMinYClearanceFromFoot()
      {
         return 0.0;
      }

      @Override
      public double getMinimumFrontStepLength()
      {
         return -0.3;
      }

      @Override
      public double getProjectInsideDistanceForExpansion()
      {
         return 0.0;
      }
   }
}
