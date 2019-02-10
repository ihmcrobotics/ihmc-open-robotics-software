package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeExpansion;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.ThreadTools;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.FootstepPlanningRandomTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettings;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

import java.util.HashSet;
import java.util.Random;

import static us.ihmc.robotics.Assert.*;

import static us.ihmc.robotics.Assert.*;

public class ParameterBasedNodeExpansionTest
{
   private static final int iters = 1000;

   private static final double epsilon = 1e-7;

   private static final double stanceLength = 1.0;
   private static final double stanceWidth = 0.5;
   private static final boolean visualize = false;
   private static final QuadrantDependentList<AppearanceDefinition> colorDefinitions = new QuadrantDependentList<>(YoAppearance.Red(), YoAppearance.Green(),
                                                                                                                   YoAppearance.DarkRed(),
                                                                                                                   YoAppearance.DarkGreen());

   @Test
   public void testExpandNodeWithBaseAtOrigin()
   {
      FootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();
      QuadrupedXGaitSettings xGaitSettingsReadOnly = new QuadrupedXGaitSettings();
      xGaitSettingsReadOnly.setStanceWidth(stanceWidth);
      xGaitSettingsReadOnly.setStanceLength(stanceLength);

      FootstepNodeExpansion expansion = new ParameterBasedNodeExpansion(parameters, xGaitSettingsReadOnly);

      FootstepNode baseNode = new FootstepNode(RobotQuadrant.FRONT_LEFT, 0.5 * stanceLength, 0.5 * stanceWidth, 0.5 * stanceLength, -0.5 * stanceWidth,
                                               -0.5 * stanceLength, 0.5 * stanceWidth, -0.5 * stanceLength, -0.5 * stanceWidth, 0.0, stanceLength, stanceWidth);

      HashSet<FootstepNode> expandedNodes = expansion.expandNode(baseNode);

      double stepBoxLength = parameters.getMaximumStepReach() - parameters.getMinimumStepLength();
      double stepBoxWidth = parameters.getMaximumStepWidth() - parameters.getMinimumStepWidth();

      int numberWide = (int) (stepBoxWidth / FootstepNode.gridSizeXY);
      int numberLong = (int) (stepBoxLength / FootstepNode.gridSizeXY);

      if (visualize)
         visualizeNodes(expandedNodes, baseNode);

      assertEquals(numberLong * numberWide, expandedNodes.size());

      RobotQuadrant expectedNewQuadrant = RobotQuadrant.FRONT_LEFT.getNextRegularGaitSwingQuadrant();

      for (FootstepNode node : expandedNodes)
      {
         assertEquals(expectedNewQuadrant, node.getMovingQuadrant());

         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            if (expectedNewQuadrant == robotQuadrant)
            {
               assertTrue(MathTools.intervalContains(node.getX(robotQuadrant), baseNode.getX(robotQuadrant) - 0.5 * stepBoxLength,
                                                     baseNode.getX(robotQuadrant) + 0.5 * stepBoxLength, true, true));
               assertTrue(MathTools.intervalContains(node.getY(robotQuadrant), baseNode.getY(robotQuadrant) - 0.5 * stepBoxWidth,
                                                     baseNode.getY(robotQuadrant) + 0.5 * stepBoxWidth, true, true));
            }
            else
            {
               assertEquals(baseNode.getX(robotQuadrant), node.getX(robotQuadrant), epsilon);
               assertEquals(baseNode.getY(robotQuadrant), node.getY(robotQuadrant), epsilon);
            }
         }
      }
   }

   @Test
   public void testExpandNodeWithTranslatedAndRotated()
   {
      FootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();
      QuadrupedXGaitSettings xGaitSettingsReadOnly = new QuadrupedXGaitSettings();
      xGaitSettingsReadOnly.setStanceLength(stanceLength);
      xGaitSettingsReadOnly.setStanceWidth(stanceWidth);
      FootstepNodeExpansion expansion = new ParameterBasedNodeExpansion(parameters, xGaitSettingsReadOnly);

      PoseReferenceFrame centerFrame = new PoseReferenceFrame("centerFrame", ReferenceFrame.getWorldFrame());
      FramePoint2D frontLeft = new FramePoint2D(centerFrame, 0.5 * stanceLength, 0.5 * stanceWidth);
      FramePoint2D frontRight = new FramePoint2D(centerFrame, 0.5 * stanceLength, -0.5 * stanceWidth);
      FramePoint2D hindLeft = new FramePoint2D(centerFrame, -0.5 * stanceLength, 0.5 * stanceWidth);
      FramePoint2D hindRight = new FramePoint2D(centerFrame, -0.5 * stanceLength, -0.5 * stanceWidth);

      FramePose3D poseInWorld = new FramePose3D();
      poseInWorld.getPosition().set(1.1, -0.5, 0.0);
      poseInWorld.getOrientation().setToYawQuaternion(Math.PI / 4.0);
      centerFrame.setPoseAndUpdate(poseInWorld);

      frontLeft.changeFrame(ReferenceFrame.getWorldFrame());
      frontRight.changeFrame(ReferenceFrame.getWorldFrame());
      hindLeft.changeFrame(ReferenceFrame.getWorldFrame());
      hindRight.changeFrame(ReferenceFrame.getWorldFrame());

      FootstepNode baseNode = new FootstepNode(RobotQuadrant.FRONT_LEFT, frontLeft, frontRight, hindLeft, hindRight, Math.PI / 4.0, stanceLength, stanceWidth);

      HashSet<FootstepNode> expandedNodes = expansion.expandNode(baseNode);

      if (visualize)
         visualizeNodes(expandedNodes, baseNode);

      RobotQuadrant expectedNewQuadrant = RobotQuadrant.FRONT_LEFT.getNextRegularGaitSwingQuadrant();

      for (FootstepNode node : expandedNodes)
      {
         assertEquals(expectedNewQuadrant, node.getMovingQuadrant());

         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            if (expectedNewQuadrant == robotQuadrant)
               continue;

            assertEquals(baseNode.getX(robotQuadrant), node.getX(robotQuadrant), epsilon);
            assertEquals(baseNode.getY(robotQuadrant), node.getY(robotQuadrant), epsilon);
         }
      }
   }

   @Test
   public void testCheckNodeIsFarEnoughFromOtherFoot()
   {
      Random random = new Random(1738L);

      for (int iter = 0; iter < iters; iter++)
      {
         Vector2D requiredClearance = EuclidCoreRandomTools.nextVector2D(random);
         requiredClearance.setX(Math.abs(requiredClearance.getX()));
         requiredClearance.setY(Math.abs(requiredClearance.getY()));

         Point2DReadOnly previousNodePoint = EuclidCoreRandomTools.nextPoint2D(random);

         String message = "iter = " + iter + " failed.";

         // check right on the edge
         Vector2D offset = new Vector2D(requiredClearance);

         Point2D positionToCheck = new Point2D();
         positionToCheck.set(previousNodePoint);
         positionToCheck.add(offset);

         assertFalse(message, ParameterBasedNodeExpansion
               .checkNodeIsFarEnoughFromOtherFoot(positionToCheck, requiredClearance, previousNodePoint.getX(), previousNodePoint.getY()));

         positionToCheck.set(previousNodePoint);
         positionToCheck.sub(offset);

         assertFalse(message, ParameterBasedNodeExpansion
               .checkNodeIsFarEnoughFromOtherFoot(positionToCheck, requiredClearance, previousNodePoint.getX(), previousNodePoint.getY()));

         // check slightly within the edge
         offset.set(requiredClearance);
         offset.scale(0.9);

         positionToCheck.set(previousNodePoint);
         positionToCheck.add(offset);

         assertFalse(message, ParameterBasedNodeExpansion
               .checkNodeIsFarEnoughFromOtherFoot(positionToCheck, requiredClearance, previousNodePoint.getX(), previousNodePoint.getY()));

         positionToCheck.set(previousNodePoint);
         positionToCheck.sub(offset);

         assertFalse(message, ParameterBasedNodeExpansion
               .checkNodeIsFarEnoughFromOtherFoot(positionToCheck, requiredClearance, previousNodePoint.getX(), previousNodePoint.getY()));

         // check slightly outside the edge
         offset.set(requiredClearance);
         offset.scale(1.1);

         positionToCheck.set(previousNodePoint);
         positionToCheck.add(offset);

         assertTrue(message, ParameterBasedNodeExpansion
               .checkNodeIsFarEnoughFromOtherFoot(positionToCheck, requiredClearance, previousNodePoint.getX(), previousNodePoint.getY()));

         positionToCheck.set(previousNodePoint);
         positionToCheck.sub(offset);

         assertTrue(message, ParameterBasedNodeExpansion
               .checkNodeIsFarEnoughFromOtherFoot(positionToCheck, requiredClearance, previousNodePoint.getX(), previousNodePoint.getY()));
      }
   }


   private void visualizeNodes(HashSet<FootstepNode> neighboringNodes, FootstepNode baseNode)
   {
      if (!visualize || ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer())
         return;

      SimulationConstructionSet scs = new SimulationConstructionSet();

      Graphics3DObject graphics3DObject = new Graphics3DObject();
      scs.setGroundVisible(false);

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         graphics3DObject.identity();
         graphics3DObject.translate(baseNode.getX(robotQuadrant), baseNode.getY(robotQuadrant), 0.0);
         graphics3DObject.translate(0.0, 0.0, 0.05);
         graphics3DObject.addCone(0.3, 0.05, colorDefinitions.get(robotQuadrant));
      }

      for (FootstepNode neighboringNode : neighboringNodes)
      {
         RobotQuadrant quadrant = neighboringNode.getMovingQuadrant();
         Point3D point = new Point3D(neighboringNode.getX(quadrant), neighboringNode.getY(quadrant), 0.0);

         graphics3DObject.identity();
         graphics3DObject.translate(point);
         graphics3DObject.addSphere(0.01, YoAppearance.Orange());
      }

      scs.addStaticLinkGraphics(graphics3DObject);

      scs.setCameraFix(0.0, 0.0, 0.0);
      scs.setCameraPosition(-0.001, 0.0, 15.0);
      scs.startOnAThread();

      ThreadTools.sleepForever();
   }
}
