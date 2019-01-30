package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeExpansion;

import org.junit.Test;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettings;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.graphics.Graphics3DObjectTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

import java.util.HashSet;
import java.util.List;

import static junit.framework.TestCase.fail;

public class ParameterBasedNodeExpansionTest
{
   private static final boolean visualize = true;
   private static final QuadrantDependentList<AppearanceDefinition> colorDefinitions = new QuadrantDependentList<>(YoAppearance.Red(), YoAppearance.Green(),
                                                                                                                   YoAppearance.DarkRed(),
                                                                                                                   YoAppearance.DarkGreen());

   @Test(timeout = 300000)
   public void testExpandNodeWithBaseAtOrigin()
   {
      FootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();
      QuadrupedXGaitSettingsReadOnly xGaitSettingsReadOnly = new QuadrupedXGaitSettings();
      FootstepNodeExpansion expansion = new ParameterBasedNodeExpansion(parameters, xGaitSettingsReadOnly);

      FootstepNode baseNode = new FootstepNode(RobotQuadrant.FRONT_LEFT, 0.5, 0.25, 0.5, -0.25, -0.5, 0.25, -0.5, -0.25);

      HashSet<FootstepNode> expandedNodes = expansion.expandNode(baseNode);

      if (visualize)
         visualizeNodes(expandedNodes, baseNode);

      fail();
   }

   @Test(timeout = 300000)
   public void testExpandNodeWithTranslatedAndRotated()
   {
      FootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();
      QuadrupedXGaitSettingsReadOnly xGaitSettingsReadOnly = new QuadrupedXGaitSettings();
      FootstepNodeExpansion expansion = new ParameterBasedNodeExpansion(parameters, xGaitSettingsReadOnly);

      PoseReferenceFrame centerFrame = new PoseReferenceFrame("centerFrame", ReferenceFrame.getWorldFrame());
      FramePoint2D frontLeft = new FramePoint2D(centerFrame, 0.5, 0.25);
      FramePoint2D frontRight = new FramePoint2D(centerFrame, 0.5, -0.25);
      FramePoint2D hindLeft = new FramePoint2D(centerFrame, -0.5, 0.25);
      FramePoint2D hindRight = new FramePoint2D(centerFrame, -0.5, -0.25);

      FramePose3D poseInWorld = new FramePose3D();
      poseInWorld.getPosition().set(1.1, -0.5, 0.0);
      poseInWorld.getOrientation().setToYawQuaternion(Math.PI / 4.0);
      centerFrame.setPoseAndUpdate(poseInWorld);

      frontLeft.changeFrame(ReferenceFrame.getWorldFrame());
      frontRight.changeFrame(ReferenceFrame.getWorldFrame());
      hindLeft.changeFrame(ReferenceFrame.getWorldFrame());
      hindRight.changeFrame(ReferenceFrame.getWorldFrame());

      FootstepNode baseNode = new FootstepNode(RobotQuadrant.FRONT_LEFT, frontLeft, frontRight, hindLeft, hindRight);

      HashSet<FootstepNode> expandedNodes = expansion.expandNode(baseNode);

      if (visualize)
         visualizeNodes(expandedNodes, baseNode);

      fail();
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
