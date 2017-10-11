package us.ihmc.footstepPlanning.graphSearch.nodeExpansion;

import java.util.HashSet;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class SimpleSideBasedExpansion implements FootstepNodeExpansion
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final FootstepPlannerParameters parameters;

   public SimpleSideBasedExpansion(FootstepPlannerParameters parameters)
   {
      this.parameters = parameters;
   }

   @Override
   public HashSet<FootstepNode> expandNode(FootstepNode node)
   {
      double maxYaw = parameters.getMaximumStepYaw();
      double defaultStepWidth = parameters.getIdealFootstepWidth();
      double[] stepLengths = new double[] {0.0, FootstepNode.gridSizeXY, 0.1, 0.2, 0.4};
      double[] stepWidths = new double[] {0.15, defaultStepWidth - FootstepNode.gridSizeXY, defaultStepWidth + FootstepNode.gridSizeXY};
      double[] stepYaws = new double[] {0.0, FootstepNode.gridSizeYaw, maxYaw};

      HashSet<FootstepNode> neighbors = new HashSet<>();

      FramePose stanceFootPose = new FramePose(worldFrame);
      stanceFootPose.setYawPitchRoll(node.getYaw(), 0.0, 0.0);
      stanceFootPose.setX(node.getX());
      stanceFootPose.setY(node.getY());
      ReferenceFrame stanceFrame = new PoseReferenceFrame("stanceFrame", stanceFootPose);

      RobotSide stepSide = node.getRobotSide().getOppositeSide();
      double ySign = stepSide.negateIfRightSide(1.0);

      // walk forward and backward
      for (int i = 0; i < stepLengths.length; i++)
      {
         double stepLength = stepLengths[i];
         for (int j = 0; j < stepYaws.length; j++)
         {
            double yaw = stepYaws[j];

            FramePose forwardStep = new FramePose(stanceFrame);
            forwardStep.setX(stepLength);
            forwardStep.setY(ySign * defaultStepWidth);
            forwardStep.changeFrame(worldFrame);
            neighbors.add(new FootstepNode(forwardStep.getX(), forwardStep.getY(), node.getYaw() + ySign * yaw, stepSide));

            FramePose backwardStep = new FramePose(stanceFrame);
            backwardStep.setX(-stepLength);
            backwardStep.setY(ySign * defaultStepWidth);
            backwardStep.changeFrame(worldFrame);
            neighbors.add(new FootstepNode(backwardStep.getX(), backwardStep.getY(), node.getYaw() + ySign * yaw, stepSide));
         }
      }

      // side step
      for (int i = 0; i < stepWidths.length; i++)
      {
         double stepWidth = stepWidths[i];
         FramePose sideStep = new FramePose(stanceFrame);
         sideStep.setY(ySign * stepWidth);
         sideStep.changeFrame(worldFrame);
         neighbors.add(new FootstepNode(sideStep.getX(), sideStep.getY(), node.getYaw(), stepSide));
      }

      // turn in place
      FramePose turnStep = new FramePose(stanceFrame);
      turnStep.setY(ySign * defaultStepWidth * (1.0 + Math.cos(maxYaw)) / 2.0);
      turnStep.setX(-defaultStepWidth * Math.sin(maxYaw) / 2.0);
      turnStep.changeFrame(worldFrame);
      double yaw = node.getYaw() + ySign * maxYaw;
      neighbors.add(new FootstepNode(turnStep.getX(), turnStep.getY(), yaw, stepSide));

      return neighbors;
   }
}
