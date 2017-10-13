package us.ihmc.footstepPlanning.graphSearch.stepCost;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

public class DistanceAndYawBasedCost implements FootstepCost
{
   private final FootstepPlannerParameters parameters;

   public DistanceAndYawBasedCost(FootstepPlannerParameters parameters)
   {
      this.parameters = parameters;
   }

   @Override
   public double compute(FootstepNode startNode, FootstepNode endNode)
   {
      Point2D startPoint = computeMidFootPoint(startNode, parameters.getIdealFootstepWidth());
      Point2D endPoint = computeMidFootPoint(endNode, parameters.getIdealFootstepWidth());
      double euclideanDistance = startPoint.distance(endPoint);
      double yaw = AngleTools.computeAngleDifferenceMinusPiToPi(startNode.getYaw(), endNode.getYaw());
      return euclideanDistance + parameters.getYawWeight() * Math.abs(yaw) + parameters.getCostPerStep();
   }

   public static Point2D computeMidFootPoint(FootstepNode node, double idealStepWidth)
   {
      FramePose stanceFootPose = new FramePose(ReferenceFrame.getWorldFrame());
      stanceFootPose.setYawPitchRoll(node.getYaw(), 0.0, 0.0);
      stanceFootPose.setX(node.getX());
      stanceFootPose.setY(node.getY());
      ReferenceFrame stanceFrame = new PoseReferenceFrame("stanceFrame", stanceFootPose);

      FramePoint2D midFootPoint = new FramePoint2D(stanceFrame);
      double ySign = node.getRobotSide().negateIfLeftSide(1.0);
      midFootPoint.setY(ySign * idealStepWidth / 2.0);
      midFootPoint.changeFrame(ReferenceFrame.getWorldFrame());
      return midFootPoint.getPoint();
   }
}
