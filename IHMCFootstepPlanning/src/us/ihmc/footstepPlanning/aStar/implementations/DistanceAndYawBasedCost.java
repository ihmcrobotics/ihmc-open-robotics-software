package us.ihmc.footstepPlanning.aStar.implementations;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.aStar.FootstepCost;
import us.ihmc.footstepPlanning.aStar.FootstepNode;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class DistanceAndYawBasedCost implements FootstepCost
{
   private static final double defaultStepWidth = 0.25;

   private final double costPerStep;
   private final double yawWeight;

   public DistanceAndYawBasedCost(double costPerStep, double yawWeight)
   {
      this.costPerStep = costPerStep;
      this.yawWeight = yawWeight;
   }

   @Override
   public double compute(FootstepNode startNode, FootstepNode endNode)
   {
      Point2D startPoint = computeMidFootPoint(startNode);
      Point2D endPoint = computeMidFootPoint(endNode);
      double euclideanDistance = startPoint.distance(endPoint);
      double yaw = AngleTools.computeAngleDifferenceMinusPiToPi(startNode.getYaw(), endNode.getYaw());
      return euclideanDistance + yawWeight * Math.abs(yaw) + costPerStep;
   }

   public static Point2D computeMidFootPoint(FootstepNode node)
   {
      FramePose stanceFootPose = new FramePose(ReferenceFrame.getWorldFrame());
      stanceFootPose.setYawPitchRoll(node.getYaw(), 0.0, 0.0);
      stanceFootPose.setX(node.getX());
      stanceFootPose.setY(node.getY());
      ReferenceFrame stanceFrame = new PoseReferenceFrame("stanceFrame", stanceFootPose);

      FramePoint2d midFootPoint = new FramePoint2d(stanceFrame);
      double ySign = node.getRobotSide().negateIfLeftSide(1.0);
      midFootPoint.setY(ySign * defaultStepWidth / 2.0);
      midFootPoint.changeFrame(ReferenceFrame.getWorldFrame());
      return midFootPoint.getPoint();
   }
}
