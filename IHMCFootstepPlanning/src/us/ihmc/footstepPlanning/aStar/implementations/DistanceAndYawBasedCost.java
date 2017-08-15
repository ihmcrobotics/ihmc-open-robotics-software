package us.ihmc.footstepPlanning.aStar.implementations;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.aStar.FootstepCost;
import us.ihmc.footstepPlanning.aStar.FootstepNode;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class DistanceAndYawBasedCost implements FootstepCost
{
   private static final double defaultStepWidth = 0.25;

   private final double costPerStep = 0.005;
   private final YoDouble yawWeight;

   public DistanceAndYawBasedCost(double yawWeight, YoVariableRegistry registry)
   {
      String namePrefix = getClass().getSimpleName();
      this.yawWeight = new YoDouble(namePrefix + "_YawWeight", registry);
      this.yawWeight.set(yawWeight);
   }

   @Override
   public double compute(FootstepNode startNode, FootstepNode endNode)
   {
      Point2D startPoint = computeMidFootPoint(startNode);
      Point2D endPoint = computeMidFootPoint(endNode);
      double euclideanDistance = startPoint.distance(endPoint);
      double yaw = AngleTools.computeAngleDifferenceMinusPiToPi(startNode.getYaw(), endNode.getYaw());
      return euclideanDistance + yawWeight.getDoubleValue() * Math.abs(yaw) + costPerStep;
   }

   public static Point2D computeMidFootPoint(FootstepNode node)
   {
      FramePose stanceFootPose = new FramePose(ReferenceFrame.getWorldFrame());
      stanceFootPose.setYawPitchRoll(node.getYaw(), 0.0, 0.0);
      stanceFootPose.setX(node.getX());
      stanceFootPose.setY(node.getY());
      ReferenceFrame stanceFrame = new PoseReferenceFrame("stanceFrame", stanceFootPose);

      FramePoint2D midFootPoint = new FramePoint2D(stanceFrame);
      double ySign = node.getRobotSide().negateIfLeftSide(1.0);
      midFootPoint.setY(ySign * defaultStepWidth / 2.0);
      midFootPoint.changeFrame(ReferenceFrame.getWorldFrame());
      return midFootPoint.getPoint();
   }
}
