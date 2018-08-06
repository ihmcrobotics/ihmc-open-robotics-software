package us.ihmc.footstepPlanning.graphSearch.stepCost;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

public class QuadraticDistanceAndYawCost implements FootstepCost
{
   private final FootstepPlannerParameters parameters;

   private static final double xCost = 1.0;
   private static final double yCost = 2.0;
   private static final double yawCostScaler = 10.0;

   private final FramePoint3D endNodePosition = new FramePoint3D();
   private final FramePose3D startNodePose = new FramePose3D(ReferenceFrame.getWorldFrame());
   private final PoseReferenceFrame startNodeFrame = new PoseReferenceFrame("startNodeFrame", ReferenceFrame.getWorldFrame());

   public QuadraticDistanceAndYawCost(FootstepPlannerParameters parameters)
   {
      this.parameters = parameters;
   }

   @Override
   public double compute(FootstepNode startNode, FootstepNode endNode)
   {
      Point2D startPoint = startNode.getOrComputeMidFootPoint(parameters.getIdealFootstepWidth());
      Point2D endPoint = endNode.getOrComputeMidFootPoint(parameters.getIdealFootstepWidth());

      startNodePose.setPosition(startPoint.getX(), startPoint.getY(), 0.0);
      startNodePose.setOrientationYawPitchRoll(startNode.getYaw(), 0.0, 0.0);
      startNodeFrame.setPoseAndUpdate(startNodePose);

      endNodePosition.setIncludingFrame(ReferenceFrame.getWorldFrame(), endPoint, 0.0);
      endNodePosition.changeFrame(startNodeFrame);

      double euclideanDistance = startPoint.distance(endPoint);
      double cost = xCost * Math.pow(endNodePosition.getX(), 2.0);
      cost += yCost * Math.pow(endNodePosition.getY(), 2.0);

      double yaw = AngleTools.computeAngleDifferenceMinusPiToPi(startNode.getYaw(), endNode.getYaw());

      cost += yawCostScaler * euclideanDistance * parameters.getYawWeight() * Math.abs(yaw);

      return cost;
   }
}
