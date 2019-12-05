package us.ihmc.footstepPlanning.graphSearch.stepCost;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

import java.util.function.DoubleSupplier;

public class QuadraticDistanceAndYawCost implements FootstepCost
{
   private final DoubleSupplier idealFootstepWidth;
   private final DoubleSupplier idealFootstepLength;
   private final DoubleSupplier longStepWeight;
   private final DoubleSupplier forwardWeight;
   private final DoubleSupplier lateralWeight;
   private final DoubleSupplier yawWeight;
   private final DoubleSupplier costPerStep;

   private final FramePoint3D endNodePosition = new FramePoint3D();
   private final FramePose3D startNodePose = new FramePose3D(ReferenceFrame.getWorldFrame());
   private final PoseReferenceFrame startNodeFrame = new PoseReferenceFrame("startNodeFrame", ReferenceFrame.getWorldFrame());

   public QuadraticDistanceAndYawCost(FootstepPlannerParametersReadOnly parameters)
   {
      this(parameters::getIdealFootstepWidth,
           parameters::getIdealFootstepLength,
           parameters::getLongStepWeight,
           parameters::getForwardWeight,
           parameters::getLateralWeight,
           parameters::getYawWeight,
           parameters::getCostPerStep);
   }

   public QuadraticDistanceAndYawCost(DoubleSupplier idealFootstepWidth,
                                      DoubleSupplier idealFootstepLength,
                                      DoubleSupplier longStepWeight,
                                      DoubleSupplier forwardWeight,
                                      DoubleSupplier lateralWeight,
                                      DoubleSupplier yawWeight,
                                      DoubleSupplier costPerStep)
   {
      this.idealFootstepWidth = idealFootstepWidth;
      this.idealFootstepLength = idealFootstepLength;
      this.longStepWeight = longStepWeight;
      this.forwardWeight = forwardWeight;
      this.lateralWeight = lateralWeight;
      this.yawWeight = yawWeight;
      this.costPerStep = costPerStep;
   }

   @Override
   public double compute(FootstepNode startNode, FootstepNode endNode)
   {
      Point2D startPoint = startNode.getOrComputeMidFootPoint(idealFootstepWidth.getAsDouble());
      Point2D endPoint = endNode.getOrComputeMidFootPoint(idealFootstepWidth.getAsDouble());

      startNodePose.setPosition(startPoint.getX(), startPoint.getY(), 0.0);
      startNodePose.setOrientationYawPitchRoll(startNode.getYaw(), 0.0, 0.0);
      startNodeFrame.setPoseAndUpdate(startNodePose);

      endNodePosition.setIncludingFrame(ReferenceFrame.getWorldFrame(), endPoint, 0.0);
      endNodePosition.changeFrame(startNodeFrame);

      double stepDistance = endNodePosition.distanceFromOrigin();
      double cost;
      if (stepDistance > idealFootstepLength.getAsDouble())
      {
         cost = longStepWeight.getAsDouble() * stepDistance;
      }
      else
      {
         cost = forwardWeight.getAsDouble() * Math.pow(endNodePosition.getX(), 2.0);
         cost += lateralWeight.getAsDouble() * Math.pow(endNodePosition.getY(), 2.0);
      }

      double yaw = AngleTools.computeAngleDifferenceMinusPiToPi(startNode.getYaw(), endNode.getYaw());
      cost += yawWeight.getAsDouble() * Math.abs(yaw) + costPerStep.getAsDouble();

      return cost;
   }
}
