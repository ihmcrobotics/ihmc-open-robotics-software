package us.ihmc.quadrupedRobotics.planning;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class QuadrupedStepCrossoverProjection
{
   private final DoubleParameter minimumStepClearanceParameter;
   private final DoubleParameter maximumStepStrideParameter;

   private final FramePoint3D goalPosition;
   private final ReferenceFrame bodyZUpFrame;

   public QuadrupedStepCrossoverProjection(ReferenceFrame bodyZUpFrame, YoVariableRegistry registry)
   {
      minimumStepClearanceParameter = new DoubleParameter("minimumStepClearance", registry, 0.075);
      maximumStepStrideParameter = new DoubleParameter("maximumStepStride", registry, 1.0);
      this.goalPosition = new FramePoint3D();
      this.bodyZUpFrame = bodyZUpFrame;
   }

   public void project(QuadrupedTimedStep step, QuadrantDependentList<FramePoint3D> solePositionEstimate)
   {
      step.getGoalPosition(goalPosition);
      project(goalPosition, solePositionEstimate, step.getRobotQuadrant());
      step.setGoalPosition(goalPosition);
   }

   public void project(FramePoint3D goalPosition, QuadrantDependentList<FramePoint3D> solePositionEstimate, RobotQuadrant stepQuadrant)
   {
      ReferenceFrame referenceFrame = goalPosition.getReferenceFrame();
      goalPosition.changeFrame(bodyZUpFrame);

      FramePoint3D acrossBodySolePosition = solePositionEstimate.get(stepQuadrant.getAcrossBodyQuadrant());
      acrossBodySolePosition.changeFrame(bodyZUpFrame);

      double xStride = goalPosition.getX() - acrossBodySolePosition.getX();
      double yStride = goalPosition.getY() - acrossBodySolePosition.getY();

      if (xStride > maximumStepStrideParameter.getValue())
         xStride = maximumStepStrideParameter.getValue();
      if (xStride < -maximumStepStrideParameter.getValue())
         xStride = -maximumStepStrideParameter.getValue();
      if (stepQuadrant.getSide().negateIfRightSide(yStride) > maximumStepStrideParameter.getValue())
         yStride = stepQuadrant.getSide().negateIfRightSide(maximumStepStrideParameter.getValue());
      if (stepQuadrant.getSide().negateIfRightSide(yStride) < minimumStepClearanceParameter.getValue())
         yStride = stepQuadrant.getSide().negateIfRightSide(minimumStepClearanceParameter.getValue());

      goalPosition.setX(acrossBodySolePosition.getX() + xStride);
      goalPosition.setY(acrossBodySolePosition.getY() + yStride);
      goalPosition.changeFrame(referenceFrame);
   }
}
