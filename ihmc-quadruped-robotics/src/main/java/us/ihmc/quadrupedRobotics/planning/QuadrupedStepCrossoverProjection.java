package us.ihmc.quadrupedRobotics.planning;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.dataStructures.parameter.DoubleParameter;
import us.ihmc.robotics.dataStructures.parameter.ParameterFactory;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class QuadrupedStepCrossoverProjection
{
   private final DoubleParameter minimumStepClearanceParameter;
   private final DoubleParameter maximumStepStrideParameter;

   private final FramePoint3D goalPosition;
   private final ReferenceFrame bodyZUpFrame;

   public QuadrupedStepCrossoverProjection(ReferenceFrame bodyZUpFrame, YoVariableRegistry registry)
   {
      ParameterFactory parameterFactory = ParameterFactory.createWithRegistry(getClass(), registry);
      minimumStepClearanceParameter = parameterFactory.createDouble("minimumStepClearance", 0.075);
      maximumStepStrideParameter = parameterFactory.createDouble("maximumStepStride", 1.0);
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

      if (xStride > maximumStepStrideParameter.get())
         xStride = maximumStepStrideParameter.get();
      if (xStride < -maximumStepStrideParameter.get())
         xStride = -maximumStepStrideParameter.get();
      if (stepQuadrant.getSide().negateIfRightSide(yStride) > maximumStepStrideParameter.get())
         yStride = stepQuadrant.getSide().negateIfRightSide(maximumStepStrideParameter.get());
      if (stepQuadrant.getSide().negateIfRightSide(yStride) < minimumStepClearanceParameter.get())
         yStride = stepQuadrant.getSide().negateIfRightSide(minimumStepClearanceParameter.get());

      goalPosition.setX(acrossBodySolePosition.getX() + xStride);
      goalPosition.setY(acrossBodySolePosition.getY() + yStride);
      goalPosition.changeFrame(referenceFrame);
   }
}
