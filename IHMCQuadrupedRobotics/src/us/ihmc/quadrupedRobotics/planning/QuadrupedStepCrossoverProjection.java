package us.ihmc.quadrupedRobotics.planning;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedStepCrossoverProjection
{
   private final double minimumClearance;
   private final double maximumStride;
   private final FramePoint goalPosition;
   private final ReferenceFrame bodyZUpFrame;

   public QuadrupedStepCrossoverProjection(ReferenceFrame bodyZUpFrame, double minimumClearance, double maximumStride)
   {
      this.minimumClearance = minimumClearance;
      this.maximumStride = maximumStride;
      this.goalPosition = new FramePoint();
      this.bodyZUpFrame = bodyZUpFrame;
   }

   public void project(QuadrupedTimedStep step, QuadrantDependentList<FramePoint> solePositionEstimate)
   {
      step.getGoalPosition(goalPosition);
      project(goalPosition, solePositionEstimate, step.getRobotQuadrant());
      step.setGoalPosition(goalPosition);
   }

   public void project(FramePoint goalPosition, QuadrantDependentList<FramePoint> solePositionEstimate, RobotQuadrant stepQuadrant)
   {
      ReferenceFrame referenceFrame = goalPosition.getReferenceFrame();
      goalPosition.changeFrame(bodyZUpFrame);

      FramePoint acrossBodySolePosition = solePositionEstimate.get(stepQuadrant.getAcrossBodyQuadrant());
      acrossBodySolePosition.changeFrame(bodyZUpFrame);

      double xStride = goalPosition.getX() - acrossBodySolePosition.getX();
      double yStride = goalPosition.getY() - acrossBodySolePosition.getY();

      if (xStride > maximumStride)
         xStride = maximumStride;
      if (xStride < -maximumStride)
         xStride = -maximumStride;
      if (stepQuadrant.getSide().negateIfRightSide(yStride) > maximumStride)
         yStride = stepQuadrant.getSide().negateIfRightSide(maximumStride);
      if (stepQuadrant.getSide().negateIfRightSide(yStride) < minimumClearance)
         yStride = stepQuadrant.getSide().negateIfRightSide(minimumClearance);

      goalPosition.setX(acrossBodySolePosition.getX() + xStride);
      goalPosition.setY(acrossBodySolePosition.getY() + yStride);
      goalPosition.changeFrame(referenceFrame);
   }
}
