package us.ihmc.quadrupedRobotics.planning;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedStepCrossoverProjection
{
   private final QuadrantDependentList<ReferenceFrame> soleFrames;
   private final FramePoint goalPosition;
   private final double minimumClearance;
   private final double maximumStride;

   public QuadrupedStepCrossoverProjection(QuadrantDependentList<ReferenceFrame> soleFrames, double minimumClearance, double maximumStride)
   {
      this.soleFrames = soleFrames;
      this.goalPosition = new FramePoint();
      this.minimumClearance = minimumClearance;
      this.maximumStride = maximumStride;
   }

   public void project(QuadrupedStep step)
   {
      step.getGoalPosition(goalPosition);
      RobotQuadrant stepQuadrant = step.getRobotQuadrant();
      RobotQuadrant acrossBodyQuadrant = stepQuadrant.getAcrossBodyQuadrant();
      ReferenceFrame acrossBodySoleFrame = soleFrames.get(acrossBodyQuadrant);
      goalPosition.changeFrame(acrossBodySoleFrame);

      if (stepQuadrant.getSide().negateIfRightSide(goalPosition.getY()) < minimumClearance)
      {
         goalPosition.setY(stepQuadrant.getSide().negateIfRightSide(minimumClearance));
      }
      if (goalPosition.getX() > maximumStride)
      {
         goalPosition.setX(maximumStride);
      }
      if (goalPosition.getX() < -maximumStride)
      {
         goalPosition.setX(-maximumStride);
      }

      step.setGoalPosition(goalPosition);
   }
}
