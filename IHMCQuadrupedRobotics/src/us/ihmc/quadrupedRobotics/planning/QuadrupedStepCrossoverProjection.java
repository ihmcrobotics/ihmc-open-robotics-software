package us.ihmc.quadrupedRobotics.planning;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedStepCrossoverProjection
{
   private final QuadrantDependentList<ReferenceFrame> soleFrames;
   private final double minimumClearance;
   private final double maximumStride;
   private final FramePoint goalPosition;

   public QuadrupedStepCrossoverProjection(QuadrantDependentList<ReferenceFrame> soleFrames, double minimumClearance, double maximumStride)
   {
      this.soleFrames = soleFrames;
      this.minimumClearance = minimumClearance;
      this.maximumStride = maximumStride;
      this.goalPosition = new FramePoint();
   }

   public void project(QuadrupedTimedStep step)
   {
      step.getGoalPosition(goalPosition);
      project(step.getRobotQuadrant(), goalPosition);
      step.setGoalPosition(goalPosition);
   }

   public void project(RobotQuadrant stepQuadrant, FramePoint goalPosition)
   {
      ReferenceFrame referenceFrame = goalPosition.getReferenceFrame();
      RobotQuadrant acrossBodyQuadrant = stepQuadrant.getAcrossBodyQuadrant();
      ReferenceFrame acrossBodySoleFrame = soleFrames.get(acrossBodyQuadrant);
      goalPosition.changeFrame(acrossBodySoleFrame);

      if (stepQuadrant.getSide().negateIfRightSide(goalPosition.getY()) > maximumStride)
         goalPosition.setY(stepQuadrant.getSide().negateIfRightSide(maximumStride));
      if (stepQuadrant.getSide().negateIfRightSide(goalPosition.getY()) < minimumClearance)
         goalPosition.setY(stepQuadrant.getSide().negateIfRightSide(minimumClearance));
      if (goalPosition.getX() > maximumStride)
         goalPosition.setX(maximumStride);
      if (goalPosition.getX() < -maximumStride)
         goalPosition.setX(-maximumStride);

      goalPosition.changeFrame(referenceFrame);
   }
}
