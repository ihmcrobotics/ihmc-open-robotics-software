package us.ihmc.aware.planning;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedStep
{
   private RobotQuadrant robotQuadrant;
   private final FramePoint goalPosition;

   public QuadrupedStep()
   {
      this.robotQuadrant = RobotQuadrant.FRONT_RIGHT;
      this.goalPosition = new FramePoint();
   }

   public QuadrupedStep(RobotQuadrant robotQuadrant)
   {
      this.robotQuadrant = robotQuadrant;
      this.goalPosition = new FramePoint();
   }

   public QuadrupedStep(RobotQuadrant robotQuadrant, FramePoint goalPosition)
   {
      this.robotQuadrant = robotQuadrant;
      this.goalPosition = new FramePoint(goalPosition);
   }

   public QuadrupedStep(QuadrupedStep quadrupedStep)
   {
      this.robotQuadrant = quadrupedStep.robotQuadrant;
      this.goalPosition = new FramePoint(quadrupedStep.goalPosition);
   }

   public void set(QuadrupedStep quadrupedStep)
   {
      this.robotQuadrant = quadrupedStep.robotQuadrant;
      this.goalPosition.setIncludingFrame(quadrupedStep.goalPosition);
   }

   public void get(QuadrupedStep quadrupedStep)
   {
      quadrupedStep.robotQuadrant = this.robotQuadrant;
      quadrupedStep.goalPosition.setIncludingFrame(this.goalPosition);
   }

   public RobotQuadrant getRobotQuadrant()
   {
      return robotQuadrant;
   }

   public void setRobotQuadrant(RobotQuadrant robotQuadrant)
   {
      this.robotQuadrant = robotQuadrant;
   }

   public FramePoint getGoalPosition()
   {
      return goalPosition;
   }

   public void getGoalPosition(FramePoint goalPosition)
   {
      goalPosition.setIncludingFrame(this.goalPosition);
   }

   public void setGoalPosition(FramePoint goalPosition)
   {
      this.goalPosition.setIncludingFrame(goalPosition);
   }
}
