package us.ihmc.quadrupedRobotics.planning;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedStep
{
   private RobotQuadrant robotQuadrant;
   private final FramePoint goalPosition;
   private double groundClearance;

   public QuadrupedStep()
   {
      this.robotQuadrant = RobotQuadrant.FRONT_RIGHT;
      this.goalPosition = new FramePoint(ReferenceFrame.getWorldFrame());
      this.groundClearance = 0.0;
   }

   public QuadrupedStep(RobotQuadrant robotQuadrant, FramePoint goalPosition, double groundClearance)
   {
      this.robotQuadrant = robotQuadrant;
      this.goalPosition = new FramePoint(goalPosition);
      this.goalPosition.changeFrame(ReferenceFrame.getWorldFrame());
      this.groundClearance = groundClearance;
   }

   public QuadrupedStep(QuadrupedStep quadrupedStep)
   {
      this.robotQuadrant = quadrupedStep.robotQuadrant;
      this.goalPosition = new FramePoint(quadrupedStep.goalPosition);
      this.goalPosition.changeFrame(ReferenceFrame.getWorldFrame());
      this.groundClearance = quadrupedStep.groundClearance;
   }

   public void set(QuadrupedStep quadrupedStep)
   {
      this.robotQuadrant = quadrupedStep.robotQuadrant;
      this.goalPosition.setIncludingFrame(quadrupedStep.goalPosition);
      this.goalPosition.changeFrame(ReferenceFrame.getWorldFrame());
      this.groundClearance = quadrupedStep.groundClearance;
   }

   public void get(QuadrupedStep quadrupedStep)
   {
      quadrupedStep.robotQuadrant = this.robotQuadrant;
      quadrupedStep.goalPosition.setIncludingFrame(this.goalPosition);
      quadrupedStep.groundClearance = this.groundClearance;
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
      this.goalPosition.changeFrame(ReferenceFrame.getWorldFrame());
   }

   public double getGroundClearance()
   {
      return groundClearance;
   }

   public void setGroundClearance(double groundClearance)
   {
      this.groundClearance = groundClearance;
   }

   public boolean epsilonEquals(QuadrupedStep other, double epsilon)
   {
      if (this.robotQuadrant != other.robotQuadrant)
      {
         return false;
      }
      else
      {
         return this.goalPosition.epsilonEquals(other.goalPosition, epsilon) && MathTools.epsilonEquals(this.groundClearance, other.groundClearance, epsilon);
      }
   }
}
