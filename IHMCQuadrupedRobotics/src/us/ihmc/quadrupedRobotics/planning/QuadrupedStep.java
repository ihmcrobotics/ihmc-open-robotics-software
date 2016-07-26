package us.ihmc.quadrupedRobotics.planning;

import javax.vecmath.Point3d;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotEnd;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedStep
{
   private RobotQuadrant robotQuadrant;
   private Point3d goalPosition;
   private double groundClearance;

   public QuadrupedStep()
   {
      this.robotQuadrant = RobotQuadrant.FRONT_RIGHT;
      this.goalPosition = new Point3d(0.0, 0.0, 0.0);
      this.groundClearance = 0.0;
   }

   public QuadrupedStep(RobotQuadrant robotQuadrant, Point3d goalPosition, double groundClearance)
   {
      this.robotQuadrant = robotQuadrant;
      this.goalPosition = new Point3d();
      this.groundClearance = groundClearance;
      setGoalPosition(goalPosition);
   }

   public QuadrupedStep(RobotQuadrant robotQuadrant, FramePoint goalPosition, double groundClearance)
   {
      this.robotQuadrant = robotQuadrant;
      this.groundClearance = groundClearance;
      setGoalPosition(goalPosition);
   }

   public QuadrupedStep(QuadrupedStep quadrupedStep)
   {
      this.robotQuadrant = quadrupedStep.robotQuadrant;
      this.goalPosition = new Point3d();
      this.groundClearance = quadrupedStep.groundClearance;
      setGoalPosition(quadrupedStep.getGoalPosition());
   }

   public void set(QuadrupedStep quadrupedStep)
   {
      this.robotQuadrant = quadrupedStep.robotQuadrant;
      this.goalPosition = new Point3d();
      this.groundClearance = quadrupedStep.groundClearance;
      setGoalPosition(quadrupedStep.getGoalPosition());
   }

   public void get(QuadrupedStep quadrupedStep)
   {
      quadrupedStep.robotQuadrant = this.robotQuadrant;
      quadrupedStep.goalPosition.set(this.goalPosition);
      quadrupedStep.groundClearance = this.groundClearance;
   }

   public RobotQuadrant getRobotQuadrant()
   {
      return robotQuadrant;
   }
   
   public RobotEnd getRobotEnd()
   {
      return robotQuadrant.getEnd();
   }

   public void setRobotQuadrant(RobotQuadrant robotQuadrant)
   {
      this.robotQuadrant = robotQuadrant;
   }

   public Point3d getGoalPosition()
   {
      return goalPosition;
   }

   public void getGoalPosition(Point3d goalPosition)
   {
      this.goalPosition.get(goalPosition);
   }

   public void getGoalPosition(FramePoint goalPosition)
   {
      ReferenceFrame originalFrame = goalPosition.getReferenceFrame();
      goalPosition.changeFrame(ReferenceFrame.getWorldFrame());
      goalPosition.setPoint(this.goalPosition);
      goalPosition.changeFrame(originalFrame);
   }

   public void setGoalPosition(Point3d goalPosition)
   {
      this.goalPosition.set(goalPosition);
   }

   public void setGoalPosition(FramePoint goalPosition)
   {
      ReferenceFrame originalFrame = goalPosition.getReferenceFrame();
      goalPosition.changeFrame(ReferenceFrame.getWorldFrame());
      goalPosition.getPoint(this.goalPosition);
      goalPosition.changeFrame(originalFrame);
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

   @Override public String toString()
   {
      String string = super.toString();
      string += "\nrobotQuadrant: " + robotQuadrant;
      string += "\ngoalPosition:" + goalPosition;
      string += "\ngroundClearance: " + groundClearance;
      return string;
   }
}
