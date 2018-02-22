package us.ihmc.quadrupedRobotics.planning;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedStep
{
   private RobotQuadrant robotQuadrant = RobotQuadrant.FRONT_RIGHT;
   private Point3D goalPosition = new Point3D(0.0, 0.0, 0.0);
   private double groundClearance = 0.0;

   public QuadrupedStep()
   {
   }

   public QuadrupedStep(RobotQuadrant robotQuadrant, FramePoint3D goalPosition, double groundClearance)
   {
      setRobotQuadrant(robotQuadrant);
      setGoalPosition(goalPosition);
      setGroundClearance(groundClearance);
   }

   public QuadrupedStep(RobotQuadrant robotQuadrant, Point3D goalPosition, double groundClearance)
   {
      this();
      setRobotQuadrant(robotQuadrant);
      setGoalPosition(goalPosition);
      setGroundClearance(groundClearance);
   }

   public QuadrupedStep(QuadrupedStep other)
   {
      this(other.getRobotQuadrant(), other.getGoalPosition(), other.getGroundClearance());
   }

   public RobotQuadrant getRobotQuadrant()
   {
      return robotQuadrant;
   }

   public void setRobotQuadrant(RobotQuadrant robotQuadrant)
   {
      this.robotQuadrant = robotQuadrant;
   }

   /**
    * Unsafe for external use.
    */
   protected Point3D getGoalPosition()
   {
      return goalPosition;
   }

   public void getGoalPosition(Point3D goalPosition)
   {
      goalPosition.set(this.goalPosition);
   }

   public void getGoalPosition(FramePoint3D goalPosition)
   {
      ReferenceFrame originalFrame = goalPosition.getReferenceFrame();
      goalPosition.changeFrame(ReferenceFrame.getWorldFrame());
      goalPosition.set(this.goalPosition);
      goalPosition.changeFrame(originalFrame);
   }

   public void setGoalPosition(Point3DReadOnly goalPosition)
   {
      this.goalPosition.set(goalPosition);
   }

   public void setGoalPosition(FramePoint3D goalPosition)
   {
      ReferenceFrame originalFrame = goalPosition.getReferenceFrame();
      goalPosition.changeFrame(ReferenceFrame.getWorldFrame());
      this.goalPosition.set(goalPosition);
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

   public void set(QuadrupedStep other)
   {
      setRobotQuadrant(other.getRobotQuadrant());
      setGoalPosition(other.getGoalPosition());
      setGroundClearance(other.getGroundClearance());
   }

   public void get(QuadrupedTimedStep other)
   {
      other.setRobotQuadrant(getRobotQuadrant());
      other.setGoalPosition(getGoalPosition());
      other.setGroundClearance(getGroundClearance());
   }

   public boolean epsilonEquals(QuadrupedTimedStep other, double epsilon)
   {
      return getRobotQuadrant() == other.getRobotQuadrant() &&
             getGoalPosition().epsilonEquals(other.getGoalPosition(), epsilon) &&
             MathTools.epsilonEquals(getGroundClearance(), other.getGroundClearance(), epsilon);

   }

   @Override public String toString()
   {
      String string = super.toString();
      string += "\nrobotQuadrant: " + getRobotQuadrant();
      string += "\ngoalPosition:" + getGoalPosition();
      string += "\ngroundClearance: " + getGroundClearance();
      return string;
   }
}
