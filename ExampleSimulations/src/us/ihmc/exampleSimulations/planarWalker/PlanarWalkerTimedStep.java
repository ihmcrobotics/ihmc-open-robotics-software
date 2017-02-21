package us.ihmc.exampleSimulations.planarWalker;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.quadrupedRobotics.util.TimeInterval;
import us.ihmc.quadrupedRobotics.util.TimeIntervalProvider;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class PlanarWalkerTimedStep implements TimeIntervalProvider
{
   private RobotSide robotSide;
   private final TimeInterval timeInterval;
   private Point3D goalPosition;
   private double groundClearance;

   public PlanarWalkerTimedStep()
   {
      this.robotSide = RobotSide.RIGHT;
      this.timeInterval = new TimeInterval(0.5, 1.0);
      this.goalPosition = new Point3D(0.0, 0.0, 0.0);
      this.groundClearance = 0.0;
   }

   public PlanarWalkerTimedStep(RobotSide robotSide, FramePoint goalPosition, double groundClearance, TimeInterval timeInterval)
   {
      this();
      setRobotSide(robotSide);
      setGoalPosition(goalPosition);
      setGroundClearance(groundClearance);
      setTimeInterval(timeInterval);
   }

   public PlanarWalkerTimedStep(RobotSide robotSide, Point3D goalPosition, double groundClearance, TimeInterval timeInterval)
   {
      this();
      setRobotSide(robotSide);
      setGoalPosition(goalPosition);
      setGroundClearance(groundClearance);
      setTimeInterval(timeInterval);
   }

   public PlanarWalkerTimedStep(PlanarWalkerTimedStep other)
   {
      this(other.getRobotSide(), other.getGoalPosition(), other.getGroundClearance(), other.getTimeInterval());
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public TimeInterval getTimeInterval()
   {
      return timeInterval;
   }

   public void getTimeInterval(TimeInterval timeInterval)
   {
      this.timeInterval.get(timeInterval);
   }

   public void setTimeInterval(TimeInterval timeInterval)
   {
      this.timeInterval.set(timeInterval);
   }

   public void setRobotSide(RobotSide robotSide)
   {
      this.robotSide = robotSide;
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

   public void getGoalPosition(FramePoint goalPosition)
   {
      ReferenceFrame originalFrame = goalPosition.getReferenceFrame();
      goalPosition.changeFrame(ReferenceFrame.getWorldFrame());
      goalPosition.setPoint(this.goalPosition);
      goalPosition.changeFrame(originalFrame);
   }

   public void setGoalPosition(Point3D goalPosition)
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

   public void set(PlanarWalkerTimedStep other)
   {
      setRobotSide(other.getRobotSide());
      setGoalPosition(other.getGoalPosition());
      setGroundClearance(other.getGroundClearance());
      setTimeInterval(other.getTimeInterval());
   }

   public void get(PlanarWalkerTimedStep other)
   {
      other.setRobotSide(getRobotSide());
      other.setGoalPosition(getGoalPosition());
      other.setGroundClearance(getGroundClearance());
      other.setTimeInterval(getTimeInterval());
   }

   public boolean epsilonEquals(PlanarWalkerTimedStep other, double epsilon)
   {
      return getRobotSide() == other.getRobotSide() && getGoalPosition().epsilonEquals(other.getGoalPosition(), epsilon)
            && MathTools.epsilonEquals(getGroundClearance(), other.getGroundClearance(), epsilon)
            && getTimeInterval().epsilonEquals(other.getTimeInterval(), epsilon);

   }

   @Override
   public String toString()
   {
      String string = super.toString();
      string += "\nrobotSide: " + getRobotSide();
      string += "\ngoalPosition:" + getGoalPosition();
      string += "\ngroundClearance: " + getGroundClearance();
      string += "\nstartTime: " + getTimeInterval().getStartTime();
      string += "\nendTime: " + getTimeInterval().getEndTime();
      return string;
   }
}
