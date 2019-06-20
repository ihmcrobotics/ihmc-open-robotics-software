package us.ihmc.quadrupedBasics.gait;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.QuadrupedTimedStepCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.TimeIntervalCommand;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.time.TimeInterval;
import us.ihmc.robotics.time.TimeIntervalBasics;
import us.ihmc.robotics.time.TimeIntervalProvider;

public class QuadrupedTimedStep extends QuadrupedStep implements TimeIntervalProvider
{
   private final TimeInterval timeInterval = new TimeInterval(0.5, 1.0);

   public QuadrupedTimedStep()
   {
   }

   public QuadrupedTimedStep(RobotQuadrant robotQuadrant, FramePoint3D goalPosition, double groundClearance, TimeIntervalBasics timeInterval)
   {
      super(robotQuadrant, goalPosition, groundClearance);
      setTimeInterval(timeInterval);
   }

   public QuadrupedTimedStep(RobotQuadrant robotQuadrant, Point3DBasics goalPosition, double groundClearance, TimeIntervalBasics timeInterval)
   {
      this();
      setRobotQuadrant(robotQuadrant);
      setGoalPosition(goalPosition);
      setGroundClearance(groundClearance);
      setTimeInterval(timeInterval);
   }

   public QuadrupedTimedStep(QuadrupedTimedStep other)
   {
      this(other.getRobotQuadrant(), other.getGoalPositionInternal(), other.getGroundClearance(), other.getTimeInterval());
   }

   @Override
   public TimeIntervalBasics getTimeInterval()
   {
      return timeInterval;
   }

   public void setTimeInterval(TimeIntervalBasics timeInterval)
   {
      getTimeInterval().set(timeInterval);
   }

   public void setTimeInterval(TimeIntervalCommand command)
   {
      command.getTimeInterval(getTimeInterval());
   }

   public void set(QuadrupedTimedStep other)
   {
      super.set(other);
      setTimeInterval(other.getTimeInterval());
   }

   public void set(QuadrupedTimedStepCommand command)
   {
      super.set(command.getStepCommand());
      setTimeInterval(command.getTimeIntervalCommand());
   }

   public void get(QuadrupedTimedStep other)
   {
      super.get(other);
      other.setTimeInterval(getTimeInterval());
   }

   public boolean epsilonEquals(QuadrupedTimedStep other, double epsilon)
   {
      return super.epsilonEquals(other, epsilon) && getTimeInterval().epsilonEquals(other.getTimeInterval(), epsilon);
   }

   @Override
   public boolean equals(Object obj)
   {
      if (this == obj)
         return true;
      if (obj == null)
         return false;
      if (getClass() != obj.getClass())
         return false;
      QuadrupedTimedStep other = (QuadrupedTimedStep) obj;

      if (!getTimeInterval().epsilonEquals(other.getTimeInterval(), 0.0))
         return false;

      return super.equals(other);
   }

   @Override
   public String toString()
   {
      String string = super.toString();
      string += "\nstartTime: " + getTimeInterval().getStartTime();
      string += "\nendTime: " + getTimeInterval().getEndTime();
      return string;
   }
}

