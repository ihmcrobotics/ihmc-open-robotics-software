package us.ihmc.quadrupedRobotics.planning;

import javax.vecmath.Point3d;

import us.ihmc.quadrupedRobotics.util.TimeInterval;
import us.ihmc.quadrupedRobotics.util.TimeIntervalProvider;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedTimedStep extends QuadrupedStep implements TimeIntervalProvider
{
   /**
    * The relative time interval of the swing phase (with respect to the start of the previous swing phase).
    */
   private final TimeInterval timeInterval;
   private boolean absolute = true;

   public QuadrupedTimedStep()
   {
      super();
      this.timeInterval = new TimeInterval(0.5, 1.0);
   }

   public QuadrupedTimedStep(RobotQuadrant robotQuadrant, FramePoint goalPosition, double groundClearance, TimeInterval timeInterval)
   {
      super(robotQuadrant, goalPosition, groundClearance);
      this.timeInterval = new TimeInterval(timeInterval);
   }

   public QuadrupedTimedStep(RobotQuadrant robotQuadrant, Point3d goalPosition, double groundClearance, TimeInterval timeInterval)
   {
      super(robotQuadrant, goalPosition, groundClearance);
      this.timeInterval = new TimeInterval(timeInterval);
   }

   public QuadrupedTimedStep(RobotQuadrant robotQuadrant, Point3d goalPosition, double groundClearance, TimeInterval timeInterval, boolean absolute)
   {
      super(robotQuadrant, goalPosition, groundClearance);
      this.timeInterval = new TimeInterval(timeInterval);
      this.absolute = absolute;
   }

   public QuadrupedTimedStep(QuadrupedStep quadrupedStep, TimeInterval timeInterval)
   {
      super(quadrupedStep);
      this.timeInterval = new TimeInterval(timeInterval);
   }

   public QuadrupedTimedStep(QuadrupedTimedStep quadrupedTimedStep)
   {
      super(quadrupedTimedStep);
      this.timeInterval = new TimeInterval(quadrupedTimedStep.timeInterval);
      this.absolute = quadrupedTimedStep.absolute;
   }

   public void set(QuadrupedTimedStep quadrupedTimedStep)
   {
      super.set(quadrupedTimedStep);
      this.timeInterval.set(quadrupedTimedStep.timeInterval);
      this.absolute = quadrupedTimedStep.absolute;
   }

   public void get(QuadrupedTimedStep quadrupedTimedStep)
   {
      super.get(quadrupedTimedStep);
      this.timeInterval.get(quadrupedTimedStep.timeInterval);
      quadrupedTimedStep.absolute = this.absolute;
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

   public boolean isAbsolute()
   {
      return absolute;
   }

   public void setAbsolute(boolean absolute)
   {
      this.absolute = absolute;
   }

   public boolean epsilonEquals(QuadrupedTimedStep other, double epsilon)
   {
      return super.epsilonEquals(other, epsilon) && this.timeInterval.epsilonEquals(other.timeInterval, epsilon);
   }

   @Override public String toString()
   {
      String string = super.toString();
      string += "\nstartTime: " + timeInterval.getStartTime();
      string += "\nendTime: " + timeInterval.getEndTime();
      return string;
   }
}

