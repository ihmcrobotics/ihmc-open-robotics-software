package us.ihmc.quadrupedRobotics.planning;

import us.ihmc.quadrupedRobotics.util.TimeInterval;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedTimedStep extends QuadrupedStep
{
   /**
    * The relative time interval of the swing phase (with respect to the start of the previous swing phase).
    */
   private final TimeInterval timeInterval;

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

   public QuadrupedTimedStep(QuadrupedStep quadrupedStep, TimeInterval timeInterval)
   {
      super(quadrupedStep);
      this.timeInterval = new TimeInterval(timeInterval);
   }

   public QuadrupedTimedStep(QuadrupedTimedStep quadrupedTimedStep)
   {
      super(quadrupedTimedStep);
      this.timeInterval = new TimeInterval(quadrupedTimedStep.timeInterval);
   }

   public void set(QuadrupedTimedStep quadrupedTimedStep)
   {
      super.set(quadrupedTimedStep);
      this.timeInterval.set(quadrupedTimedStep.timeInterval);
   }

   public void get(QuadrupedTimedStep quadrupedTimedStep)
   {
      super.get(quadrupedTimedStep);
      this.timeInterval.get(quadrupedTimedStep.timeInterval);
   }

   public TimeInterval getTimeInterval()
   {
      return timeInterval;
   }

   public void getTimeInterval(TimeInterval timeInterval)
   {
      timeInterval.get(this.timeInterval);
   }

   public void setTimeInterval(TimeInterval timeInterval)
   {
      this.timeInterval.set(timeInterval);
   }

   public boolean epsilonEquals(QuadrupedTimedStep other, double epsilon)
   {
      return super.epsilonEquals(other, epsilon) && this.timeInterval.epsilonEquals(other.timeInterval, epsilon);
   }
}

