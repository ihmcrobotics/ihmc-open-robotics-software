package us.ihmc.aware.util;

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

   public QuadrupedTimedStep(RobotQuadrant robotQuadrant)
   {
      super(robotQuadrant);
      this.timeInterval = new TimeInterval(0.5, 1.0);
   }

   public QuadrupedTimedStep(RobotQuadrant robotQuadrant, FramePoint goalPosition)
   {
      super(robotQuadrant, goalPosition);
      this.timeInterval = new TimeInterval(0.5, 1.0);
   }

   public QuadrupedTimedStep(RobotQuadrant robotQuadrant, FramePoint goalPosition, TimeInterval timeInterval)
   {
      super(robotQuadrant, goalPosition);
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
}

