package us.ihmc.commonWalkingControlModules.capturePoint.comBasedPlanner;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.time.TimeInterval;
import us.ihmc.robotics.time.TimeIntervalBasics;
import us.ihmc.robotics.time.TimeIntervalProvider;
import us.ihmc.robotics.time.TimeIntervalReadOnly;

import java.util.List;

public class BipedTimedStep extends BipedStep implements TimeIntervalProvider
{
   private final TimeInterval timeInterval = new TimeInterval(Double.NaN, Double.NaN);

   public BipedTimedStep()
   {
      super();
   }

   public BipedTimedStep(RobotSide robotSide, FramePose3D goalPose, double groundClearance, TimeIntervalReadOnly timeInterval)
   {
      super(robotSide, goalPose, groundClearance, null);
      setTimeInterval(timeInterval);
   }

   public BipedTimedStep(RobotSide robotSide, FramePose3D goalPose, double groundClearance, TimeIntervalReadOnly timeInterval,
                         List<Point2D> predictedContactPoints)
   {
      super(robotSide, goalPose, groundClearance, predictedContactPoints);
      setTimeInterval(timeInterval);
   }

   @Override
   public TimeIntervalBasics getTimeInterval()
   {
      return timeInterval;
   }

   public void setTimeInterval(TimeIntervalReadOnly timeInterval)
   {
      this.timeInterval.set(timeInterval);
   }

   public void set(BipedTimedStep other)
   {
      super.set(other);
      setTimeInterval(other.getTimeInterval());
   }

}
