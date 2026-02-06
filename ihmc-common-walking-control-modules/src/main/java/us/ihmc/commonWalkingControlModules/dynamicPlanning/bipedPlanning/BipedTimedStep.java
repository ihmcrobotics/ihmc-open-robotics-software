package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import us.ihmc.commons.time.TimeInterval;
import us.ihmc.commons.time.TimeIntervalBasics;
import us.ihmc.commons.time.TimeIntervalProvider;
import us.ihmc.commons.time.TimeIntervalReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.BipedTimedStepCommand;
import us.ihmc.robotics.robotSide.RobotSide;

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

   public void set(BipedTimedStepCommand command)
   {
      setTimeInterval(command.getTimeInterval());
      setGoalPose(command.getGoalPose());
      setRobotSide(command.getRobotSide());
      setSwingHeight(command.getSwingHeight());
      setSequenceID(command.getSequenceId());
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
