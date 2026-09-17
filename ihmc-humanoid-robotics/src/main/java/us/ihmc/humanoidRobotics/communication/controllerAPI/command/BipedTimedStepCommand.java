package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.BipedTimedStepMessage;
import us.ihmc.commons.time.TimeInterval;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;

public class BipedTimedStepCommand implements Command<BipedTimedStepCommand, BipedTimedStepMessage>
{
   private final TimeInterval timeInterval = new TimeInterval();

   private long sequenceId;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private RobotSide robotSide = null;
   private final FixedFramePose3DBasics goalPose = new FramePose3D(worldFrame);
   private double swingHeight = 0.0;

   public void setGoalPose(FramePose3DReadOnly goalPose)
   {
      this.goalPose.set(goalPose);
   }

   public void setRobotSide(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }

   public void setTimeInterval(double startTime, double endTime)
   {
      timeInterval.setInterval(startTime, endTime);
   }

   public void setSwingHeight(double swingHeight)
   {
      this.swingHeight = swingHeight;
   }

   public TimeInterval getTimeInterval()
   {
      return timeInterval;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public FramePose3DReadOnly getGoalPose()
   {
      return goalPose;
   }

   public double getSwingHeight()
   {
      return swingHeight;
   }

   @Override
   public void clear()
   {
      sequenceId = 0;
      robotSide = null;
      goalPose.setToNaN();
      swingHeight = 0.0;
   }

   @Override
   public void setFromMessage(BipedTimedStepMessage message)
   {
      sequenceId = message.getSequenceId();
      robotSide = RobotSide.fromByte(message.getRobotSide());
      swingHeight = message.getSwingHeight();
      goalPose.set(message.getLocation().getPoint(), message.getOrientation().getQuaternion());
      timeInterval.setInterval(message.getStartTime(), message.getEndTime());
   }

   @Override
   public Class<BipedTimedStepMessage> getMessageClass()
   {
      return BipedTimedStepMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return robotSide != null;
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }

   @Override
   public void set(BipedTimedStepCommand other)
   {
      sequenceId = other.getSequenceId();
      robotSide = other.robotSide;
      swingHeight = other.swingHeight;
      goalPose.set(other.goalPose);
      timeInterval.set(other.timeInterval);
   }
}
