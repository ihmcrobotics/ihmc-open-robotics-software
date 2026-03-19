package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin;

import controller_msgs.msg.dds.ControllerWalkToGoalMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.geometry.interfaces.Pose2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;

public class ControllerWalkToGoalCommand implements Command<ControllerWalkToGoalCommand, ControllerWalkToGoalMessage>
{
   private long sequenceId;
   private final FramePose2D goalPose = new FramePose2D();
   private double timeToReachGoal = Double.NaN;
   private boolean holdPosition = false;

   private double positionProximity = 0.02;
   private double orientationProximity = 0.1;

   public ControllerWalkToGoalCommand()
   {
   }

   @Override
   public void clear()
   {
      sequenceId = 0;
      goalPose.setToNaN();
      timeToReachGoal = Double.NaN;
      holdPosition = false;
      positionProximity = 0.02;
      orientationProximity = 0.1;
   }

   @Override
   public void setFromMessage(ControllerWalkToGoalMessage message)
   {
      goalPose.setX(message.getXPosition());
      goalPose.setY(message.getYPosition());
      goalPose.setYaw(message.getYaw());
      timeToReachGoal = message.getTimeToReach();
      sequenceId = message.getSequenceId();
      holdPosition = message.getHoldPosition();
      positionProximity = message.getPositionProximity();
      orientationProximity = message.getOrientationProximity();
   }

   @Override
   public Class<ControllerWalkToGoalMessage> getMessageClass()
   {
      return ControllerWalkToGoalMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return true;
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }

   @Override
   public void set(ControllerWalkToGoalCommand other)
   {
      sequenceId = other.sequenceId;
      goalPose.set(other.goalPose);
      timeToReachGoal = other.timeToReachGoal;
      holdPosition = other.holdPosition;
      positionProximity = other.positionProximity;
      orientationProximity = other.orientationProximity;
   }

   public void setSequenceId(long sequenceId)
   {
      this.sequenceId = sequenceId;
   }

   public void setGoalPose(Pose2DReadOnly goalPose)
   {
      this.goalPose.set(goalPose);
   }

   public void setTimeToReachGoal(double timeToReachGoal)
   {
      this.timeToReachGoal = timeToReachGoal;
   }

   public FramePose2DReadOnly getGoalPose()
   {
      return goalPose;
   }

   public boolean getShouldHoldPosition()
   {
      return holdPosition;
   }

   public double getPositionProximity()
   {
      return positionProximity;
   }

   public double getOrientationProximity()
   {
      return orientationProximity;
   }

   public double getTimeToReachGoal()
   {
      return timeToReachGoal;
   }

   public void setPositionProximity(double positionProximity)
   {
      this.positionProximity = positionProximity;
   }

   public void setOrientationProximity(double orientationProximity)
   {
      this.orientationProximity = orientationProximity;
   }

   public void setShouldHoldPosition(boolean shouldHoldPosition)
   {
      this.holdPosition = shouldHoldPosition;
   }
}
