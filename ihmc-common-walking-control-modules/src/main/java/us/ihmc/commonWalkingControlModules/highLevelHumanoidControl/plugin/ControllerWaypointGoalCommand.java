package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin;

import controller_msgs.msg.dds.ControllerWaypointGoalMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.geometry.interfaces.Pose2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;

public class ControllerWaypointGoalCommand implements Command<ControllerWaypointGoalCommand, ControllerWaypointGoalMessage>
{
   private long sequenceId;
   private final FramePose2D goalPose = new FramePose2D();
   private double timeToReachGoal = Double.NaN;
   private boolean holdPosition = false;

   private double positionProximity = 0.02;
   private double orientationProximity = 0.1;
   private boolean goalOrientationMatters = true;

   public ControllerWaypointGoalCommand()
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
      goalOrientationMatters = true;
   }

   @Override
   public void setFromMessage(ControllerWaypointGoalMessage message)
   {
      goalPose.setX(message.getXPosition());
      goalPose.setY(message.getYPosition());
      goalPose.setYaw(message.getYaw());
      timeToReachGoal = message.getTimeToReach();
      sequenceId = message.getSequenceId();
      holdPosition = message.getHoldPosition();
      positionProximity = message.getPositionProximity();
      orientationProximity = message.getOrientationProximity();
      goalOrientationMatters = message.getGoalOrientationMatters();
   }

   @Override
   public Class<ControllerWaypointGoalMessage> getMessageClass()
   {
      return ControllerWaypointGoalMessage.class;
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
   public void set(ControllerWaypointGoalCommand other)
   {
      sequenceId = other.sequenceId;
      goalPose.set(other.goalPose);
      timeToReachGoal = other.timeToReachGoal;
      holdPosition = other.holdPosition;
      positionProximity = other.positionProximity;
      orientationProximity = other.orientationProximity;
      goalOrientationMatters = other.goalOrientationMatters;
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

   public boolean getGoalOrientationMatters()
   {
      return goalOrientationMatters;
   }

   public void setGoalOrientationMatters(boolean goalOrientationMatters)
   {
      this.goalOrientationMatters = goalOrientationMatters;
   }
}
