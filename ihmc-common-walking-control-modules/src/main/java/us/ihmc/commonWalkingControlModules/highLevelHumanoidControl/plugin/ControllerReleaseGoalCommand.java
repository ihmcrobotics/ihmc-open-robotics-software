package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin;

import controller_msgs.ControllerReleaseGoalMessage;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;

public class ControllerReleaseGoalCommand implements Command<ControllerReleaseGoalCommand, ControllerReleaseGoalMessage>
{
   private long sequenceId;
   private boolean releaseGoal;

   public ControllerReleaseGoalCommand()
   {
   }

   @Override
   public void clear()
   {
      sequenceId = 0;
      releaseGoal = false;
   }

   @Override
   public void setFromMessage(ControllerReleaseGoalMessage message)
   {
      this.releaseGoal = message.getReleaseGoal();
   }

   @Override
   public Class<ControllerReleaseGoalMessage> getMessageClass()
   {
      return ControllerReleaseGoalMessage.class;
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
   public void set(ControllerReleaseGoalCommand other)
   {
      sequenceId = other.sequenceId;
      this.releaseGoal = other.releaseGoal;
   }

   public void setSequenceId(long sequenceId)
   {
      this.sequenceId = sequenceId;
   }

   public boolean getReleaseGoal()
   {
      return releaseGoal;
   }
}
