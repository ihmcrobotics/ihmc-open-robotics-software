package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.PrepareForLocomotionMessage;
import us.ihmc.communication.controllerAPI.command.Command;

public class PrepareForLocomotionCommand implements Command<PrepareForLocomotionCommand, PrepareForLocomotionMessage>
{
   private long sequenceId;
   private boolean prepareManipulation = true;
   private boolean preparePelvis = true;

   public PrepareForLocomotionCommand()
   {
   }

   @Override
   public void clear()
   {
      sequenceId = 0;
      prepareManipulation = true;
      preparePelvis = true;
   }

   @Override
   public void set(PrepareForLocomotionCommand other)
   {
      sequenceId = other.sequenceId;
      prepareManipulation = other.prepareManipulation;
      preparePelvis = other.preparePelvis;
   }

   @Override
   public void setFromMessage(PrepareForLocomotionMessage message)
   {
      sequenceId = message.getSequenceId();
      prepareManipulation = message.getPrepareManipulation();
      preparePelvis = message.getPreparePelvis();

   }

   /**
    * Specifies if the arm controller should be switching to chest frame or jointspace only if
    * necessary. This is particularly useful when manipulation was performed with respect to world
    * during standing to prevent "leaving a hand behind" when the robot starts walking.
    *
    * @param prepareManipulation whether the manipulation control should get prepared for walking.
    */
   public void setPrepareManipulation(boolean prepareManipulation)
   {
      this.prepareManipulation = prepareManipulation;
   }

   /**
    * Specifies if the pelvis orientation controller should be initialized before starting to walk.
    * When the controller is initialized, the pelvis will smoothly cancel out the user orientation
    * offset on the first transfer of a walking sequence.
    *
    * @param preparePelvis whether the pelvis orientation control should get prepared for walking.
    */
   public void setPreparePelvis(boolean preparePelvis)
   {
      this.preparePelvis = preparePelvis;
   }

   public boolean isPrepareManipulation()
   {
      return prepareManipulation;
   }

   public boolean isPreparePelvis()
   {
      return preparePelvis;
   }

   @Override
   public Class<PrepareForLocomotionMessage> getMessageClass()
   {
      return PrepareForLocomotionMessage.class;
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
}
