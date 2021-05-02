package us.ihmc.humanoidRobotics.communication.directionalControlToolboxAPI;

import controller_msgs.msg.dds.DirectionalControlConfigurationMessage;
import us.ihmc.communication.controllerAPI.command.Command;

public class DirectionalControlConfigurationCommand implements Command<DirectionalControlConfigurationCommand, DirectionalControlConfigurationMessage>
{
   private long sequenceId;
   private boolean enableWalking;
   private String profileName;

   /**
    * This is essentially an assignment operator. When a command is copied from the internal buffer for
    * use by a toolbox controller, this function is called. It should copy in every member of the
    * class.
    */
   @Override
   public void set(DirectionalControlConfigurationCommand other)
   {
      clear();

      sequenceId = other.sequenceId;
      enableWalking = other.enableWalking;
      profileName = other.profileName;
   }

   @Override
   public void clear()
   {
      sequenceId = 0;
      enableWalking = false;
      profileName = "";
   }

   @Override
   public void setFromMessage(DirectionalControlConfigurationMessage message)
   {
      set(message);
   }

   public void set(DirectionalControlConfigurationMessage message)
   {
      clear();
      sequenceId = message.getSequenceId();
      enableWalking = message.getEnableWalking();
      profileName = message.getProfileNameAsString();
   }

   @Override
   public Class<DirectionalControlConfigurationMessage> getMessageClass()
   {
      return DirectionalControlConfigurationMessage.class;
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

   public boolean getEnableWalking()
   {
      return enableWalking;
   }

   public String getProfileName()
   {
      return profileName;
   }

   public String toString()
   {
      return this.getClass().getSimpleName() + ": " + "{ sequenceId = " + String.valueOf(getSequenceId()) + ", enableWalking = "
            + String.valueOf(getEnableWalking()) + ", profileName = '" + String.valueOf(getProfileName()) + "' }";
   }
}
