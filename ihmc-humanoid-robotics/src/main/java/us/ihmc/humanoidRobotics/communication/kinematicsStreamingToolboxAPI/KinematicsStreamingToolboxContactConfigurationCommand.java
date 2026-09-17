package us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI;

import toolbox_msgs.KinematicsStreamingToolboxContactConfigurationMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class KinematicsStreamingToolboxContactConfigurationCommand implements Command<KinematicsStreamingToolboxContactConfigurationCommand, KinematicsStreamingToolboxContactConfigurationMessage>
{
   private long sequenceId;
   private final SideDependentList<Boolean> isFootInContact = new SideDependentList<>();

   @Override
   public void clear()
   {
      sequenceId = 0;
      for (RobotSide side : RobotSide.values)
         isFootInContact.put(side, true);
   }

   @Override
   public void set(KinematicsStreamingToolboxContactConfigurationCommand other)
   {
      clear();

      sequenceId = other.sequenceId;
      isFootInContact.set(other.isFootInContact);
   }

   @Override
   public void setFromMessage(KinematicsStreamingToolboxContactConfigurationMessage message)
   {
      clear();

      sequenceId = message.getSequenceId();
      isFootInContact.put(RobotSide.LEFT, message.getLeftFootInContact());
      isFootInContact.put(RobotSide.RIGHT, message.getRightFootInContact());
   }

   public SideDependentList<Boolean> getIsFootInContact()
   {
      return isFootInContact;
   }

   public boolean getIsFootInContact(RobotSide side)
   {
      return isFootInContact.get(side);
   }

   @Override
   public Class<KinematicsStreamingToolboxContactConfigurationMessage> getMessageClass()
   {
      return KinematicsStreamingToolboxContactConfigurationMessage.class;
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
