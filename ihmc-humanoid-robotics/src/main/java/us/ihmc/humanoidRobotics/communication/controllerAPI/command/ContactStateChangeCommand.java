package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.ContactStateChangeMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.EpsilonComparable;

public class ContactStateChangeCommand implements Command<ContactStateChangeCommand, ContactStateChangeMessage>, EpsilonComparable<ContactStateChangeCommand>
{
   private long sequenceId;
   private int hashCode;
   private boolean addContactPoint;

   @Override
   public void clear()
   {
      sequenceId = 0;
      hashCode = -1;
      addContactPoint = false;
   }

   @Override
   public void setFromMessage(ContactStateChangeMessage message)
   {
      this.sequenceId = message.getSequenceId();
      this.hashCode = message.getRigidBodyHashCode();
      this.addContactPoint = message.getAddContactPoint();
   }

   @Override
   public Class<ContactStateChangeMessage> getMessageClass()
   {
      return ContactStateChangeMessage.class;
   }

   public int getRigidBodyHashcode()
   {
      return hashCode;
   }

   public boolean getAddContactPoint()
   {
      return addContactPoint;
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
   public boolean epsilonEquals(ContactStateChangeCommand other, double epsilon)
   {
      return (sequenceId == other.sequenceId) && (hashCode == other.hashCode) && (addContactPoint == other.addContactPoint);
   }

   @Override
   public void set(ContactStateChangeCommand other)
   {
      this.sequenceId = other.sequenceId;
      this.hashCode = other.hashCode;
      this.addContactPoint = other.addContactPoint;
   }
}
