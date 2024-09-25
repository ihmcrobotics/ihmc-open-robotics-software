package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.ObjectCarryMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.robotics.robotSide.RobotSide;

public class ObjectCarryCommand implements Command<ObjectCarryCommand, ObjectCarryMessage>
{
   private RobotSide robotSide;
   private double objectMass;
   private boolean isPickingUp;

   @Override
   public void clear()
   {
      robotSide = null;
      objectMass = 0.0;
      isPickingUp = false;
   }

   @Override
   public void setFromMessage(ObjectCarryMessage message)
   {
      robotSide = RobotSide.fromByte(message.getRobotSide());
      objectMass = message.getObjectMass();
      isPickingUp = message.getIsPickingUp();
   }

   @Override
   public Class<ObjectCarryMessage> getMessageClass()
   {
      return ObjectCarryMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return true;
   }

   @Override
   public long getSequenceId()
   {
      return 0;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public double getObjectMass()
   {
      return objectMass;
   }

   public boolean isPickingUp()
   {
      return isPickingUp;
   }

   @Override
   public void set(ObjectCarryCommand other)
   {
      this.robotSide = other.robotSide;
      this.objectMass = other.objectMass;
      this.isPickingUp = other.isPickingUp;
   }
}
