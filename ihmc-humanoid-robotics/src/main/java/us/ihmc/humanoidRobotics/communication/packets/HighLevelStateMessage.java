package us.ihmc.humanoidRobotics.communication.packets;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;

@RosMessagePacket(documentation = "This message is used to switch the control scheme between force and position control.\n"
      + "WARNING: When in position control, the IHMC balance algorithms will be disabled and\n"
      + "it is up to the user to ensure stability.", rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE, topic = "/control/high_level_state")
public class HighLevelStateMessage extends Packet<HighLevelStateMessage>
{
   @RosExportedField(documentation = "The enum value of the current high level state of the robot.")
   public byte highLevelControllerName;

   public HighLevelStateMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public byte getHighLevelControllerName()
   {
      return highLevelControllerName;
   }

   @Override
   public void set(HighLevelStateMessage other)
   {
      highLevelControllerName = other.highLevelControllerName;
      setPacketInformation(other);
   }

   @Override
   public boolean equals(Object obj)
   {
      return ((obj instanceof HighLevelStateMessage) && this.epsilonEquals((HighLevelStateMessage) obj, 0));
   }

   @Override
   public String toString()
   {
      return "State= " + HighLevelControllerName.fromByte(highLevelControllerName).toString();
   }

   @Override
   public boolean epsilonEquals(HighLevelStateMessage other, double epsilon)
   {
      return highLevelControllerName == other.highLevelControllerName;
   }
}
