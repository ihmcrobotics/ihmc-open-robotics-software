package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packets.SettablePacket;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;

@RosMessagePacket(documentation = "This class is used to report the status of walking.",
      rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE,
      topic = "/output/walking_status")
public class WalkingStatusMessage extends SettablePacket<WalkingStatusMessage>
{
   @RosExportedField(documentation = "Status of walking. Either STARTED, COMPLETED, or ABORT_REQUESTED.")
   public byte status;

   public WalkingStatusMessage()
   {
   }

   @Override
   public void set(WalkingStatusMessage other)
   {
      status = other.status;
      setPacketInformation(other);
   }

   public void setWalkingStatus(byte status)
   {
      this.status = status;
   }

   public byte getWalkingStatus()
   {
      return status;
   }

   @Override
   public boolean epsilonEquals(WalkingStatusMessage other, double epsilon)
   {
      return status == other.status;
   }
}
