package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packets.SettablePacket;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;

@RosMessagePacket(documentation = "This class is used to report the status of walking.",
      rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE,
      topic = "/output/walking_status")
public class WalkingStatusMessage extends SettablePacket<WalkingStatusMessage>
{
   public static final byte STARTED = 0;
   public static final byte COMPLETED = 1;
   public static final byte ABORT_REQUESTED = 2;

   @RosExportedField(documentation = "Status of walking. Either STARTED, COMPLETED, or ABORT_REQUESTED.")
   public byte walkingStatus;

   public WalkingStatusMessage()
   {
   }

   @Override
   public void set(WalkingStatusMessage other)
   {
      walkingStatus = other.walkingStatus;
      setPacketInformation(other);
   }

   public void setWalkingStatus(byte status)
   {
      this.walkingStatus = status;
   }

   public byte getWalkingStatus()
   {
      return walkingStatus;
   }

   @Override
   public boolean epsilonEquals(WalkingStatusMessage other, double epsilon)
   {
      return walkingStatus == other.walkingStatus;
   }
}
