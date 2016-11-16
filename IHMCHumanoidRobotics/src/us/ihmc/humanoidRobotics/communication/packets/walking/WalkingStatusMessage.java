package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.ros.generators.RosEnumValueDocumentation;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.packets.StatusPacket;
import us.ihmc.robotics.random.RandomTools;

import java.util.Random;

@RosMessagePacket(documentation = "This class is used to report the status of walking.",
      rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE,
      topic = "/output/walking_status")
public class WalkingStatusMessage extends StatusPacket<WalkingStatusMessage>
{
   public enum Status
   {
      @RosEnumValueDocumentation(documentation = "The robot has begun its initial transfer/sway at the start of a walking plan")
      STARTED,
      @RosEnumValueDocumentation(documentation = "The robot has finished its final transfer/sway at the end of a walking plan")
      COMPLETED,
      @RosEnumValueDocumentation(documentation = "A walking abort has been requested")
      ABORT_REQUESTED;

      public static final Status[] values = values();
   }

   @RosExportedField(documentation = "Status of walking. Either STARTED, COMPLETED, or ABORT_REQUESTED.")
   public Status status;

   public WalkingStatusMessage()
   {
   }

   public WalkingStatusMessage(Random random)
   {
      status = RandomTools.generateRandomEnum(random, Status.class);
   }

   public WalkingStatusMessage(Status status)
   {
      this.status = status;
   }

   @Override
   public void set(WalkingStatusMessage other)
   {
      status = other.status;
   }

   public void setWalkingStatus(Status status)
   {
      this.status = status;
   }

   public Status getWalkingStatus()
   {
      return status;
   }

   @Override
   public boolean epsilonEquals(WalkingStatusMessage other, double epsilon)
   {
      return status == other.status;
   }
}
