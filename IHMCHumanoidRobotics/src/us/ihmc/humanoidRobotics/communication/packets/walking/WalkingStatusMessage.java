package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.annotations.ros.RosMessagePacket;
import us.ihmc.communication.annotations.ros.RosExportedField;
import us.ihmc.communication.packets.StatusPacket;
import us.ihmc.tools.DocumentedEnum;

@RosMessagePacket(documentation = "This class is used to report the status of walking.",
      rosPackage = "ihmc_msgs",
      topic = "/output/walking_status")
public class WalkingStatusMessage extends StatusPacket<WalkingStatusMessage>
{
   public enum Status implements DocumentedEnum<Status>
   {
      STARTED, COMPLETED, ABORT_REQUESTED;

      public static final Status[] values = values();

      @Override
      public String getDocumentation(Status var)
      {
         switch (var)
         {
         case STARTED:
            return "The robot has begun its initial transfer/sway at the start of a walking plan";
         case COMPLETED:
            return "The robot has finished its final transfer/sway at the end of a walking plan";
         case ABORT_REQUESTED:
            return "A walking abort has been requested";
         default:
            return "Shouldn't get here";
         }
      }

      @Override
      public Status[] getDocumentedValues()
      {
         return values;
      }
   }

   @RosExportedField(documentation = "Status of walking. Either STARTED, COMPLETED, or ABORT_REQUESTED.")
   public Status status;

   public WalkingStatusMessage()
   {
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
