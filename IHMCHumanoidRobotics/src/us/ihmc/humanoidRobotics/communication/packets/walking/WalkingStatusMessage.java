package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packetAnnotations.FieldDocumentation;
import us.ihmc.communication.packets.StatusPacket;

@ClassDocumentation("This class is used to report the status of walking.")
public class WalkingStatusMessage extends StatusPacket<WalkingStatusMessage>
{
   public enum Status {STARTED, COMPLETED, ABORT_REQUESTED}

   @FieldDocumentation("Status of walking. Either STARTED, COMPLETED, or ABORT_REQUESTED.")
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
