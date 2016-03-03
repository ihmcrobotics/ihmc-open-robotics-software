package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packets.StatusPacket;

public class WalkingStatusMessage extends StatusPacket<WalkingStatusMessage>
{
   public enum Status {STARTED, COMPLETED, ABORT_REQUESTED}

   private Status status;

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
