package us.ihmc.humanoidRobotics.communication.packets.sensing;

import us.ihmc.communication.packets.Packet;

public class StateEstimatorModePacket extends Packet<StateEstimatorModePacket>
{
   public enum StateEstimatorMode
   {
      NORMAL, FROZEN
   }

   public StateEstimatorMode requestedOperatingMode;

   public StateEstimatorModePacket()
   {
   }

   @Override
   public void set(StateEstimatorModePacket other)
   {
      requestedOperatingMode = other.requestedOperatingMode;
      setPacketInformation(other);
   }

   public StateEstimatorMode getRequestedOperatingMode()
   {
      return requestedOperatingMode;
   }

   public void setRequestedOperatingMode(StateEstimatorMode requestedOperatingMode)
   {
      this.requestedOperatingMode = requestedOperatingMode;
   }

   @Override
   public boolean epsilonEquals(StateEstimatorModePacket other, double epsilon)
   {
      boolean ret = requestedOperatingMode == other.requestedOperatingMode;

      return ret;
   }

}
