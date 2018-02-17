package us.ihmc.humanoidRobotics.communication.packets.sensing;

import us.ihmc.communication.packets.Packet;

public class StateEstimatorModePacket extends Packet<StateEstimatorModePacket>
{
   public byte requestedStateEstimatorMode;

   public StateEstimatorModePacket()
   {
   }

   @Override
   public void set(StateEstimatorModePacket other)
   {
      requestedStateEstimatorMode = other.requestedStateEstimatorMode;
      setPacketInformation(other);
   }

   public byte getRequestedOperatingMode()
   {
      return requestedStateEstimatorMode;
   }

   public void setRequestedOperatingMode(byte requestedOperatingMode)
   {
      this.requestedStateEstimatorMode = requestedOperatingMode;
   }

   @Override
   public boolean epsilonEquals(StateEstimatorModePacket other, double epsilon)
   {
      boolean ret = requestedStateEstimatorMode == other.requestedStateEstimatorMode;

      return ret;
   }

}
