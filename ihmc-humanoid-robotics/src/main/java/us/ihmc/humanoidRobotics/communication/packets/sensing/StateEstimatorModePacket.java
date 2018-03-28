package us.ihmc.humanoidRobotics.communication.packets.sensing;

import us.ihmc.communication.packets.Packet;

public class StateEstimatorModePacket extends Packet<StateEstimatorModePacket>
{
   public static final byte NORMAL = 0;
   public static final byte FROZEN = 1;

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

   public byte getRequestedStateEstimatorMode()
   {
      return requestedStateEstimatorMode;
   }

   public void setRequestedStateEstimatorMode(byte requestedOperatingMode)
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
