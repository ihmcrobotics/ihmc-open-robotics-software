package us.ihmc.humanoidRobotics.communication.packets.atlas;

import us.ihmc.communication.packets.Packet;

public class AtlasLowLevelControlModeMessage extends Packet<AtlasLowLevelControlModeMessage>
{
   public byte requestedControlMode;

   public AtlasLowLevelControlModeMessage()
   {
   }

   @Override
   public void set(AtlasLowLevelControlModeMessage other)
   {
      setPacketInformation(other);
      requestedControlMode = other.requestedControlMode;
   }

   public void setRequestedControlMode(byte requestedControlMode)
   {
      this.requestedControlMode = requestedControlMode;
   }

   public byte getRequestedControlMode()
   {
      return requestedControlMode;
   }

   @Override
   public boolean epsilonEquals(AtlasLowLevelControlModeMessage other, double epsilon)
   {
      return requestedControlMode == other.requestedControlMode;
   }
}
