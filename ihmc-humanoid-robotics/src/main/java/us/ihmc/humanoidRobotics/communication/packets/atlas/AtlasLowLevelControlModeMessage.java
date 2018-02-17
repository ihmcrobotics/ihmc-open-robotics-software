package us.ihmc.humanoidRobotics.communication.packets.atlas;

import us.ihmc.communication.packets.Packet;

public class AtlasLowLevelControlModeMessage extends Packet<AtlasLowLevelControlModeMessage>
{
   public byte requestedAtlasLowLevelControlMode;

   public AtlasLowLevelControlModeMessage()
   {
   }

   @Override
   public void set(AtlasLowLevelControlModeMessage other)
   {
      setPacketInformation(other);
      requestedAtlasLowLevelControlMode = other.requestedAtlasLowLevelControlMode;
   }

   public void setRequestedControlMode(byte requestedControlMode)
   {
      this.requestedAtlasLowLevelControlMode = requestedControlMode;
   }

   public byte getRequestedControlMode()
   {
      return requestedAtlasLowLevelControlMode;
   }

   @Override
   public boolean epsilonEquals(AtlasLowLevelControlModeMessage other, double epsilon)
   {
      return requestedAtlasLowLevelControlMode == other.requestedAtlasLowLevelControlMode;
   }
}
