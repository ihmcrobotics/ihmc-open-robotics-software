package us.ihmc.quadrupedRobotics.communication.packets;

import us.ihmc.communication.packets.Packet;
import us.ihmc.quadrupedRobotics.planning.QuadrupedXGaitSettings;
import us.ihmc.quadrupedRobotics.planning.QuadrupedXGaitSettingsReadOnly;

public class QuadrupedXGaitSettingsPacket extends Packet<QuadrupedXGaitSettingsPacket>
{
   public QuadrupedXGaitSettings xGaitSettings;

   public QuadrupedXGaitSettingsPacket()
   {
   }

   public QuadrupedXGaitSettingsPacket(QuadrupedXGaitSettingsReadOnly xGaitSettings)
   {
      this.xGaitSettings = new QuadrupedXGaitSettings();
      this.xGaitSettings.set(xGaitSettings);
   }

   @Override
   public void set(QuadrupedXGaitSettingsPacket other)
   {
      xGaitSettings.set(other.xGaitSettings);
      setPacketInformation(other);
   }

   public QuadrupedXGaitSettings get()
   {
      return xGaitSettings;
   }

   @Override
   public boolean epsilonEquals(QuadrupedXGaitSettingsPacket other, double epsilon)
   {
      return this.xGaitSettings.epsilonEquals(other.xGaitSettings, epsilon);
   }
}
