package us.ihmc.quadrupedRobotics.communication.packets;

import us.ihmc.communication.packets.Packet;
import us.ihmc.quadrupedRobotics.planning.QuadrupedXGaitSettings;

import javax.vecmath.Vector3d;

public class QuadrupedXGaitSettingsPacket extends Packet<QuadrupedXGaitSettingsPacket>
{
   private final QuadrupedXGaitSettings xGaitSettings;

   public QuadrupedXGaitSettingsPacket()
   {
      this.xGaitSettings = new QuadrupedXGaitSettings();
   }

   public QuadrupedXGaitSettingsPacket(QuadrupedXGaitSettings xGaitSettings)
   {
      this.xGaitSettings = new QuadrupedXGaitSettings();
      this.xGaitSettings.set(xGaitSettings);
   }

   public QuadrupedXGaitSettings get()
   {
      return xGaitSettings;
   }

   public void get(QuadrupedXGaitSettings xGaitSettings)
   {
      xGaitSettings.set(this.xGaitSettings);
   }

   @Override public boolean epsilonEquals(QuadrupedXGaitSettingsPacket other, double epsilon)
   {
      return this.xGaitSettings.epsilonEquals(other.xGaitSettings, epsilon);
   }
}
