package us.ihmc.communication.packets;

import us.ihmc.commons.MathTools;

public class PilotAlarmPacket extends Packet<PilotAlarmPacket>
{
   public double beepRate;
   private boolean enableTone;
   
   @Override
   public boolean epsilonEquals(PilotAlarmPacket other, double epsilon)
   {
      return MathTools.epsilonEquals(beepRate, other.beepRate, epsilon);
   }

   public double getBeepRate()
   {
      return beepRate;
   }

   public void setBeepRate(double beepRate)
   {
      this.beepRate = beepRate;
   }

   public boolean isToneEnabled()
   {
      return enableTone;
   }

   public void setEnableTone(boolean enableTone)
   {
      this.enableTone = enableTone;
   }

}
