package us.ihmc.humanoidRobotics.communication.packets.sensing;

import us.ihmc.commons.MathTools;
import us.ihmc.communication.packets.Packet;

public class PelvisPoseErrorPacket extends Packet<PelvisPoseErrorPacket>
{
   public float residualError;
   public float totalError;
   public boolean hasMapBeenReset;

   public PelvisPoseErrorPacket()
   {
   }

   public PelvisPoseErrorPacket(float residualError, float totalError, boolean hasMapBeenReset)
   {
      this.residualError = residualError;
      this.totalError = totalError;
      this.hasMapBeenReset = hasMapBeenReset;
   }

   @Override
   public void set(PelvisPoseErrorPacket other)
   {
      residualError = other.residualError;
      totalError = other.totalError;
      hasMapBeenReset = other.hasMapBeenReset;
      setPacketInformation(other);
   }

   @Override
   public boolean epsilonEquals(PelvisPoseErrorPacket other, double epsilon)
   {
      return (MathTools.epsilonEquals(residualError, other.residualError, epsilon) && MathTools.epsilonEquals(totalError, other.totalError, epsilon)
            && other.hasMapBeenReset == hasMapBeenReset);
   }

}
