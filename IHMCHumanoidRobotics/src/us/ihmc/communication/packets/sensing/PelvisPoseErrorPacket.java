package us.ihmc.communication.packets.sensing;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.MathTools;

public class PelvisPoseErrorPacket extends Packet<PelvisPoseErrorPacket> 
{
   public float residualError;
   public float totalError;   
   public boolean hasMapBeenReset;
   
	public PelvisPoseErrorPacket(Random random)
	{
	   residualError = random.nextFloat();
	   totalError = random.nextFloat();
	}
	
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
   public boolean epsilonEquals(PelvisPoseErrorPacket other, double epsilon)
   {
      return (MathTools.epsilonEquals(residualError, other.residualError, epsilon)
         && MathTools.epsilonEquals(totalError, other.totalError, epsilon)
         && other.hasMapBeenReset == hasMapBeenReset);
	}

}
