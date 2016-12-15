package us.ihmc.communication.packets;

import com.google.common.math.DoubleMath;

public class SetDoubleArrayParameterPacket extends Packet<SetDoubleArrayParameterPacket>
{
   private final String parameterName;
   private final double[] parameterValue;

   // Empty constructor for serialization
   public SetDoubleArrayParameterPacket()
   {
      this(null, new double[] { });
   }

   public SetDoubleArrayParameterPacket(String parameterName, double[] parameterValue)
   {
      this.parameterName = parameterName;
      this.parameterValue = parameterValue;
   }

   public String getParameterName()
   {
      return parameterName;
   }

   public double[] getParameterValue()
   {
      return parameterValue;
   }

   @Override
   public boolean epsilonEquals(SetDoubleArrayParameterPacket other, double epsilon)
   {
      if (!parameterName.equals(other.parameterName) || parameterValue.length != other.parameterValue.length)
      {
         return false;
      }

      for (int i = 0; i < parameterValue.length; i++)
      {
         if (!DoubleMath.fuzzyEquals(parameterValue[i], other.parameterValue[i], 1e-12))
         {
            return false;
         }
      }

      return true;
   }
}
