package us.ihmc.communication.packets;

import com.google.common.math.DoubleMath;

public class SetDoubleParameterPacket extends Packet<SetDoubleParameterPacket>
{
   private final String parameterName;
   private final double parameterValue;

   // Empty constructor for serialization
   public SetDoubleParameterPacket()
   {
      this(null, Double.NaN);
   }

   public SetDoubleParameterPacket(String parameterName, double parameterValue)
   {
      this.parameterName = parameterName;
      this.parameterValue = parameterValue;
   }

   public String getParameterName()
   {
      return parameterName;
   }

   public double getParameterValue()
   {
      return parameterValue;
   }

   @Override
   public boolean epsilonEquals(SetDoubleParameterPacket other, double epsilon)
   {
      return parameterName.equals(other.parameterName) && DoubleMath.fuzzyEquals(parameterValue, other.parameterValue, 1e-12);
   }
}
