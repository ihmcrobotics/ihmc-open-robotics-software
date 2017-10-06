package us.ihmc.communication.packets;

public class SetStringParameterPacket extends Packet<SetStringParameterPacket>
{
   private final String parameterName;
   private final String parameterValue;

   // Empty constructor for serialization
   public SetStringParameterPacket()
   {
      this(null, "");
   }

   public SetStringParameterPacket(String parameterName, String parameterValue)
   {
      this.parameterName = parameterName;
      this.parameterValue = parameterValue;
   }

   public String getParameterName()
   {
      return parameterName;
   }

   public String getParameterValue()
   {
      return parameterValue;
   }

   @Override
   public boolean epsilonEquals(SetStringParameterPacket other, double epsilon)
   {
      return parameterName.equals(other.parameterName) && parameterValue.equals(other.parameterValue);
   }
}
