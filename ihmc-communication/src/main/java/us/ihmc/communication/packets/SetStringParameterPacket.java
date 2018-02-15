package us.ihmc.communication.packets;

public class SetStringParameterPacket extends Packet<SetStringParameterPacket>
{
   public String parameterName;
   public String parameterValue;

   // Empty constructor for serialization
   public SetStringParameterPacket()
   {
   }

   @Override
   public void set(SetStringParameterPacket other)
   {
      parameterName = other.parameterName;
      parameterValue = other.parameterValue;
      setPacketInformation(other);
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
