package us.ihmc.communication.packets;

public class SetBooleanParameterPacket extends Packet<SetBooleanParameterPacket>
{
   public String parameterName;
   public boolean parameterValue;

   // Empty constructor for serialization
   public SetBooleanParameterPacket()
   {
      this(null, false);
   }

   public SetBooleanParameterPacket(String parameterName, boolean parameterValue)
   {
      this.parameterName = parameterName;
      this.parameterValue = parameterValue;
   }

   @Override
   public void set(SetBooleanParameterPacket other)
   {
      parameterName = other.parameterName;
      parameterValue = other.parameterValue;
      setPacketInformation(other);
   }

   public String getParameterName()
   {
      return parameterName;
   }

   public boolean getParameterValue()
   {
      return parameterValue;
   }

   @Override
   public boolean epsilonEquals(SetBooleanParameterPacket other, double epsilon)
   {
      return parameterName.equals(other.parameterName) && parameterValue == other.parameterValue;
   }
}
