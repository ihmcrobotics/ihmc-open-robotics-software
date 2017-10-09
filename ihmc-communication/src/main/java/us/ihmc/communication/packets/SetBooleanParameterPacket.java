package us.ihmc.communication.packets;

public class SetBooleanParameterPacket extends Packet<SetBooleanParameterPacket>
{
   private final String parameterName;
   private final boolean parameterValue;

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
