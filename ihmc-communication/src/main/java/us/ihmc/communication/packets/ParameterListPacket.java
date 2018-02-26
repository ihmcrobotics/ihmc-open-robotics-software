package us.ihmc.communication.packets;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.robotics.dataStructures.parameter.Parameter;

public class ParameterListPacket extends Packet<ParameterListPacket>
{
   public List<Parameter> parameters;

   public ParameterListPacket() // no-arg for serialization
   {
   }

   @Override
   public void set(ParameterListPacket other)
   {
      parameters = new ArrayList<>();
      parameters.addAll(other.parameters);
      setPacketInformation(other);
   }

   public List<Parameter> getParameters()
   {
      return parameters;
   }

   @Override
   public boolean epsilonEquals(ParameterListPacket other, double epsilon)
   {
      if (parameters.size() != other.parameters.size())
      {
         return false;
      }

      for(int i = 0; i < parameters.size(); i++)
      {
         if (!parameters.get(i).equals(other.parameters.get(i)))
         {
            return false;
         }
      }

      return true;
   }
}
