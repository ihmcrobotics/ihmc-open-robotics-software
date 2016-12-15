package us.ihmc.quadrupedRobotics.params;

import java.util.List;

import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.dataStructures.parameter.Parameter;

public class ParameterListPacket extends Packet<ParameterListPacket>
{
   private List<Parameter> parameters;

   public ParameterListPacket() // no-arg for serialization
   {
   }

   public ParameterListPacket(List<Parameter> parameters)
   {
      this.parameters = parameters;
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
