package us.ihmc.quadrupedRobotics.packets;

import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedStep;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;

import java.util.ArrayList;
import java.util.List;

public class QuadrupedTimedStepPacket extends Packet<QuadrupedTimedStepPacket>
{
   public ArrayList<QuadrupedTimedStep> steps;

   public QuadrupedTimedStepPacket()
   {
      setDestination(PacketDestination.CONTROLLER);
      steps = new ArrayList<>();
   }

   public QuadrupedTimedStepPacket(List<QuadrupedTimedStep> steps)
   {
      this();
      this.steps = new ArrayList<>(steps.size());
      for (int i = 0; i < steps.size(); i++)
      {
         this.steps.set(i, steps.get(i));
      }
   }

   public int size()
   {
      return steps.size();
   }

   public QuadrupedTimedStep get(int i)
   {
      return steps.get(i);
   }

   public ArrayList<QuadrupedTimedStep> get()
   {
      return steps;
   }

   @Override
   public String toString()
   {
      String string = "steps: ";
      for (int i = 0; i < steps.size(); i++)
      {
         string += " ";
         string += steps.get(i).toString();
      }
      return string;
   }

   @Override
   public boolean epsilonEquals(QuadrupedTimedStepPacket other, double epsilon)
   {
      if(other == null)
         return false;

      if (this.steps.size() != other.steps.size())
         return false;

      for (int i = 0; i < this.steps.size(); i++)
      {
         if (!this.steps.get(i).epsilonEquals(other.steps.get(i), epsilon));
            return false;
      }
      return true;
   }
}
