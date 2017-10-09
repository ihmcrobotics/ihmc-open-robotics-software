package us.ihmc.quadrupedRobotics.communication.packets;

import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedStep;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class QuadrupedTimedStepPacket extends Packet<QuadrupedTimedStepPacket>
{
   private final ArrayList<QuadrupedTimedStep> steps;
   private final boolean isExpressedInAbsoluteTime;

   public QuadrupedTimedStepPacket()
   {
      this(Collections.<QuadrupedTimedStep> emptyList());
   }

   public QuadrupedTimedStepPacket(boolean isExpressedInAbsoluteTime)
   {
      this(Collections.<QuadrupedTimedStep> emptyList(), isExpressedInAbsoluteTime);
   }

   public QuadrupedTimedStepPacket(List<QuadrupedTimedStep> steps)
   {
      this(steps, true);
   }

   public QuadrupedTimedStepPacket(List<QuadrupedTimedStep> steps, boolean isExpressedInAbsoluteTime)
   {
      this.steps = new ArrayList<>(steps);
      this.isExpressedInAbsoluteTime = isExpressedInAbsoluteTime;
      setDestination(PacketDestination.CONTROLLER);
   }

   public int size()
   {
      return steps.size();
   }

   public ArrayList<QuadrupedTimedStep> getSteps()
   {
      return steps;
   }

   public boolean isExpressedInAbsoluteTime()
   {
      return isExpressedInAbsoluteTime;
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

      if (isExpressedInAbsoluteTime() != other.isExpressedInAbsoluteTime)
         return false;

      if (this.steps.size() != other.steps.size())
         return false;

      for (int i = 0; i < this.steps.size(); i++)
      {
         if (!this.steps.get(i).epsilonEquals(other.steps.get(i), epsilon))
            return false;
      }

      return true;
   }
}
