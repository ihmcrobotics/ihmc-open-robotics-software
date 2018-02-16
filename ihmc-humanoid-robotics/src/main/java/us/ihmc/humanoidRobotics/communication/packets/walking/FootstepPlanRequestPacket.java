package us.ihmc.humanoidRobotics.communication.packets.walking;

import java.util.ArrayList;

import us.ihmc.communication.packets.Packet;

public class FootstepPlanRequestPacket extends Packet<FootstepPlanRequestPacket>
{

   public FootstepDataMessage startFootstep;
   public double thetaStart;
   public double maxSuboptimality = 1;

   public ArrayList<FootstepDataMessage> goals = new ArrayList<FootstepDataMessage>();

   public byte requestType;

   public FootstepPlanRequestPacket()
   {
      // for serialization.
   }

   @Override
   public void set(FootstepPlanRequestPacket other)
   {
      startFootstep = new FootstepDataMessage();
      startFootstep.set(other.startFootstep);
      thetaStart = other.thetaStart;
      maxSuboptimality = other.maxSuboptimality;

      goals = new ArrayList<>();
      for (int i = 0; i < other.goals.size(); i++)
      {
         FootstepDataMessage footstep = new FootstepDataMessage();
         footstep.set(other.goals.get(i));
         goals.add(footstep);
      }

      requestType = other.requestType;

      setPacketInformation(other);
   }

   @Override
   public boolean epsilonEquals(FootstepPlanRequestPacket other, double epsilon)
   {
      if (this.requestType != other.requestType)
         return false;
      if (Math.abs(this.thetaStart - other.thetaStart) > epsilon)
         return false;
      if (Math.abs(this.maxSuboptimality - other.maxSuboptimality) > epsilon)
         return false;
      if (!this.startFootstep.epsilonEquals(other.startFootstep, epsilon))
         return false;
      if (this.goals.size() != other.goals.size())
         return false;
      for (int i = 0; i < goals.size(); i++)
      {
         if (!goals.get(i).epsilonEquals(other.goals.get(i), epsilon))
            return false;
      }
      return true;
   }
}
