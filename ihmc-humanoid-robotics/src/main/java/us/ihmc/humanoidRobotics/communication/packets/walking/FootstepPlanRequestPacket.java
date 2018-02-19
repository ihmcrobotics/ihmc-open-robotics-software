package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.idl.PreallocatedList;

public class FootstepPlanRequestPacket extends Packet<FootstepPlanRequestPacket>
{
   public static final byte FOOTSTEP_PLAN_REQUEST_TYPE_START_SEARCH = 0;
   public static final byte FOOTSTEP_PLAN_REQUEST_TYPE_STOP_SEARCH = 1;
   public static final byte FOOTSTEP_PLAN_REQUEST_TYPE_UPDATE_START = 2;

   public FootstepDataMessage startFootstep;
   public double thetaStart;
   public double maxSuboptimality = 1;

   public PreallocatedList<FootstepDataMessage> goals = new PreallocatedList<>(FootstepDataMessage.class, FootstepDataMessage::new, 20);

   public byte footstepPlanRequestType;

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
      MessageTools.copyData(other.goals, goals);
      footstepPlanRequestType = other.footstepPlanRequestType;

      setPacketInformation(other);
   }

   @Override
   public boolean epsilonEquals(FootstepPlanRequestPacket other, double epsilon)
   {
      if (this.footstepPlanRequestType != other.footstepPlanRequestType)
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
