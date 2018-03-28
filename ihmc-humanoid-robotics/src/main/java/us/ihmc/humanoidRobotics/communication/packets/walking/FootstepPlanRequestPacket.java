package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.idl.RecyclingArrayListPubSub;

public class FootstepPlanRequestPacket extends Packet<FootstepPlanRequestPacket>
{
   public static final byte FOOTSTEP_PLAN_REQUEST_TYPE_START_SEARCH = 0;
   public static final byte FOOTSTEP_PLAN_REQUEST_TYPE_STOP_SEARCH = 1;
   public static final byte FOOTSTEP_PLAN_REQUEST_TYPE_UPDATE_START = 2;

   public FootstepDataMessage startFootstep = new FootstepDataMessage();
   public double thetaStart;
   public double maxSuboptimality = 1;

   public RecyclingArrayListPubSub<FootstepDataMessage> goals = new RecyclingArrayListPubSub<>(FootstepDataMessage.class, FootstepDataMessage::new, 5);

   public byte footstepPlanRequestType;

   public FootstepPlanRequestPacket()
   {
      // for serialization.
   }

   @Override
   public void set(FootstepPlanRequestPacket other)
   {
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
      if (!MessageTools.epsilonEquals(goals, other.goals, epsilon))
         return false;
      return true;
   }
}
