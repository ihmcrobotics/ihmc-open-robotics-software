package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class ValkyrieFootstepPlanningActionPacket extends Packet<ValkyrieFootstepPlanningActionPacket> implements Settable<ValkyrieFootstepPlanningActionPacket>, EpsilonComparable<ValkyrieFootstepPlanningActionPacket>
{
   /**
          * Halts active plan
          */
   public static final byte PLANNING_ACTION_HALT = (byte) 0;
   /**
          * Returns default planner parameters
          */
   public static final byte PLANNING_ACTION_REQUEST_PARAMETERS = (byte) 1;
   public byte planner_action_ = (byte) 255;

   public ValkyrieFootstepPlanningActionPacket()
   {
   }

   public ValkyrieFootstepPlanningActionPacket(ValkyrieFootstepPlanningActionPacket other)
   {
      this();
      set(other);
   }

   public void set(ValkyrieFootstepPlanningActionPacket other)
   {
      planner_action_ = other.planner_action_;

   }

   public void setPlannerAction(byte planner_action)
   {
      planner_action_ = planner_action;
   }
   public byte getPlannerAction()
   {
      return planner_action_;
   }


   public static Supplier<ValkyrieFootstepPlanningActionPacketPubSubType> getPubSubType()
   {
      return ValkyrieFootstepPlanningActionPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ValkyrieFootstepPlanningActionPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ValkyrieFootstepPlanningActionPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.planner_action_, other.planner_action_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ValkyrieFootstepPlanningActionPacket)) return false;

      ValkyrieFootstepPlanningActionPacket otherMyClass = (ValkyrieFootstepPlanningActionPacket) other;

      if(this.planner_action_ != otherMyClass.planner_action_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ValkyrieFootstepPlanningActionPacket {");
      builder.append("planner_action=");
      builder.append(this.planner_action_);
      builder.append("}");
      return builder.toString();
   }
}
