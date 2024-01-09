package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class FootstepPlanActionStateBasicsMessage extends Packet<FootstepPlanActionStateBasicsMessage> implements Settable<FootstepPlanActionStateBasicsMessage>, EpsilonComparable<FootstepPlanActionStateBasicsMessage>
{
   /**
            * Total number of footsteps; used for walking actions
            */
   public int total_number_of_footsteps_;
   /**
            * Incomplete footsteps; used for walking actions
            */
   public int number_of_incomplete_footsteps_;

   public FootstepPlanActionStateBasicsMessage()
   {
   }

   public FootstepPlanActionStateBasicsMessage(FootstepPlanActionStateBasicsMessage other)
   {
      this();
      set(other);
   }

   public void set(FootstepPlanActionStateBasicsMessage other)
   {
      total_number_of_footsteps_ = other.total_number_of_footsteps_;

      number_of_incomplete_footsteps_ = other.number_of_incomplete_footsteps_;

   }

   /**
            * Total number of footsteps; used for walking actions
            */
   public void setTotalNumberOfFootsteps(int total_number_of_footsteps)
   {
      total_number_of_footsteps_ = total_number_of_footsteps;
   }
   /**
            * Total number of footsteps; used for walking actions
            */
   public int getTotalNumberOfFootsteps()
   {
      return total_number_of_footsteps_;
   }

   /**
            * Incomplete footsteps; used for walking actions
            */
   public void setNumberOfIncompleteFootsteps(int number_of_incomplete_footsteps)
   {
      number_of_incomplete_footsteps_ = number_of_incomplete_footsteps;
   }
   /**
            * Incomplete footsteps; used for walking actions
            */
   public int getNumberOfIncompleteFootsteps()
   {
      return number_of_incomplete_footsteps_;
   }


   public static Supplier<FootstepPlanActionStateBasicsMessagePubSubType> getPubSubType()
   {
      return FootstepPlanActionStateBasicsMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FootstepPlanActionStateBasicsMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FootstepPlanActionStateBasicsMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.total_number_of_footsteps_, other.total_number_of_footsteps_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.number_of_incomplete_footsteps_, other.number_of_incomplete_footsteps_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FootstepPlanActionStateBasicsMessage)) return false;

      FootstepPlanActionStateBasicsMessage otherMyClass = (FootstepPlanActionStateBasicsMessage) other;

      if(this.total_number_of_footsteps_ != otherMyClass.total_number_of_footsteps_) return false;

      if(this.number_of_incomplete_footsteps_ != otherMyClass.number_of_incomplete_footsteps_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FootstepPlanActionStateBasicsMessage {");
      builder.append("total_number_of_footsteps=");
      builder.append(this.total_number_of_footsteps_);      builder.append(", ");
      builder.append("number_of_incomplete_footsteps=");
      builder.append(this.number_of_incomplete_footsteps_);
      builder.append("}");
      return builder.toString();
   }
}
