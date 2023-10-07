package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class FootstepPlanActionFootstepStateMessage extends Packet<FootstepPlanActionFootstepStateMessage> implements Settable<FootstepPlanActionFootstepStateMessage>, EpsilonComparable<FootstepPlanActionFootstepStateMessage>
{
   /**
            * Index of the footstep
            */
   public int index_;

   public FootstepPlanActionFootstepStateMessage()
   {
   }

   public FootstepPlanActionFootstepStateMessage(FootstepPlanActionFootstepStateMessage other)
   {
      this();
      set(other);
   }

   public void set(FootstepPlanActionFootstepStateMessage other)
   {
      index_ = other.index_;

   }

   /**
            * Index of the footstep
            */
   public void setIndex(int index)
   {
      index_ = index;
   }
   /**
            * Index of the footstep
            */
   public int getIndex()
   {
      return index_;
   }


   public static Supplier<FootstepPlanActionFootstepStateMessagePubSubType> getPubSubType()
   {
      return FootstepPlanActionFootstepStateMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FootstepPlanActionFootstepStateMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FootstepPlanActionFootstepStateMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.index_, other.index_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FootstepPlanActionFootstepStateMessage)) return false;

      FootstepPlanActionFootstepStateMessage otherMyClass = (FootstepPlanActionFootstepStateMessage) other;

      if(this.index_ != otherMyClass.index_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FootstepPlanActionFootstepStateMessage {");
      builder.append("index=");
      builder.append(this.index_);
      builder.append("}");
      return builder.toString();
   }
}
