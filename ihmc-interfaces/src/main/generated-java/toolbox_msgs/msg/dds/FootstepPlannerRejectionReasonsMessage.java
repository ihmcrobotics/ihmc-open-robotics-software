package toolbox_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class FootstepPlannerRejectionReasonsMessage extends Packet<FootstepPlannerRejectionReasonsMessage> implements Settable<FootstepPlannerRejectionReasonsMessage>, EpsilonComparable<FootstepPlannerRejectionReasonsMessage>
{
   /**
            * Rejection reasons
            */
   public us.ihmc.idl.IDLSequence.Object<toolbox_msgs.msg.dds.FootstepPlannerRejectionReasonMessage>  rejection_reasons_;

   public FootstepPlannerRejectionReasonsMessage()
   {
      rejection_reasons_ = new us.ihmc.idl.IDLSequence.Object<toolbox_msgs.msg.dds.FootstepPlannerRejectionReasonMessage> (30, new toolbox_msgs.msg.dds.FootstepPlannerRejectionReasonMessagePubSubType());

   }

   public FootstepPlannerRejectionReasonsMessage(FootstepPlannerRejectionReasonsMessage other)
   {
      this();
      set(other);
   }

   public void set(FootstepPlannerRejectionReasonsMessage other)
   {
      rejection_reasons_.set(other.rejection_reasons_);
   }


   /**
            * Rejection reasons
            */
   public us.ihmc.idl.IDLSequence.Object<toolbox_msgs.msg.dds.FootstepPlannerRejectionReasonMessage>  getRejectionReasons()
   {
      return rejection_reasons_;
   }


   public static Supplier<FootstepPlannerRejectionReasonsMessagePubSubType> getPubSubType()
   {
      return FootstepPlannerRejectionReasonsMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FootstepPlannerRejectionReasonsMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FootstepPlannerRejectionReasonsMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (this.rejection_reasons_.size() != other.rejection_reasons_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.rejection_reasons_.size(); i++)
         {  if (!this.rejection_reasons_.get(i).epsilonEquals(other.rejection_reasons_.get(i), epsilon)) return false; }
      }

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FootstepPlannerRejectionReasonsMessage)) return false;

      FootstepPlannerRejectionReasonsMessage otherMyClass = (FootstepPlannerRejectionReasonsMessage) other;

      if (!this.rejection_reasons_.equals(otherMyClass.rejection_reasons_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FootstepPlannerRejectionReasonsMessage {");
      builder.append("rejection_reasons=");
      builder.append(this.rejection_reasons_);
      builder.append("}");
      return builder.toString();
   }
}
