package toolbox_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class FootstepPlannerRejectionReasonMessage extends Packet<FootstepPlannerRejectionReasonMessage> implements Settable<FootstepPlannerRejectionReasonMessage>, EpsilonComparable<FootstepPlannerRejectionReasonMessage>
{
   /**
            * Reason ordinal
            */
   public long reason_;
   /**
            * Rejection percentage
            */
   public float rejection_percentage_;

   public FootstepPlannerRejectionReasonMessage()
   {
   }

   public FootstepPlannerRejectionReasonMessage(FootstepPlannerRejectionReasonMessage other)
   {
      this();
      set(other);
   }

   public void set(FootstepPlannerRejectionReasonMessage other)
   {
      reason_ = other.reason_;

      rejection_percentage_ = other.rejection_percentage_;

   }

   /**
            * Reason ordinal
            */
   public void setReason(long reason)
   {
      reason_ = reason;
   }
   /**
            * Reason ordinal
            */
   public long getReason()
   {
      return reason_;
   }

   /**
            * Rejection percentage
            */
   public void setRejectionPercentage(float rejection_percentage)
   {
      rejection_percentage_ = rejection_percentage;
   }
   /**
            * Rejection percentage
            */
   public float getRejectionPercentage()
   {
      return rejection_percentage_;
   }


   public static Supplier<FootstepPlannerRejectionReasonMessagePubSubType> getPubSubType()
   {
      return FootstepPlannerRejectionReasonMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FootstepPlannerRejectionReasonMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FootstepPlannerRejectionReasonMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.reason_, other.reason_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.rejection_percentage_, other.rejection_percentage_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FootstepPlannerRejectionReasonMessage)) return false;

      FootstepPlannerRejectionReasonMessage otherMyClass = (FootstepPlannerRejectionReasonMessage) other;

      if(this.reason_ != otherMyClass.reason_) return false;

      if(this.rejection_percentage_ != otherMyClass.rejection_percentage_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FootstepPlannerRejectionReasonMessage {");
      builder.append("reason=");
      builder.append(this.reason_);      builder.append(", ");
      builder.append("rejection_percentage=");
      builder.append(this.rejection_percentage_);
      builder.append("}");
      return builder.toString();
   }
}
