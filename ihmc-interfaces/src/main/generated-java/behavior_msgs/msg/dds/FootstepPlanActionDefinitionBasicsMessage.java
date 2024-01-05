package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class FootstepPlanActionDefinitionBasicsMessage extends Packet<FootstepPlanActionDefinitionBasicsMessage> implements Settable<FootstepPlanActionDefinitionBasicsMessage>, EpsilonComparable<FootstepPlanActionDefinitionBasicsMessage>
{
   /**
            * Name of the parent frame the footsteps are expressed in
            */
   public java.lang.StringBuilder parent_frame_name_;
   /**
            * Swing duration
            */
   public double swing_duration_;
   /**
            * Transfer duration
            */
   public double transfer_duration_;

   public FootstepPlanActionDefinitionBasicsMessage()
   {
      parent_frame_name_ = new java.lang.StringBuilder(255);
   }

   public FootstepPlanActionDefinitionBasicsMessage(FootstepPlanActionDefinitionBasicsMessage other)
   {
      this();
      set(other);
   }

   public void set(FootstepPlanActionDefinitionBasicsMessage other)
   {
      parent_frame_name_.setLength(0);
      parent_frame_name_.append(other.parent_frame_name_);

      swing_duration_ = other.swing_duration_;

      transfer_duration_ = other.transfer_duration_;

   }

   /**
            * Name of the parent frame the footsteps are expressed in
            */
   public void setParentFrameName(java.lang.String parent_frame_name)
   {
      parent_frame_name_.setLength(0);
      parent_frame_name_.append(parent_frame_name);
   }

   /**
            * Name of the parent frame the footsteps are expressed in
            */
   public java.lang.String getParentFrameNameAsString()
   {
      return getParentFrameName().toString();
   }
   /**
            * Name of the parent frame the footsteps are expressed in
            */
   public java.lang.StringBuilder getParentFrameName()
   {
      return parent_frame_name_;
   }

   /**
            * Swing duration
            */
   public void setSwingDuration(double swing_duration)
   {
      swing_duration_ = swing_duration;
   }
   /**
            * Swing duration
            */
   public double getSwingDuration()
   {
      return swing_duration_;
   }

   /**
            * Transfer duration
            */
   public void setTransferDuration(double transfer_duration)
   {
      transfer_duration_ = transfer_duration;
   }
   /**
            * Transfer duration
            */
   public double getTransferDuration()
   {
      return transfer_duration_;
   }


   public static Supplier<FootstepPlanActionDefinitionBasicsMessagePubSubType> getPubSubType()
   {
      return FootstepPlanActionDefinitionBasicsMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FootstepPlanActionDefinitionBasicsMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FootstepPlanActionDefinitionBasicsMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.parent_frame_name_, other.parent_frame_name_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.swing_duration_, other.swing_duration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.transfer_duration_, other.transfer_duration_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FootstepPlanActionDefinitionBasicsMessage)) return false;

      FootstepPlanActionDefinitionBasicsMessage otherMyClass = (FootstepPlanActionDefinitionBasicsMessage) other;

      if (!us.ihmc.idl.IDLTools.equals(this.parent_frame_name_, otherMyClass.parent_frame_name_)) return false;

      if(this.swing_duration_ != otherMyClass.swing_duration_) return false;

      if(this.transfer_duration_ != otherMyClass.transfer_duration_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FootstepPlanActionDefinitionBasicsMessage {");
      builder.append("parent_frame_name=");
      builder.append(this.parent_frame_name_);      builder.append(", ");
      builder.append("swing_duration=");
      builder.append(this.swing_duration_);      builder.append(", ");
      builder.append("transfer_duration=");
      builder.append(this.transfer_duration_);
      builder.append("}");
      return builder.toString();
   }
}
