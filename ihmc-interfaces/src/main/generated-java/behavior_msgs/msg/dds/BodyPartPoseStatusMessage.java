package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class BodyPartPoseStatusMessage extends Packet<BodyPartPoseStatusMessage> implements Settable<BodyPartPoseStatusMessage>, EpsilonComparable<BodyPartPoseStatusMessage>
{
   /**
            * Whether the action hs to be executed next and is part of a group of concurrent actions
            */
   public boolean current_and_concurrent_;
   /**
            * Name of the frame the this action is expressed in
            */
   public us.ihmc.idl.IDLSequence.StringBuilderHolder  parent_frame_;
   /**
            * Transform that expresses the pelvis pose in the parent frame
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage transform_to_parent_;

   public BodyPartPoseStatusMessage()
   {
      parent_frame_ = new us.ihmc.idl.IDLSequence.StringBuilderHolder (1000, "type_d");
      transform_to_parent_ = new controller_msgs.msg.dds.RigidBodyTransformMessage();
   }

   public BodyPartPoseStatusMessage(BodyPartPoseStatusMessage other)
   {
      this();
      set(other);
   }

   public void set(BodyPartPoseStatusMessage other)
   {
      current_and_concurrent_ = other.current_and_concurrent_;

      parent_frame_.set(other.parent_frame_);
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.staticCopy(other.transform_to_parent_, transform_to_parent_);
   }

   /**
            * Whether the action hs to be executed next and is part of a group of concurrent actions
            */
   public void setCurrentAndConcurrent(boolean current_and_concurrent)
   {
      current_and_concurrent_ = current_and_concurrent;
   }
   /**
            * Whether the action hs to be executed next and is part of a group of concurrent actions
            */
   public boolean getCurrentAndConcurrent()
   {
      return current_and_concurrent_;
   }


   /**
            * Name of the frame the this action is expressed in
            */
   public us.ihmc.idl.IDLSequence.StringBuilderHolder  getParentFrame()
   {
      return parent_frame_;
   }


   /**
            * Transform that expresses the pelvis pose in the parent frame
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage getTransformToParent()
   {
      return transform_to_parent_;
   }


   public static Supplier<BodyPartPoseStatusMessagePubSubType> getPubSubType()
   {
      return BodyPartPoseStatusMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return BodyPartPoseStatusMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(BodyPartPoseStatusMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.current_and_concurrent_, other.current_and_concurrent_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilderSequence(this.parent_frame_, other.parent_frame_, epsilon)) return false;

      if (!this.transform_to_parent_.epsilonEquals(other.transform_to_parent_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof BodyPartPoseStatusMessage)) return false;

      BodyPartPoseStatusMessage otherMyClass = (BodyPartPoseStatusMessage) other;

      if(this.current_and_concurrent_ != otherMyClass.current_and_concurrent_) return false;

      if (!this.parent_frame_.equals(otherMyClass.parent_frame_)) return false;
      if (!this.transform_to_parent_.equals(otherMyClass.transform_to_parent_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BodyPartPoseStatusMessage {");
      builder.append("current_and_concurrent=");
      builder.append(this.current_and_concurrent_);      builder.append(", ");
      builder.append("parent_frame=");
      builder.append(this.parent_frame_);      builder.append(", ");
      builder.append("transform_to_parent=");
      builder.append(this.transform_to_parent_);
      builder.append("}");
      return builder.toString();
   }
}
