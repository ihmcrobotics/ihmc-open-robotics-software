package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class PelvisHeightPitchActionDefinitionMessage extends Packet<PelvisHeightPitchActionDefinitionMessage> implements Settable<PelvisHeightPitchActionDefinitionMessage>, EpsilonComparable<PelvisHeightPitchActionDefinitionMessage>
{
   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.BehaviorActionDefinitionMessage action_definition_;
   /**
            * Name of the frame the this action is expressed in
            */
   public us.ihmc.idl.IDLSequence.StringBuilderHolder  parent_frame_;
   /**
            * Transform that expresses the pelvis pose in the parent frame
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage pelvis_transform_to_parent_;
   /**
            * Duration of the trajectory
            */
   public double trajectory_duration_;

   public PelvisHeightPitchActionDefinitionMessage()
   {
      action_definition_ = new behavior_msgs.msg.dds.BehaviorActionDefinitionMessage();
      parent_frame_ = new us.ihmc.idl.IDLSequence.StringBuilderHolder (1000, "type_d");
      pelvis_transform_to_parent_ = new controller_msgs.msg.dds.RigidBodyTransformMessage();
   }

   public PelvisHeightPitchActionDefinitionMessage(PelvisHeightPitchActionDefinitionMessage other)
   {
      this();
      set(other);
   }

   public void set(PelvisHeightPitchActionDefinitionMessage other)
   {
      behavior_msgs.msg.dds.BehaviorActionDefinitionMessagePubSubType.staticCopy(other.action_definition_, action_definition_);
      parent_frame_.set(other.parent_frame_);
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.staticCopy(other.pelvis_transform_to_parent_, pelvis_transform_to_parent_);
      trajectory_duration_ = other.trajectory_duration_;

   }


   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.BehaviorActionDefinitionMessage getActionDefinition()
   {
      return action_definition_;
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
   public controller_msgs.msg.dds.RigidBodyTransformMessage getPelvisTransformToParent()
   {
      return pelvis_transform_to_parent_;
   }

   /**
            * Duration of the trajectory
            */
   public void setTrajectoryDuration(double trajectory_duration)
   {
      trajectory_duration_ = trajectory_duration;
   }
   /**
            * Duration of the trajectory
            */
   public double getTrajectoryDuration()
   {
      return trajectory_duration_;
   }


   public static Supplier<PelvisHeightPitchActionDefinitionMessagePubSubType> getPubSubType()
   {
      return PelvisHeightPitchActionDefinitionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return PelvisHeightPitchActionDefinitionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(PelvisHeightPitchActionDefinitionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.action_definition_.epsilonEquals(other.action_definition_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilderSequence(this.parent_frame_, other.parent_frame_, epsilon)) return false;

      if (!this.pelvis_transform_to_parent_.epsilonEquals(other.pelvis_transform_to_parent_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.trajectory_duration_, other.trajectory_duration_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof PelvisHeightPitchActionDefinitionMessage)) return false;

      PelvisHeightPitchActionDefinitionMessage otherMyClass = (PelvisHeightPitchActionDefinitionMessage) other;

      if (!this.action_definition_.equals(otherMyClass.action_definition_)) return false;
      if (!this.parent_frame_.equals(otherMyClass.parent_frame_)) return false;
      if (!this.pelvis_transform_to_parent_.equals(otherMyClass.pelvis_transform_to_parent_)) return false;
      if(this.trajectory_duration_ != otherMyClass.trajectory_duration_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("PelvisHeightPitchActionDefinitionMessage {");
      builder.append("action_definition=");
      builder.append(this.action_definition_);      builder.append(", ");
      builder.append("parent_frame=");
      builder.append(this.parent_frame_);      builder.append(", ");
      builder.append("pelvis_transform_to_parent=");
      builder.append(this.pelvis_transform_to_parent_);      builder.append(", ");
      builder.append("trajectory_duration=");
      builder.append(this.trajectory_duration_);
      builder.append("}");
      return builder.toString();
   }
}
