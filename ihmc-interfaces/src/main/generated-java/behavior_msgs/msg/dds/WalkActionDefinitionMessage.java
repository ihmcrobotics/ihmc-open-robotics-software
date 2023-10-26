package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class WalkActionDefinitionMessage extends Packet<WalkActionDefinitionMessage> implements Settable<WalkActionDefinitionMessage>, EpsilonComparable<WalkActionDefinitionMessage>
{
   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.ActionNodeDefinitionMessage definition_;
   /**
            * Name of the frame the this action is expressed in
            */
   public java.lang.StringBuilder parent_frame_name_;
   /**
            * Transform that expresses the walk gizmo pose in the parent frame
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage transform_to_parent_;
   /**
            * Left goal foot transform to the walk gizmo
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage left_goal_foot_transform_to_gizmo_;
   /**
            * Right goal foot transform to the walk gizmo
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage right_goal_foot_transform_to_gizmo_;
   /**
            * Swing duration
            */
   public double swing_duration_;
   /**
            * Transfer duration
            */
   public double transfer_duration_;

   public WalkActionDefinitionMessage()
   {
      definition_ = new behavior_msgs.msg.dds.ActionNodeDefinitionMessage();
      parent_frame_name_ = new java.lang.StringBuilder(255);
      transform_to_parent_ = new controller_msgs.msg.dds.RigidBodyTransformMessage();
      left_goal_foot_transform_to_gizmo_ = new controller_msgs.msg.dds.RigidBodyTransformMessage();
      right_goal_foot_transform_to_gizmo_ = new controller_msgs.msg.dds.RigidBodyTransformMessage();
   }

   public WalkActionDefinitionMessage(WalkActionDefinitionMessage other)
   {
      this();
      set(other);
   }

   public void set(WalkActionDefinitionMessage other)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      parent_frame_name_.setLength(0);
      parent_frame_name_.append(other.parent_frame_name_);

      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.staticCopy(other.transform_to_parent_, transform_to_parent_);
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.staticCopy(other.left_goal_foot_transform_to_gizmo_, left_goal_foot_transform_to_gizmo_);
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.staticCopy(other.right_goal_foot_transform_to_gizmo_, right_goal_foot_transform_to_gizmo_);
      swing_duration_ = other.swing_duration_;

      transfer_duration_ = other.transfer_duration_;

   }


   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.ActionNodeDefinitionMessage getDefinition()
   {
      return definition_;
   }

   /**
            * Name of the frame the this action is expressed in
            */
   public void setParentFrameName(java.lang.String parent_frame_name)
   {
      parent_frame_name_.setLength(0);
      parent_frame_name_.append(parent_frame_name);
   }

   /**
            * Name of the frame the this action is expressed in
            */
   public java.lang.String getParentFrameNameAsString()
   {
      return getParentFrameName().toString();
   }
   /**
            * Name of the frame the this action is expressed in
            */
   public java.lang.StringBuilder getParentFrameName()
   {
      return parent_frame_name_;
   }


   /**
            * Transform that expresses the walk gizmo pose in the parent frame
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage getTransformToParent()
   {
      return transform_to_parent_;
   }


   /**
            * Left goal foot transform to the walk gizmo
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage getLeftGoalFootTransformToGizmo()
   {
      return left_goal_foot_transform_to_gizmo_;
   }


   /**
            * Right goal foot transform to the walk gizmo
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage getRightGoalFootTransformToGizmo()
   {
      return right_goal_foot_transform_to_gizmo_;
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


   public static Supplier<WalkActionDefinitionMessagePubSubType> getPubSubType()
   {
      return WalkActionDefinitionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return WalkActionDefinitionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(WalkActionDefinitionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.parent_frame_name_, other.parent_frame_name_, epsilon)) return false;

      if (!this.transform_to_parent_.epsilonEquals(other.transform_to_parent_, epsilon)) return false;
      if (!this.left_goal_foot_transform_to_gizmo_.epsilonEquals(other.left_goal_foot_transform_to_gizmo_, epsilon)) return false;
      if (!this.right_goal_foot_transform_to_gizmo_.epsilonEquals(other.right_goal_foot_transform_to_gizmo_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.swing_duration_, other.swing_duration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.transfer_duration_, other.transfer_duration_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof WalkActionDefinitionMessage)) return false;

      WalkActionDefinitionMessage otherMyClass = (WalkActionDefinitionMessage) other;

      if (!this.definition_.equals(otherMyClass.definition_)) return false;
      if (!us.ihmc.idl.IDLTools.equals(this.parent_frame_name_, otherMyClass.parent_frame_name_)) return false;

      if (!this.transform_to_parent_.equals(otherMyClass.transform_to_parent_)) return false;
      if (!this.left_goal_foot_transform_to_gizmo_.equals(otherMyClass.left_goal_foot_transform_to_gizmo_)) return false;
      if (!this.right_goal_foot_transform_to_gizmo_.equals(otherMyClass.right_goal_foot_transform_to_gizmo_)) return false;
      if(this.swing_duration_ != otherMyClass.swing_duration_) return false;

      if(this.transfer_duration_ != otherMyClass.transfer_duration_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("WalkActionDefinitionMessage {");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("parent_frame_name=");
      builder.append(this.parent_frame_name_);      builder.append(", ");
      builder.append("transform_to_parent=");
      builder.append(this.transform_to_parent_);      builder.append(", ");
      builder.append("left_goal_foot_transform_to_gizmo=");
      builder.append(this.left_goal_foot_transform_to_gizmo_);      builder.append(", ");
      builder.append("right_goal_foot_transform_to_gizmo=");
      builder.append(this.right_goal_foot_transform_to_gizmo_);      builder.append(", ");
      builder.append("swing_duration=");
      builder.append(this.swing_duration_);      builder.append(", ");
      builder.append("transfer_duration=");
      builder.append(this.transfer_duration_);
      builder.append("}");
      return builder.toString();
   }
}
