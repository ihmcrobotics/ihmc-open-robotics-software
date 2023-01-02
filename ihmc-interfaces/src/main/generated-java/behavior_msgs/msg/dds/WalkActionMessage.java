package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class WalkActionMessage extends Packet<WalkActionMessage> implements Settable<WalkActionMessage>, EpsilonComparable<WalkActionMessage>
{
   /**
            * Used for syncing action sequences
            */
   public behavior_msgs.msg.dds.ActionInformationMessage action_information_;
   /**
            * Name of the frame the this action is expressed in
            */
   public us.ihmc.idl.IDLSequence.StringBuilderHolder  parent_frame_;
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

   public WalkActionMessage()
   {
      action_information_ = new behavior_msgs.msg.dds.ActionInformationMessage();
      parent_frame_ = new us.ihmc.idl.IDLSequence.StringBuilderHolder (1000, "type_d");
      transform_to_parent_ = new controller_msgs.msg.dds.RigidBodyTransformMessage();
      left_goal_foot_transform_to_gizmo_ = new controller_msgs.msg.dds.RigidBodyTransformMessage();
      right_goal_foot_transform_to_gizmo_ = new controller_msgs.msg.dds.RigidBodyTransformMessage();
   }

   public WalkActionMessage(WalkActionMessage other)
   {
      this();
      set(other);
   }

   public void set(WalkActionMessage other)
   {
      behavior_msgs.msg.dds.ActionInformationMessagePubSubType.staticCopy(other.action_information_, action_information_);
      parent_frame_.set(other.parent_frame_);
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.staticCopy(other.transform_to_parent_, transform_to_parent_);
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.staticCopy(other.left_goal_foot_transform_to_gizmo_, left_goal_foot_transform_to_gizmo_);
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.staticCopy(other.right_goal_foot_transform_to_gizmo_, right_goal_foot_transform_to_gizmo_);
      swing_duration_ = other.swing_duration_;

      transfer_duration_ = other.transfer_duration_;

   }


   /**
            * Used for syncing action sequences
            */
   public behavior_msgs.msg.dds.ActionInformationMessage getActionInformation()
   {
      return action_information_;
   }


   /**
            * Name of the frame the this action is expressed in
            */
   public us.ihmc.idl.IDLSequence.StringBuilderHolder  getParentFrame()
   {
      return parent_frame_;
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


   public static Supplier<WalkActionMessagePubSubType> getPubSubType()
   {
      return WalkActionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return WalkActionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(WalkActionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.action_information_.epsilonEquals(other.action_information_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilderSequence(this.parent_frame_, other.parent_frame_, epsilon)) return false;

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
      if(!(other instanceof WalkActionMessage)) return false;

      WalkActionMessage otherMyClass = (WalkActionMessage) other;

      if (!this.action_information_.equals(otherMyClass.action_information_)) return false;
      if (!this.parent_frame_.equals(otherMyClass.parent_frame_)) return false;
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

      builder.append("WalkActionMessage {");
      builder.append("action_information=");
      builder.append(this.action_information_);      builder.append(", ");
      builder.append("parent_frame=");
      builder.append(this.parent_frame_);      builder.append(", ");
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
