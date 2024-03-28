package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class WholeBodyBimanipulationActionDefinitionMessage extends Packet<WholeBodyBimanipulationActionDefinitionMessage> implements Settable<WholeBodyBimanipulationActionDefinitionMessage>, EpsilonComparable<WholeBodyBimanipulationActionDefinitionMessage>
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
            * Transform that expresses the hand pose in the parent frame
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage right_hand_transform_to_parent_;
   /**
            * Transform that expresses the hand pose in the parent frame
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage left_hand_transform_to_parent_;
   /**
            * The trajectory duration
            */
   public double trajectory_duration_;
   /**
            * Whether maintaining the rigid body controlled in world after the action is complete
            */
   public boolean hold_pose_in_world_;

   public WholeBodyBimanipulationActionDefinitionMessage()
   {
      definition_ = new behavior_msgs.msg.dds.ActionNodeDefinitionMessage();
      parent_frame_name_ = new java.lang.StringBuilder(255);
      right_hand_transform_to_parent_ = new controller_msgs.msg.dds.RigidBodyTransformMessage();
      left_hand_transform_to_parent_ = new controller_msgs.msg.dds.RigidBodyTransformMessage();
   }

   public WholeBodyBimanipulationActionDefinitionMessage(WholeBodyBimanipulationActionDefinitionMessage other)
   {
      this();
      set(other);
   }

   public void set(WholeBodyBimanipulationActionDefinitionMessage other)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      parent_frame_name_.setLength(0);
      parent_frame_name_.append(other.parent_frame_name_);

      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.staticCopy(other.right_hand_transform_to_parent_, right_hand_transform_to_parent_);
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.staticCopy(other.left_hand_transform_to_parent_, left_hand_transform_to_parent_);
      trajectory_duration_ = other.trajectory_duration_;

      hold_pose_in_world_ = other.hold_pose_in_world_;

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
            * Transform that expresses the hand pose in the parent frame
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage getRightHandTransformToParent()
   {
      return right_hand_transform_to_parent_;
   }


   /**
            * Transform that expresses the hand pose in the parent frame
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage getLeftHandTransformToParent()
   {
      return left_hand_transform_to_parent_;
   }

   /**
            * The trajectory duration
            */
   public void setTrajectoryDuration(double trajectory_duration)
   {
      trajectory_duration_ = trajectory_duration;
   }
   /**
            * The trajectory duration
            */
   public double getTrajectoryDuration()
   {
      return trajectory_duration_;
   }

   /**
            * Whether maintaining the rigid body controlled in world after the action is complete
            */
   public void setHoldPoseInWorld(boolean hold_pose_in_world)
   {
      hold_pose_in_world_ = hold_pose_in_world;
   }
   /**
            * Whether maintaining the rigid body controlled in world after the action is complete
            */
   public boolean getHoldPoseInWorld()
   {
      return hold_pose_in_world_;
   }


   public static Supplier<WholeBodyBimanipulationActionDefinitionMessagePubSubType> getPubSubType()
   {
      return WholeBodyBimanipulationActionDefinitionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return WholeBodyBimanipulationActionDefinitionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(WholeBodyBimanipulationActionDefinitionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.parent_frame_name_, other.parent_frame_name_, epsilon)) return false;

      if (!this.right_hand_transform_to_parent_.epsilonEquals(other.right_hand_transform_to_parent_, epsilon)) return false;
      if (!this.left_hand_transform_to_parent_.epsilonEquals(other.left_hand_transform_to_parent_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.trajectory_duration_, other.trajectory_duration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.hold_pose_in_world_, other.hold_pose_in_world_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof WholeBodyBimanipulationActionDefinitionMessage)) return false;

      WholeBodyBimanipulationActionDefinitionMessage otherMyClass = (WholeBodyBimanipulationActionDefinitionMessage) other;

      if (!this.definition_.equals(otherMyClass.definition_)) return false;
      if (!us.ihmc.idl.IDLTools.equals(this.parent_frame_name_, otherMyClass.parent_frame_name_)) return false;

      if (!this.right_hand_transform_to_parent_.equals(otherMyClass.right_hand_transform_to_parent_)) return false;
      if (!this.left_hand_transform_to_parent_.equals(otherMyClass.left_hand_transform_to_parent_)) return false;
      if(this.trajectory_duration_ != otherMyClass.trajectory_duration_) return false;

      if(this.hold_pose_in_world_ != otherMyClass.hold_pose_in_world_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("WholeBodyBimanipulationActionDefinitionMessage {");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("parent_frame_name=");
      builder.append(this.parent_frame_name_);      builder.append(", ");
      builder.append("right_hand_transform_to_parent=");
      builder.append(this.right_hand_transform_to_parent_);      builder.append(", ");
      builder.append("left_hand_transform_to_parent=");
      builder.append(this.left_hand_transform_to_parent_);      builder.append(", ");
      builder.append("trajectory_duration=");
      builder.append(this.trajectory_duration_);      builder.append(", ");
      builder.append("hold_pose_in_world=");
      builder.append(this.hold_pose_in_world_);
      builder.append("}");
      return builder.toString();
   }
}
