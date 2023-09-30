package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class BodyPartPoseActionDefinitionMessage extends Packet<BodyPartPoseActionDefinitionMessage> implements Settable<BodyPartPoseActionDefinitionMessage>, EpsilonComparable<BodyPartPoseActionDefinitionMessage>
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
            * Transform that expresses the body part pose in the parent frame
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage transform_to_parent_;
   /**
            * Duration of the trajectory
            */
   public double trajectory_duration_;
   /**
            * Whether the next action can be executed at the same time of this one
            */
   public boolean execute_with_next_action_;
   /**
            * Whether maintaining the rigid body controlled in world after the action is complete
            */
   public boolean hold_pose_in_world_;

   public BodyPartPoseActionDefinitionMessage()
   {
      action_information_ = new behavior_msgs.msg.dds.ActionInformationMessage();
      parent_frame_ = new us.ihmc.idl.IDLSequence.StringBuilderHolder (1000, "type_d");
      transform_to_parent_ = new controller_msgs.msg.dds.RigidBodyTransformMessage();
   }

   public BodyPartPoseActionDefinitionMessage(BodyPartPoseActionDefinitionMessage other)
   {
      this();
      set(other);
   }

   public void set(BodyPartPoseActionDefinitionMessage other)
   {
      behavior_msgs.msg.dds.ActionInformationMessagePubSubType.staticCopy(other.action_information_, action_information_);
      parent_frame_.set(other.parent_frame_);
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.staticCopy(other.transform_to_parent_, transform_to_parent_);
      trajectory_duration_ = other.trajectory_duration_;

      execute_with_next_action_ = other.execute_with_next_action_;

      hold_pose_in_world_ = other.hold_pose_in_world_;

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
            * Transform that expresses the body part pose in the parent frame
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage getTransformToParent()
   {
      return transform_to_parent_;
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

   /**
            * Whether the next action can be executed at the same time of this one
            */
   public void setExecuteWithNextAction(boolean execute_with_next_action)
   {
      execute_with_next_action_ = execute_with_next_action;
   }
   /**
            * Whether the next action can be executed at the same time of this one
            */
   public boolean getExecuteWithNextAction()
   {
      return execute_with_next_action_;
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


   public static Supplier<BodyPartPoseActionDefinitionMessagePubSubType> getPubSubType()
   {
      return BodyPartPoseActionDefinitionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return BodyPartPoseActionDefinitionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(BodyPartPoseActionDefinitionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.action_information_.epsilonEquals(other.action_information_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilderSequence(this.parent_frame_, other.parent_frame_, epsilon)) return false;

      if (!this.transform_to_parent_.epsilonEquals(other.transform_to_parent_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.trajectory_duration_, other.trajectory_duration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.execute_with_next_action_, other.execute_with_next_action_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.hold_pose_in_world_, other.hold_pose_in_world_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof BodyPartPoseActionDefinitionMessage)) return false;

      BodyPartPoseActionDefinitionMessage otherMyClass = (BodyPartPoseActionDefinitionMessage) other;

      if (!this.action_information_.equals(otherMyClass.action_information_)) return false;
      if (!this.parent_frame_.equals(otherMyClass.parent_frame_)) return false;
      if (!this.transform_to_parent_.equals(otherMyClass.transform_to_parent_)) return false;
      if(this.trajectory_duration_ != otherMyClass.trajectory_duration_) return false;

      if(this.execute_with_next_action_ != otherMyClass.execute_with_next_action_) return false;

      if(this.hold_pose_in_world_ != otherMyClass.hold_pose_in_world_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BodyPartPoseActionDefinitionMessage {");
      builder.append("action_information=");
      builder.append(this.action_information_);      builder.append(", ");
      builder.append("parent_frame=");
      builder.append(this.parent_frame_);      builder.append(", ");
      builder.append("transform_to_parent=");
      builder.append(this.transform_to_parent_);      builder.append(", ");
      builder.append("trajectory_duration=");
      builder.append(this.trajectory_duration_);      builder.append(", ");
      builder.append("execute_with_next_action=");
      builder.append(this.execute_with_next_action_);      builder.append(", ");
      builder.append("hold_pose_in_world=");
      builder.append(this.hold_pose_in_world_);
      builder.append("}");
      return builder.toString();
   }
}
