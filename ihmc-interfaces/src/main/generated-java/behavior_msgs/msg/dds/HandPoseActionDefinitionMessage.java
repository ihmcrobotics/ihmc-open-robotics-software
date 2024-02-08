package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class HandPoseActionDefinitionMessage extends Packet<HandPoseActionDefinitionMessage> implements Settable<HandPoseActionDefinitionMessage>, EpsilonComparable<HandPoseActionDefinitionMessage>
{
   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.ActionNodeDefinitionMessage definition_;
   /**
            * Specifies the side of the robot that this message refers to.
            */
   public byte robot_side_ = (byte) 255;
   /**
            * Name of the frame the this action is expressed in
            */
   public java.lang.StringBuilder parent_frame_name_;
   /**
            * Transform that expresses the hand pose in the parent frame
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage transform_to_parent_;
   /**
            * The trajectory duration
            */
   public double trajectory_duration_;
   /**
            * Whether maintaining the rigid body controlled in world after the action is complete
            */
   public boolean hold_pose_in_world_;
   /**
            * Whether the rigid body is controlled in jointspace (true) or taskspace (false)
            */
   public boolean joint_space_control_;
   public double linear_position_weight_;
   public double angular_position_weight_;

   public HandPoseActionDefinitionMessage()
   {
      definition_ = new behavior_msgs.msg.dds.ActionNodeDefinitionMessage();
      parent_frame_name_ = new java.lang.StringBuilder(255);
      transform_to_parent_ = new controller_msgs.msg.dds.RigidBodyTransformMessage();
   }

   public HandPoseActionDefinitionMessage(HandPoseActionDefinitionMessage other)
   {
      this();
      set(other);
   }

   public void set(HandPoseActionDefinitionMessage other)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      robot_side_ = other.robot_side_;

      parent_frame_name_.setLength(0);
      parent_frame_name_.append(other.parent_frame_name_);

      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.staticCopy(other.transform_to_parent_, transform_to_parent_);
      trajectory_duration_ = other.trajectory_duration_;

      hold_pose_in_world_ = other.hold_pose_in_world_;

      joint_space_control_ = other.joint_space_control_;

      linear_position_weight_ = other.linear_position_weight_;

      angular_position_weight_ = other.angular_position_weight_;

   }


   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.ActionNodeDefinitionMessage getDefinition()
   {
      return definition_;
   }

   /**
            * Specifies the side of the robot that this message refers to.
            */
   public void setRobotSide(byte robot_side)
   {
      robot_side_ = robot_side;
   }
   /**
            * Specifies the side of the robot that this message refers to.
            */
   public byte getRobotSide()
   {
      return robot_side_;
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
   public controller_msgs.msg.dds.RigidBodyTransformMessage getTransformToParent()
   {
      return transform_to_parent_;
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

   /**
            * Whether the rigid body is controlled in jointspace (true) or taskspace (false)
            */
   public void setJointSpaceControl(boolean joint_space_control)
   {
      joint_space_control_ = joint_space_control;
   }
   /**
            * Whether the rigid body is controlled in jointspace (true) or taskspace (false)
            */
   public boolean getJointSpaceControl()
   {
      return joint_space_control_;
   }

   public void setLinearPositionWeight(double linear_position_weight)
   {
      linear_position_weight_ = linear_position_weight;
   }
   public double getLinearPositionWeight()
   {
      return linear_position_weight_;
   }

   public void setAngularPositionWeight(double angular_position_weight)
   {
      angular_position_weight_ = angular_position_weight;
   }
   public double getAngularPositionWeight()
   {
      return angular_position_weight_;
   }


   public static Supplier<HandPoseActionDefinitionMessagePubSubType> getPubSubType()
   {
      return HandPoseActionDefinitionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return HandPoseActionDefinitionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(HandPoseActionDefinitionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.parent_frame_name_, other.parent_frame_name_, epsilon)) return false;

      if (!this.transform_to_parent_.epsilonEquals(other.transform_to_parent_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.trajectory_duration_, other.trajectory_duration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.hold_pose_in_world_, other.hold_pose_in_world_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.joint_space_control_, other.joint_space_control_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.linear_position_weight_, other.linear_position_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.angular_position_weight_, other.angular_position_weight_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof HandPoseActionDefinitionMessage)) return false;

      HandPoseActionDefinitionMessage otherMyClass = (HandPoseActionDefinitionMessage) other;

      if (!this.definition_.equals(otherMyClass.definition_)) return false;
      if(this.robot_side_ != otherMyClass.robot_side_) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.parent_frame_name_, otherMyClass.parent_frame_name_)) return false;

      if (!this.transform_to_parent_.equals(otherMyClass.transform_to_parent_)) return false;
      if(this.trajectory_duration_ != otherMyClass.trajectory_duration_) return false;

      if(this.hold_pose_in_world_ != otherMyClass.hold_pose_in_world_) return false;

      if(this.joint_space_control_ != otherMyClass.joint_space_control_) return false;

      if(this.linear_position_weight_ != otherMyClass.linear_position_weight_) return false;

      if(this.angular_position_weight_ != otherMyClass.angular_position_weight_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("HandPoseActionDefinitionMessage {");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");
      builder.append("parent_frame_name=");
      builder.append(this.parent_frame_name_);      builder.append(", ");
      builder.append("transform_to_parent=");
      builder.append(this.transform_to_parent_);      builder.append(", ");
      builder.append("trajectory_duration=");
      builder.append(this.trajectory_duration_);      builder.append(", ");
      builder.append("hold_pose_in_world=");
      builder.append(this.hold_pose_in_world_);      builder.append(", ");
      builder.append("joint_space_control=");
      builder.append(this.joint_space_control_);      builder.append(", ");
      builder.append("linear_position_weight=");
      builder.append(this.linear_position_weight_);      builder.append(", ");
      builder.append("angular_position_weight=");
      builder.append(this.angular_position_weight_);
      builder.append("}");
      return builder.toString();
   }
}
