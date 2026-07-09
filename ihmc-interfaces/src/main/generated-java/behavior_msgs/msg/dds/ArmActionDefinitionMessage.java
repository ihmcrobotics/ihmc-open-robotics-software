package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class ArmActionDefinitionMessage extends Packet<ArmActionDefinitionMessage> implements Settable<ArmActionDefinitionMessage>, EpsilonComparable<ArmActionDefinitionMessage>
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
            * Whether the action is defined in jointspace (true) or taskspace (false)
            */
   public boolean defined_in_jointspace_;
   /**
            * Taskspace trajectory mode: 0 = single pose, 1 = screw primitive
            */
   public byte taskspace_trajectory_mode_;
   /**
            * The trajectory duration
            */
   public double trajectory_duration_;
   /**
            * Preset arm configuration
            */
   public byte preset_;
   /**
            * Joint angles
            */
   public double[] joint_angles_;
   /**
            * Name of the frame the this action is expressed in
            */
   public java.lang.StringBuilder parent_frame_name_;
   /**
            * Transform that expresses the hand pose in the parent frame
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage transform_to_parent_;
   /**
            * Whether the rigid body is controlled in jointspace (true) or hybrid (false) during single-point taskspace execution
            */
   public boolean joint_space_control_;
   /**
            * Whether maintaining the rigid body controlled in world after the action is complete
            */
   public boolean hold_pose_in_world_;
   /**
            * Screw primitive fields
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage screw_axis_pose_;
   public double translation_;
   public double rotation_;
   public double max_linear_velocity_;
   public double max_angular_velocity_;
   public double linear_position_weight_;
   public double angular_position_weight_;
   public double jointspace_weight_;
   public double position_error_tolerance_;
   public double orientation_error_tolerance_;

   public ArmActionDefinitionMessage()
   {
      definition_ = new behavior_msgs.msg.dds.ActionNodeDefinitionMessage();
      joint_angles_ = new double[7];

      parent_frame_name_ = new java.lang.StringBuilder(255);
      transform_to_parent_ = new controller_msgs.msg.dds.RigidBodyTransformMessage();
      screw_axis_pose_ = new controller_msgs.msg.dds.RigidBodyTransformMessage();
   }

   public ArmActionDefinitionMessage(ArmActionDefinitionMessage other)
   {
      this();
      set(other);
   }

   public void set(ArmActionDefinitionMessage other)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      robot_side_ = other.robot_side_;

      defined_in_jointspace_ = other.defined_in_jointspace_;

      taskspace_trajectory_mode_ = other.taskspace_trajectory_mode_;

      trajectory_duration_ = other.trajectory_duration_;

      preset_ = other.preset_;

      for(int i1 = 0; i1 < joint_angles_.length; ++i1)
      {
            joint_angles_[i1] = other.joint_angles_[i1];

      }

      parent_frame_name_.setLength(0);
      parent_frame_name_.append(other.parent_frame_name_);

      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.staticCopy(other.transform_to_parent_, transform_to_parent_);
      joint_space_control_ = other.joint_space_control_;

      hold_pose_in_world_ = other.hold_pose_in_world_;

      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.staticCopy(other.screw_axis_pose_, screw_axis_pose_);
      translation_ = other.translation_;

      rotation_ = other.rotation_;

      max_linear_velocity_ = other.max_linear_velocity_;

      max_angular_velocity_ = other.max_angular_velocity_;

      linear_position_weight_ = other.linear_position_weight_;

      angular_position_weight_ = other.angular_position_weight_;

      jointspace_weight_ = other.jointspace_weight_;

      position_error_tolerance_ = other.position_error_tolerance_;

      orientation_error_tolerance_ = other.orientation_error_tolerance_;

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
            * Whether the action is defined in jointspace (true) or taskspace (false)
            */
   public void setDefinedInJointspace(boolean defined_in_jointspace)
   {
      defined_in_jointspace_ = defined_in_jointspace;
   }
   /**
            * Whether the action is defined in jointspace (true) or taskspace (false)
            */
   public boolean getDefinedInJointspace()
   {
      return defined_in_jointspace_;
   }

   /**
            * Taskspace trajectory mode: 0 = single pose, 1 = screw primitive
            */
   public void setTaskspaceTrajectoryMode(byte taskspace_trajectory_mode)
   {
      taskspace_trajectory_mode_ = taskspace_trajectory_mode;
   }
   /**
            * Taskspace trajectory mode: 0 = single pose, 1 = screw primitive
            */
   public byte getTaskspaceTrajectoryMode()
   {
      return taskspace_trajectory_mode_;
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
            * Preset arm configuration
            */
   public void setPreset(byte preset)
   {
      preset_ = preset;
   }
   /**
            * Preset arm configuration
            */
   public byte getPreset()
   {
      return preset_;
   }


   /**
            * Joint angles
            */
   public double[] getJointAngles()
   {
      return joint_angles_;
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
            * Whether the rigid body is controlled in jointspace (true) or hybrid (false) during single-point taskspace execution
            */
   public void setJointSpaceControl(boolean joint_space_control)
   {
      joint_space_control_ = joint_space_control;
   }
   /**
            * Whether the rigid body is controlled in jointspace (true) or hybrid (false) during single-point taskspace execution
            */
   public boolean getJointSpaceControl()
   {
      return joint_space_control_;
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
            * Screw primitive fields
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage getScrewAxisPose()
   {
      return screw_axis_pose_;
   }

   public void setTranslation(double translation)
   {
      translation_ = translation;
   }
   public double getTranslation()
   {
      return translation_;
   }

   public void setRotation(double rotation)
   {
      rotation_ = rotation;
   }
   public double getRotation()
   {
      return rotation_;
   }

   public void setMaxLinearVelocity(double max_linear_velocity)
   {
      max_linear_velocity_ = max_linear_velocity;
   }
   public double getMaxLinearVelocity()
   {
      return max_linear_velocity_;
   }

   public void setMaxAngularVelocity(double max_angular_velocity)
   {
      max_angular_velocity_ = max_angular_velocity;
   }
   public double getMaxAngularVelocity()
   {
      return max_angular_velocity_;
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

   public void setJointspaceWeight(double jointspace_weight)
   {
      jointspace_weight_ = jointspace_weight;
   }
   public double getJointspaceWeight()
   {
      return jointspace_weight_;
   }

   public void setPositionErrorTolerance(double position_error_tolerance)
   {
      position_error_tolerance_ = position_error_tolerance;
   }
   public double getPositionErrorTolerance()
   {
      return position_error_tolerance_;
   }

   public void setOrientationErrorTolerance(double orientation_error_tolerance)
   {
      orientation_error_tolerance_ = orientation_error_tolerance;
   }
   public double getOrientationErrorTolerance()
   {
      return orientation_error_tolerance_;
   }


   public static Supplier<ArmActionDefinitionMessagePubSubType> getPubSubType()
   {
      return ArmActionDefinitionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ArmActionDefinitionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ArmActionDefinitionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.defined_in_jointspace_, other.defined_in_jointspace_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.taskspace_trajectory_mode_, other.taskspace_trajectory_mode_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.trajectory_duration_, other.trajectory_duration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.preset_, other.preset_, epsilon)) return false;

      for(int i3 = 0; i3 < joint_angles_.length; ++i3)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.joint_angles_[i3], other.joint_angles_[i3], epsilon)) return false;
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.parent_frame_name_, other.parent_frame_name_, epsilon)) return false;

      if (!this.transform_to_parent_.epsilonEquals(other.transform_to_parent_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.joint_space_control_, other.joint_space_control_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.hold_pose_in_world_, other.hold_pose_in_world_, epsilon)) return false;

      if (!this.screw_axis_pose_.epsilonEquals(other.screw_axis_pose_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.translation_, other.translation_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.rotation_, other.rotation_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.max_linear_velocity_, other.max_linear_velocity_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.max_angular_velocity_, other.max_angular_velocity_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.linear_position_weight_, other.linear_position_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.angular_position_weight_, other.angular_position_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.jointspace_weight_, other.jointspace_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.position_error_tolerance_, other.position_error_tolerance_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.orientation_error_tolerance_, other.orientation_error_tolerance_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ArmActionDefinitionMessage)) return false;

      ArmActionDefinitionMessage otherMyClass = (ArmActionDefinitionMessage) other;

      if (!this.definition_.equals(otherMyClass.definition_)) return false;
      if(this.robot_side_ != otherMyClass.robot_side_) return false;

      if(this.defined_in_jointspace_ != otherMyClass.defined_in_jointspace_) return false;

      if(this.taskspace_trajectory_mode_ != otherMyClass.taskspace_trajectory_mode_) return false;

      if(this.trajectory_duration_ != otherMyClass.trajectory_duration_) return false;

      if(this.preset_ != otherMyClass.preset_) return false;

      for(int i5 = 0; i5 < joint_angles_.length; ++i5)
      {
                if(this.joint_angles_[i5] != otherMyClass.joint_angles_[i5]) return false;

      }
      if (!us.ihmc.idl.IDLTools.equals(this.parent_frame_name_, otherMyClass.parent_frame_name_)) return false;

      if (!this.transform_to_parent_.equals(otherMyClass.transform_to_parent_)) return false;
      if(this.joint_space_control_ != otherMyClass.joint_space_control_) return false;

      if(this.hold_pose_in_world_ != otherMyClass.hold_pose_in_world_) return false;

      if (!this.screw_axis_pose_.equals(otherMyClass.screw_axis_pose_)) return false;
      if(this.translation_ != otherMyClass.translation_) return false;

      if(this.rotation_ != otherMyClass.rotation_) return false;

      if(this.max_linear_velocity_ != otherMyClass.max_linear_velocity_) return false;

      if(this.max_angular_velocity_ != otherMyClass.max_angular_velocity_) return false;

      if(this.linear_position_weight_ != otherMyClass.linear_position_weight_) return false;

      if(this.angular_position_weight_ != otherMyClass.angular_position_weight_) return false;

      if(this.jointspace_weight_ != otherMyClass.jointspace_weight_) return false;

      if(this.position_error_tolerance_ != otherMyClass.position_error_tolerance_) return false;

      if(this.orientation_error_tolerance_ != otherMyClass.orientation_error_tolerance_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ArmActionDefinitionMessage {");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");
      builder.append("defined_in_jointspace=");
      builder.append(this.defined_in_jointspace_);      builder.append(", ");
      builder.append("taskspace_trajectory_mode=");
      builder.append(this.taskspace_trajectory_mode_);      builder.append(", ");
      builder.append("trajectory_duration=");
      builder.append(this.trajectory_duration_);      builder.append(", ");
      builder.append("preset=");
      builder.append(this.preset_);      builder.append(", ");
      builder.append("joint_angles=");
      builder.append(java.util.Arrays.toString(this.joint_angles_));      builder.append(", ");
      builder.append("parent_frame_name=");
      builder.append(this.parent_frame_name_);      builder.append(", ");
      builder.append("transform_to_parent=");
      builder.append(this.transform_to_parent_);      builder.append(", ");
      builder.append("joint_space_control=");
      builder.append(this.joint_space_control_);      builder.append(", ");
      builder.append("hold_pose_in_world=");
      builder.append(this.hold_pose_in_world_);      builder.append(", ");
      builder.append("screw_axis_pose=");
      builder.append(this.screw_axis_pose_);      builder.append(", ");
      builder.append("translation=");
      builder.append(this.translation_);      builder.append(", ");
      builder.append("rotation=");
      builder.append(this.rotation_);      builder.append(", ");
      builder.append("max_linear_velocity=");
      builder.append(this.max_linear_velocity_);      builder.append(", ");
      builder.append("max_angular_velocity=");
      builder.append(this.max_angular_velocity_);      builder.append(", ");
      builder.append("linear_position_weight=");
      builder.append(this.linear_position_weight_);      builder.append(", ");
      builder.append("angular_position_weight=");
      builder.append(this.angular_position_weight_);      builder.append(", ");
      builder.append("jointspace_weight=");
      builder.append(this.jointspace_weight_);      builder.append(", ");
      builder.append("position_error_tolerance=");
      builder.append(this.position_error_tolerance_);      builder.append(", ");
      builder.append("orientation_error_tolerance=");
      builder.append(this.orientation_error_tolerance_);
      builder.append("}");
      return builder.toString();
   }
}
