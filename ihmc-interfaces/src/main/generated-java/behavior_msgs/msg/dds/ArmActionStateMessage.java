package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class ArmActionStateMessage extends Packet<ArmActionStateMessage> implements Settable<ArmActionStateMessage>, EpsilonComparable<ArmActionStateMessage>
{
   /**
            * Parent state fields
            */
   public behavior_msgs.msg.dds.ActionNodeStateMessage state_;
   /**
            * Definition
            */
   public behavior_msgs.msg.dds.ArmActionDefinitionMessage definition_;
   /**
            * This is the estimated goal chest frame as the robot executes a potential whole body action.
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage goal_chest_transform_to_world_;
   /**
            * Hand force
            */
   public us.ihmc.euclid.tuple3D.Vector3D force_;
   /**
            * Hand torque
            */
   public us.ihmc.euclid.tuple3D.Vector3D torque_;
   /**
            * Joint angles
            */
   public double[] joint_angles_;
   /**
            * Quality of the IK solution
            */
   public double solution_quality_;
   /**
            * Screw primitive preview fields
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.geometry.Pose3D>  preview_trajectory_;
   public double preview_trajectory_duration_;
   public double preview_trajectory_linear_velocity_;
   public double preview_trajectory_angular_velocity_;
   /**
            * The user requested IK preview time normalized from 0.0 to 1.0
            */
   public double preview_requested_time_;
   public double[] preview_joint_angles_;
   public double preview_solution_quality_;

   public ArmActionStateMessage()
   {
      state_ = new behavior_msgs.msg.dds.ActionNodeStateMessage();
      definition_ = new behavior_msgs.msg.dds.ArmActionDefinitionMessage();
      goal_chest_transform_to_world_ = new controller_msgs.msg.dds.RigidBodyTransformMessage();
      force_ = new us.ihmc.euclid.tuple3D.Vector3D();
      torque_ = new us.ihmc.euclid.tuple3D.Vector3D();
      joint_angles_ = new double[7];

      preview_trajectory_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.geometry.Pose3D> (50, new geometry_msgs.msg.dds.PosePubSubType());
      preview_joint_angles_ = new double[7];


   }

   public ArmActionStateMessage(ArmActionStateMessage other)
   {
      this();
      set(other);
   }

   public void set(ArmActionStateMessage other)
   {
      behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.staticCopy(other.state_, state_);
      behavior_msgs.msg.dds.ArmActionDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.staticCopy(other.goal_chest_transform_to_world_, goal_chest_transform_to_world_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.force_, force_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.torque_, torque_);
      for(int i1 = 0; i1 < joint_angles_.length; ++i1)
      {
            joint_angles_[i1] = other.joint_angles_[i1];

      }

      solution_quality_ = other.solution_quality_;

      preview_trajectory_.set(other.preview_trajectory_);
      preview_trajectory_duration_ = other.preview_trajectory_duration_;

      preview_trajectory_linear_velocity_ = other.preview_trajectory_linear_velocity_;

      preview_trajectory_angular_velocity_ = other.preview_trajectory_angular_velocity_;

      preview_requested_time_ = other.preview_requested_time_;

      for(int i3 = 0; i3 < preview_joint_angles_.length; ++i3)
      {
            preview_joint_angles_[i3] = other.preview_joint_angles_[i3];

      }

      preview_solution_quality_ = other.preview_solution_quality_;

   }


   /**
            * Parent state fields
            */
   public behavior_msgs.msg.dds.ActionNodeStateMessage getState()
   {
      return state_;
   }


   /**
            * Definition
            */
   public behavior_msgs.msg.dds.ArmActionDefinitionMessage getDefinition()
   {
      return definition_;
   }


   /**
            * This is the estimated goal chest frame as the robot executes a potential whole body action.
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage getGoalChestTransformToWorld()
   {
      return goal_chest_transform_to_world_;
   }


   /**
            * Hand force
            */
   public us.ihmc.euclid.tuple3D.Vector3D getForce()
   {
      return force_;
   }


   /**
            * Hand torque
            */
   public us.ihmc.euclid.tuple3D.Vector3D getTorque()
   {
      return torque_;
   }


   /**
            * Joint angles
            */
   public double[] getJointAngles()
   {
      return joint_angles_;
   }

   /**
            * Quality of the IK solution
            */
   public void setSolutionQuality(double solution_quality)
   {
      solution_quality_ = solution_quality;
   }
   /**
            * Quality of the IK solution
            */
   public double getSolutionQuality()
   {
      return solution_quality_;
   }


   /**
            * Screw primitive preview fields
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.geometry.Pose3D>  getPreviewTrajectory()
   {
      return preview_trajectory_;
   }

   public void setPreviewTrajectoryDuration(double preview_trajectory_duration)
   {
      preview_trajectory_duration_ = preview_trajectory_duration;
   }
   public double getPreviewTrajectoryDuration()
   {
      return preview_trajectory_duration_;
   }

   public void setPreviewTrajectoryLinearVelocity(double preview_trajectory_linear_velocity)
   {
      preview_trajectory_linear_velocity_ = preview_trajectory_linear_velocity;
   }
   public double getPreviewTrajectoryLinearVelocity()
   {
      return preview_trajectory_linear_velocity_;
   }

   public void setPreviewTrajectoryAngularVelocity(double preview_trajectory_angular_velocity)
   {
      preview_trajectory_angular_velocity_ = preview_trajectory_angular_velocity;
   }
   public double getPreviewTrajectoryAngularVelocity()
   {
      return preview_trajectory_angular_velocity_;
   }

   /**
            * The user requested IK preview time normalized from 0.0 to 1.0
            */
   public void setPreviewRequestedTime(double preview_requested_time)
   {
      preview_requested_time_ = preview_requested_time;
   }
   /**
            * The user requested IK preview time normalized from 0.0 to 1.0
            */
   public double getPreviewRequestedTime()
   {
      return preview_requested_time_;
   }


   public double[] getPreviewJointAngles()
   {
      return preview_joint_angles_;
   }

   public void setPreviewSolutionQuality(double preview_solution_quality)
   {
      preview_solution_quality_ = preview_solution_quality;
   }
   public double getPreviewSolutionQuality()
   {
      return preview_solution_quality_;
   }


   public static Supplier<ArmActionStateMessagePubSubType> getPubSubType()
   {
      return ArmActionStateMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ArmActionStateMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ArmActionStateMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.state_.epsilonEquals(other.state_, epsilon)) return false;
      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;
      if (!this.goal_chest_transform_to_world_.epsilonEquals(other.goal_chest_transform_to_world_, epsilon)) return false;
      if (!this.force_.epsilonEquals(other.force_, epsilon)) return false;
      if (!this.torque_.epsilonEquals(other.torque_, epsilon)) return false;
      for(int i5 = 0; i5 < joint_angles_.length; ++i5)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.joint_angles_[i5], other.joint_angles_[i5], epsilon)) return false;
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.solution_quality_, other.solution_quality_, epsilon)) return false;

      if (this.preview_trajectory_.size() != other.preview_trajectory_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.preview_trajectory_.size(); i++)
         {  if (!this.preview_trajectory_.get(i).epsilonEquals(other.preview_trajectory_.get(i), epsilon)) return false; }
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.preview_trajectory_duration_, other.preview_trajectory_duration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.preview_trajectory_linear_velocity_, other.preview_trajectory_linear_velocity_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.preview_trajectory_angular_velocity_, other.preview_trajectory_angular_velocity_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.preview_requested_time_, other.preview_requested_time_, epsilon)) return false;

      for(int i7 = 0; i7 < preview_joint_angles_.length; ++i7)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.preview_joint_angles_[i7], other.preview_joint_angles_[i7], epsilon)) return false;
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.preview_solution_quality_, other.preview_solution_quality_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ArmActionStateMessage)) return false;

      ArmActionStateMessage otherMyClass = (ArmActionStateMessage) other;

      if (!this.state_.equals(otherMyClass.state_)) return false;
      if (!this.definition_.equals(otherMyClass.definition_)) return false;
      if (!this.goal_chest_transform_to_world_.equals(otherMyClass.goal_chest_transform_to_world_)) return false;
      if (!this.force_.equals(otherMyClass.force_)) return false;
      if (!this.torque_.equals(otherMyClass.torque_)) return false;
      for(int i9 = 0; i9 < joint_angles_.length; ++i9)
      {
                if(this.joint_angles_[i9] != otherMyClass.joint_angles_[i9]) return false;

      }
      if(this.solution_quality_ != otherMyClass.solution_quality_) return false;

      if (!this.preview_trajectory_.equals(otherMyClass.preview_trajectory_)) return false;
      if(this.preview_trajectory_duration_ != otherMyClass.preview_trajectory_duration_) return false;

      if(this.preview_trajectory_linear_velocity_ != otherMyClass.preview_trajectory_linear_velocity_) return false;

      if(this.preview_trajectory_angular_velocity_ != otherMyClass.preview_trajectory_angular_velocity_) return false;

      if(this.preview_requested_time_ != otherMyClass.preview_requested_time_) return false;

      for(int i11 = 0; i11 < preview_joint_angles_.length; ++i11)
      {
                if(this.preview_joint_angles_[i11] != otherMyClass.preview_joint_angles_[i11]) return false;

      }
      if(this.preview_solution_quality_ != otherMyClass.preview_solution_quality_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ArmActionStateMessage {");
      builder.append("state=");
      builder.append(this.state_);      builder.append(", ");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("goal_chest_transform_to_world=");
      builder.append(this.goal_chest_transform_to_world_);      builder.append(", ");
      builder.append("force=");
      builder.append(this.force_);      builder.append(", ");
      builder.append("torque=");
      builder.append(this.torque_);      builder.append(", ");
      builder.append("joint_angles=");
      builder.append(java.util.Arrays.toString(this.joint_angles_));      builder.append(", ");
      builder.append("solution_quality=");
      builder.append(this.solution_quality_);      builder.append(", ");
      builder.append("preview_trajectory=");
      builder.append(this.preview_trajectory_);      builder.append(", ");
      builder.append("preview_trajectory_duration=");
      builder.append(this.preview_trajectory_duration_);      builder.append(", ");
      builder.append("preview_trajectory_linear_velocity=");
      builder.append(this.preview_trajectory_linear_velocity_);      builder.append(", ");
      builder.append("preview_trajectory_angular_velocity=");
      builder.append(this.preview_trajectory_angular_velocity_);      builder.append(", ");
      builder.append("preview_requested_time=");
      builder.append(this.preview_requested_time_);      builder.append(", ");
      builder.append("preview_joint_angles=");
      builder.append(java.util.Arrays.toString(this.preview_joint_angles_));      builder.append(", ");
      builder.append("preview_solution_quality=");
      builder.append(this.preview_solution_quality_);
      builder.append("}");
      return builder.toString();
   }
}
