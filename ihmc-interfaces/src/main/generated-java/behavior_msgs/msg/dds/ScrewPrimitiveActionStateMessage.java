package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class ScrewPrimitiveActionStateMessage extends Packet<ScrewPrimitiveActionStateMessage> implements Settable<ScrewPrimitiveActionStateMessage>, EpsilonComparable<ScrewPrimitiveActionStateMessage>
{
   /**
            * Parent state fields
            */
   public behavior_msgs.msg.dds.ActionNodeStateMessage state_;
   /**
            * Definition
            */
   public behavior_msgs.msg.dds.ScrewPrimitiveActionDefinitionMessage definition_;
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.geometry.Pose3D>  preview_trajectory_;
   /**
            * Hand force
            */
   public us.ihmc.euclid.tuple3D.Vector3D force_;
   /**
            * Hand torque
            */
   public us.ihmc.euclid.tuple3D.Vector3D torque_;
   public double preview_trajectory_duration_;
   public double preview_trajectory_linear_velocity_;
   public double preview_trajectory_angular_velocity_;
   /**
            * The user requested IK preview time normalized from 0.0 to 1.0
            */
   public double preview_requested_time_;
   public double[] preview_joint_angles_;
   public double preview_solution_quality_;

   public ScrewPrimitiveActionStateMessage()
   {
      state_ = new behavior_msgs.msg.dds.ActionNodeStateMessage();
      definition_ = new behavior_msgs.msg.dds.ScrewPrimitiveActionDefinitionMessage();
      preview_trajectory_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.geometry.Pose3D> (50, new geometry_msgs.msg.dds.PosePubSubType());
      force_ = new us.ihmc.euclid.tuple3D.Vector3D();
      torque_ = new us.ihmc.euclid.tuple3D.Vector3D();
      preview_joint_angles_ = new double[7];


   }

   public ScrewPrimitiveActionStateMessage(ScrewPrimitiveActionStateMessage other)
   {
      this();
      set(other);
   }

   public void set(ScrewPrimitiveActionStateMessage other)
   {
      behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.staticCopy(other.state_, state_);
      behavior_msgs.msg.dds.ScrewPrimitiveActionDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      preview_trajectory_.set(other.preview_trajectory_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.force_, force_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.torque_, torque_);
      preview_trajectory_duration_ = other.preview_trajectory_duration_;

      preview_trajectory_linear_velocity_ = other.preview_trajectory_linear_velocity_;

      preview_trajectory_angular_velocity_ = other.preview_trajectory_angular_velocity_;

      preview_requested_time_ = other.preview_requested_time_;

      for(int i1 = 0; i1 < preview_joint_angles_.length; ++i1)
      {
            preview_joint_angles_[i1] = other.preview_joint_angles_[i1];

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
   public behavior_msgs.msg.dds.ScrewPrimitiveActionDefinitionMessage getDefinition()
   {
      return definition_;
   }


   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.geometry.Pose3D>  getPreviewTrajectory()
   {
      return preview_trajectory_;
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


   public static Supplier<ScrewPrimitiveActionStateMessagePubSubType> getPubSubType()
   {
      return ScrewPrimitiveActionStateMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ScrewPrimitiveActionStateMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ScrewPrimitiveActionStateMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.state_.epsilonEquals(other.state_, epsilon)) return false;
      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;
      if (this.preview_trajectory_.size() != other.preview_trajectory_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.preview_trajectory_.size(); i++)
         {  if (!this.preview_trajectory_.get(i).epsilonEquals(other.preview_trajectory_.get(i), epsilon)) return false; }
      }

      if (!this.force_.epsilonEquals(other.force_, epsilon)) return false;
      if (!this.torque_.epsilonEquals(other.torque_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.preview_trajectory_duration_, other.preview_trajectory_duration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.preview_trajectory_linear_velocity_, other.preview_trajectory_linear_velocity_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.preview_trajectory_angular_velocity_, other.preview_trajectory_angular_velocity_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.preview_requested_time_, other.preview_requested_time_, epsilon)) return false;

      for(int i3 = 0; i3 < preview_joint_angles_.length; ++i3)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.preview_joint_angles_[i3], other.preview_joint_angles_[i3], epsilon)) return false;
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.preview_solution_quality_, other.preview_solution_quality_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ScrewPrimitiveActionStateMessage)) return false;

      ScrewPrimitiveActionStateMessage otherMyClass = (ScrewPrimitiveActionStateMessage) other;

      if (!this.state_.equals(otherMyClass.state_)) return false;
      if (!this.definition_.equals(otherMyClass.definition_)) return false;
      if (!this.preview_trajectory_.equals(otherMyClass.preview_trajectory_)) return false;
      if (!this.force_.equals(otherMyClass.force_)) return false;
      if (!this.torque_.equals(otherMyClass.torque_)) return false;
      if(this.preview_trajectory_duration_ != otherMyClass.preview_trajectory_duration_) return false;

      if(this.preview_trajectory_linear_velocity_ != otherMyClass.preview_trajectory_linear_velocity_) return false;

      if(this.preview_trajectory_angular_velocity_ != otherMyClass.preview_trajectory_angular_velocity_) return false;

      if(this.preview_requested_time_ != otherMyClass.preview_requested_time_) return false;

      for(int i5 = 0; i5 < preview_joint_angles_.length; ++i5)
      {
                if(this.preview_joint_angles_[i5] != otherMyClass.preview_joint_angles_[i5]) return false;

      }
      if(this.preview_solution_quality_ != otherMyClass.preview_solution_quality_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ScrewPrimitiveActionStateMessage {");
      builder.append("state=");
      builder.append(this.state_);      builder.append(", ");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("preview_trajectory=");
      builder.append(this.preview_trajectory_);      builder.append(", ");
      builder.append("force=");
      builder.append(this.force_);      builder.append(", ");
      builder.append("torque=");
      builder.append(this.torque_);      builder.append(", ");
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
