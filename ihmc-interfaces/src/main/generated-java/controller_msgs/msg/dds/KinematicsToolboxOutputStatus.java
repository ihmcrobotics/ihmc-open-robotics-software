package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * General purpose message normally used to report the solution of a whole-body inverse kinematics solver.
       * Main usage is for the IHMC KinematicsToolbox.
       */
public class KinematicsToolboxOutputStatus extends Packet<KinematicsToolboxOutputStatus> implements Settable<KinematicsToolboxOutputStatus>, EpsilonComparable<KinematicsToolboxOutputStatus>
{
   /**
          * Nothing reported by the toolbox.
          */
   public static final byte CURRENT_TOOLBOX_STATE_NO_STATUS = (byte) 0;
   /**
          * The toolbox just initialized successfully and is about to start running.
          */
   public static final byte CURRENT_TOOLBOX_STATE_INITIALIZE_SUCCESSFUL = (byte) 1;
   /**
          * The toolbox failed its initialization and cannot run until it succeeds.
          * This failure specifies that the toolbox has not received RobotConfigurationData from the IHMC walking controller.
          */
   public static final byte CURRENT_TOOLBOX_STATE_INITIALIZE_FAILURE_MISSING_RCD = (byte) 2;
   /**
          * The toolbox has been initialized properly and is running.
          */
   public static final byte CURRENT_TOOLBOX_STATE_RUNNING = (byte) 3;
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Provides insight about the current state of the toolbox, e.g. waiting for user input or waiting for controller input.
            */
   public byte current_toolbox_state_;
   public int joint_name_hash_;
   public us.ihmc.idl.IDLSequence.Float  desired_joint_angles_;
   public us.ihmc.euclid.tuple3D.Vector3D desired_root_translation_;
   public us.ihmc.euclid.tuple4D.Quaternion desired_root_orientation_;
   /**
            * Desired joint velocities might be empty.
            */
   public us.ihmc.idl.IDLSequence.Float  desired_joint_velocities_;
   /**
            * Desired twist of root might be empty.
            */
   public us.ihmc.euclid.tuple3D.Vector3D desired_root_linear_velocity_;
   public us.ihmc.euclid.tuple3D.Vector3D desired_root_angular_velocity_;
   public double solution_quality_ = -1.0;

   public KinematicsToolboxOutputStatus()
   {
      desired_joint_angles_ = new us.ihmc.idl.IDLSequence.Float (100, "type_5");

      desired_root_translation_ = new us.ihmc.euclid.tuple3D.Vector3D();
      desired_root_orientation_ = new us.ihmc.euclid.tuple4D.Quaternion();
      desired_joint_velocities_ = new us.ihmc.idl.IDLSequence.Float (100, "type_5");

      desired_root_linear_velocity_ = new us.ihmc.euclid.tuple3D.Vector3D();
      desired_root_angular_velocity_ = new us.ihmc.euclid.tuple3D.Vector3D();
   }

   public KinematicsToolboxOutputStatus(KinematicsToolboxOutputStatus other)
   {
      this();
      set(other);
   }

   public void set(KinematicsToolboxOutputStatus other)
   {
      sequence_id_ = other.sequence_id_;

      current_toolbox_state_ = other.current_toolbox_state_;

      joint_name_hash_ = other.joint_name_hash_;

      desired_joint_angles_.set(other.desired_joint_angles_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.desired_root_translation_, desired_root_translation_);
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.desired_root_orientation_, desired_root_orientation_);
      desired_joint_velocities_.set(other.desired_joint_velocities_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.desired_root_linear_velocity_, desired_root_linear_velocity_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.desired_root_angular_velocity_, desired_root_angular_velocity_);
      solution_quality_ = other.solution_quality_;

   }

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public void setSequenceId(long sequence_id)
   {
      sequence_id_ = sequence_id;
   }
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long getSequenceId()
   {
      return sequence_id_;
   }

   /**
            * Provides insight about the current state of the toolbox, e.g. waiting for user input or waiting for controller input.
            */
   public void setCurrentToolboxState(byte current_toolbox_state)
   {
      current_toolbox_state_ = current_toolbox_state;
   }
   /**
            * Provides insight about the current state of the toolbox, e.g. waiting for user input or waiting for controller input.
            */
   public byte getCurrentToolboxState()
   {
      return current_toolbox_state_;
   }

   public void setJointNameHash(int joint_name_hash)
   {
      joint_name_hash_ = joint_name_hash;
   }
   public int getJointNameHash()
   {
      return joint_name_hash_;
   }


   public us.ihmc.idl.IDLSequence.Float  getDesiredJointAngles()
   {
      return desired_joint_angles_;
   }


   public us.ihmc.euclid.tuple3D.Vector3D getDesiredRootTranslation()
   {
      return desired_root_translation_;
   }


   public us.ihmc.euclid.tuple4D.Quaternion getDesiredRootOrientation()
   {
      return desired_root_orientation_;
   }


   /**
            * Desired joint velocities might be empty.
            */
   public us.ihmc.idl.IDLSequence.Float  getDesiredJointVelocities()
   {
      return desired_joint_velocities_;
   }


   /**
            * Desired twist of root might be empty.
            */
   public us.ihmc.euclid.tuple3D.Vector3D getDesiredRootLinearVelocity()
   {
      return desired_root_linear_velocity_;
   }


   public us.ihmc.euclid.tuple3D.Vector3D getDesiredRootAngularVelocity()
   {
      return desired_root_angular_velocity_;
   }

   public void setSolutionQuality(double solution_quality)
   {
      solution_quality_ = solution_quality;
   }
   public double getSolutionQuality()
   {
      return solution_quality_;
   }


   public static Supplier<KinematicsToolboxOutputStatusPubSubType> getPubSubType()
   {
      return KinematicsToolboxOutputStatusPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return KinematicsToolboxOutputStatusPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(KinematicsToolboxOutputStatus other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_toolbox_state_, other.current_toolbox_state_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.joint_name_hash_, other.joint_name_hash_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.desired_joint_angles_, other.desired_joint_angles_, epsilon)) return false;

      if (!this.desired_root_translation_.epsilonEquals(other.desired_root_translation_, epsilon)) return false;
      if (!this.desired_root_orientation_.epsilonEquals(other.desired_root_orientation_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.desired_joint_velocities_, other.desired_joint_velocities_, epsilon)) return false;

      if (!this.desired_root_linear_velocity_.epsilonEquals(other.desired_root_linear_velocity_, epsilon)) return false;
      if (!this.desired_root_angular_velocity_.epsilonEquals(other.desired_root_angular_velocity_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.solution_quality_, other.solution_quality_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof KinematicsToolboxOutputStatus)) return false;

      KinematicsToolboxOutputStatus otherMyClass = (KinematicsToolboxOutputStatus) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.current_toolbox_state_ != otherMyClass.current_toolbox_state_) return false;

      if(this.joint_name_hash_ != otherMyClass.joint_name_hash_) return false;

      if (!this.desired_joint_angles_.equals(otherMyClass.desired_joint_angles_)) return false;
      if (!this.desired_root_translation_.equals(otherMyClass.desired_root_translation_)) return false;
      if (!this.desired_root_orientation_.equals(otherMyClass.desired_root_orientation_)) return false;
      if (!this.desired_joint_velocities_.equals(otherMyClass.desired_joint_velocities_)) return false;
      if (!this.desired_root_linear_velocity_.equals(otherMyClass.desired_root_linear_velocity_)) return false;
      if (!this.desired_root_angular_velocity_.equals(otherMyClass.desired_root_angular_velocity_)) return false;
      if(this.solution_quality_ != otherMyClass.solution_quality_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("KinematicsToolboxOutputStatus {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("current_toolbox_state=");
      builder.append(this.current_toolbox_state_);      builder.append(", ");
      builder.append("joint_name_hash=");
      builder.append(this.joint_name_hash_);      builder.append(", ");
      builder.append("desired_joint_angles=");
      builder.append(this.desired_joint_angles_);      builder.append(", ");
      builder.append("desired_root_translation=");
      builder.append(this.desired_root_translation_);      builder.append(", ");
      builder.append("desired_root_orientation=");
      builder.append(this.desired_root_orientation_);      builder.append(", ");
      builder.append("desired_joint_velocities=");
      builder.append(this.desired_joint_velocities_);      builder.append(", ");
      builder.append("desired_root_linear_velocity=");
      builder.append(this.desired_root_linear_velocity_);      builder.append(", ");
      builder.append("desired_root_angular_velocity=");
      builder.append(this.desired_root_angular_velocity_);      builder.append(", ");
      builder.append("solution_quality=");
      builder.append(this.solution_quality_);
      builder.append("}");
      return builder.toString();
   }
}
