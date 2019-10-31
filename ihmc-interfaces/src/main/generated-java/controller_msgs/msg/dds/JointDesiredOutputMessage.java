package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * This message contains joint-level desired values that are output from the whole-body controller
       */
public class JointDesiredOutputMessage extends Packet<JointDesiredOutputMessage> implements Settable<JointDesiredOutputMessage>, EpsilonComparable<JointDesiredOutputMessage>
{
   public static final byte CONTROL_MODE_POSITION = (byte) 0;
   public static final byte CONTROL_VELOCITY = (byte) 1;
   public static final byte CONTROL_EFFORT = (byte) 2;
   public static final byte CONTROL_DISABLED = (byte) 3;
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   public java.lang.StringBuilder joint_name_;
   public byte control_mode_ = (byte) 255;
   public boolean has_desired_torque_;
   public boolean has_desired_position_;
   public boolean has_desired_velocity_;
   public boolean has_desired_acceleration_;
   public boolean has_stiffness_;
   public boolean has_damping_;
   public boolean has_master_gain_;
   public boolean has_velocity_scaling_;
   public boolean has_position_integration_break_frequency_;
   public boolean has_velocity_integration_break_frequency_;
   public boolean has_position_integration_max_error_;
   public boolean has_velocity_integration_max_error_;
   public boolean has_position_feedback_max_error_;
   public boolean has_velocity_feedback_max_error_;
   public double desired_torque_;
   public double desired_position_;
   public double desired_velocity_;
   public double desired_acceleration_;
   public double stiffness_;
   public double damping_;
   public double master_gain_;
   public double velocity_scaling_;
   public double position_integration_break_frequency_;
   public double velocity_integration_break_frequency_;
   public double position_integration_max_error_;
   public double velocity_integration_max_error_;
   public double position_feedback_max_error_;
   public double velocity_feedback_max_error_;

   public JointDesiredOutputMessage()
   {
      joint_name_ = new java.lang.StringBuilder(255);
   }

   public JointDesiredOutputMessage(JointDesiredOutputMessage other)
   {
      this();
      set(other);
   }

   public void set(JointDesiredOutputMessage other)
   {
      sequence_id_ = other.sequence_id_;

      joint_name_.setLength(0);
      joint_name_.append(other.joint_name_);

      control_mode_ = other.control_mode_;

      has_desired_torque_ = other.has_desired_torque_;

      has_desired_position_ = other.has_desired_position_;

      has_desired_velocity_ = other.has_desired_velocity_;

      has_desired_acceleration_ = other.has_desired_acceleration_;

      has_stiffness_ = other.has_stiffness_;

      has_damping_ = other.has_damping_;

      has_master_gain_ = other.has_master_gain_;

      has_velocity_scaling_ = other.has_velocity_scaling_;

      has_position_integration_break_frequency_ = other.has_position_integration_break_frequency_;

      has_velocity_integration_break_frequency_ = other.has_velocity_integration_break_frequency_;

      has_position_integration_max_error_ = other.has_position_integration_max_error_;

      has_velocity_integration_max_error_ = other.has_velocity_integration_max_error_;

      has_position_feedback_max_error_ = other.has_position_feedback_max_error_;

      has_velocity_feedback_max_error_ = other.has_velocity_feedback_max_error_;

      desired_torque_ = other.desired_torque_;

      desired_position_ = other.desired_position_;

      desired_velocity_ = other.desired_velocity_;

      desired_acceleration_ = other.desired_acceleration_;

      stiffness_ = other.stiffness_;

      damping_ = other.damping_;

      master_gain_ = other.master_gain_;

      velocity_scaling_ = other.velocity_scaling_;

      position_integration_break_frequency_ = other.position_integration_break_frequency_;

      velocity_integration_break_frequency_ = other.velocity_integration_break_frequency_;

      position_integration_max_error_ = other.position_integration_max_error_;

      velocity_integration_max_error_ = other.velocity_integration_max_error_;

      position_feedback_max_error_ = other.position_feedback_max_error_;

      velocity_feedback_max_error_ = other.velocity_feedback_max_error_;

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

   public void setJointName(java.lang.String joint_name)
   {
      joint_name_.setLength(0);
      joint_name_.append(joint_name);
   }

   public java.lang.String getJointNameAsString()
   {
      return getJointName().toString();
   }
   public java.lang.StringBuilder getJointName()
   {
      return joint_name_;
   }

   public void setControlMode(byte control_mode)
   {
      control_mode_ = control_mode;
   }
   public byte getControlMode()
   {
      return control_mode_;
   }

   public void setHasDesiredTorque(boolean has_desired_torque)
   {
      has_desired_torque_ = has_desired_torque;
   }
   public boolean getHasDesiredTorque()
   {
      return has_desired_torque_;
   }

   public void setHasDesiredPosition(boolean has_desired_position)
   {
      has_desired_position_ = has_desired_position;
   }
   public boolean getHasDesiredPosition()
   {
      return has_desired_position_;
   }

   public void setHasDesiredVelocity(boolean has_desired_velocity)
   {
      has_desired_velocity_ = has_desired_velocity;
   }
   public boolean getHasDesiredVelocity()
   {
      return has_desired_velocity_;
   }

   public void setHasDesiredAcceleration(boolean has_desired_acceleration)
   {
      has_desired_acceleration_ = has_desired_acceleration;
   }
   public boolean getHasDesiredAcceleration()
   {
      return has_desired_acceleration_;
   }

   public void setHasStiffness(boolean has_stiffness)
   {
      has_stiffness_ = has_stiffness;
   }
   public boolean getHasStiffness()
   {
      return has_stiffness_;
   }

   public void setHasDamping(boolean has_damping)
   {
      has_damping_ = has_damping;
   }
   public boolean getHasDamping()
   {
      return has_damping_;
   }

   public void setHasMasterGain(boolean has_master_gain)
   {
      has_master_gain_ = has_master_gain;
   }
   public boolean getHasMasterGain()
   {
      return has_master_gain_;
   }

   public void setHasVelocityScaling(boolean has_velocity_scaling)
   {
      has_velocity_scaling_ = has_velocity_scaling;
   }
   public boolean getHasVelocityScaling()
   {
      return has_velocity_scaling_;
   }

   public void setHasPositionIntegrationBreakFrequency(boolean has_position_integration_break_frequency)
   {
      has_position_integration_break_frequency_ = has_position_integration_break_frequency;
   }
   public boolean getHasPositionIntegrationBreakFrequency()
   {
      return has_position_integration_break_frequency_;
   }

   public void setHasVelocityIntegrationBreakFrequency(boolean has_velocity_integration_break_frequency)
   {
      has_velocity_integration_break_frequency_ = has_velocity_integration_break_frequency;
   }
   public boolean getHasVelocityIntegrationBreakFrequency()
   {
      return has_velocity_integration_break_frequency_;
   }

   public void setHasPositionIntegrationMaxError(boolean has_position_integration_max_error)
   {
      has_position_integration_max_error_ = has_position_integration_max_error;
   }
   public boolean getHasPositionIntegrationMaxError()
   {
      return has_position_integration_max_error_;
   }

   public void setHasVelocityIntegrationMaxError(boolean has_velocity_integration_max_error)
   {
      has_velocity_integration_max_error_ = has_velocity_integration_max_error;
   }
   public boolean getHasVelocityIntegrationMaxError()
   {
      return has_velocity_integration_max_error_;
   }

   public void setHasPositionFeedbackMaxError(boolean has_position_feedback_max_error)
   {
      has_position_feedback_max_error_ = has_position_feedback_max_error;
   }
   public boolean getHasPositionFeedbackMaxError()
   {
      return has_position_feedback_max_error_;
   }

   public void setHasVelocityFeedbackMaxError(boolean has_velocity_feedback_max_error)
   {
      has_velocity_feedback_max_error_ = has_velocity_feedback_max_error;
   }
   public boolean getHasVelocityFeedbackMaxError()
   {
      return has_velocity_feedback_max_error_;
   }

   public void setDesiredTorque(double desired_torque)
   {
      desired_torque_ = desired_torque;
   }
   public double getDesiredTorque()
   {
      return desired_torque_;
   }

   public void setDesiredPosition(double desired_position)
   {
      desired_position_ = desired_position;
   }
   public double getDesiredPosition()
   {
      return desired_position_;
   }

   public void setDesiredVelocity(double desired_velocity)
   {
      desired_velocity_ = desired_velocity;
   }
   public double getDesiredVelocity()
   {
      return desired_velocity_;
   }

   public void setDesiredAcceleration(double desired_acceleration)
   {
      desired_acceleration_ = desired_acceleration;
   }
   public double getDesiredAcceleration()
   {
      return desired_acceleration_;
   }

   public void setStiffness(double stiffness)
   {
      stiffness_ = stiffness;
   }
   public double getStiffness()
   {
      return stiffness_;
   }

   public void setDamping(double damping)
   {
      damping_ = damping;
   }
   public double getDamping()
   {
      return damping_;
   }

   public void setMasterGain(double master_gain)
   {
      master_gain_ = master_gain;
   }
   public double getMasterGain()
   {
      return master_gain_;
   }

   public void setVelocityScaling(double velocity_scaling)
   {
      velocity_scaling_ = velocity_scaling;
   }
   public double getVelocityScaling()
   {
      return velocity_scaling_;
   }

   public void setPositionIntegrationBreakFrequency(double position_integration_break_frequency)
   {
      position_integration_break_frequency_ = position_integration_break_frequency;
   }
   public double getPositionIntegrationBreakFrequency()
   {
      return position_integration_break_frequency_;
   }

   public void setVelocityIntegrationBreakFrequency(double velocity_integration_break_frequency)
   {
      velocity_integration_break_frequency_ = velocity_integration_break_frequency;
   }
   public double getVelocityIntegrationBreakFrequency()
   {
      return velocity_integration_break_frequency_;
   }

   public void setPositionIntegrationMaxError(double position_integration_max_error)
   {
      position_integration_max_error_ = position_integration_max_error;
   }
   public double getPositionIntegrationMaxError()
   {
      return position_integration_max_error_;
   }

   public void setVelocityIntegrationMaxError(double velocity_integration_max_error)
   {
      velocity_integration_max_error_ = velocity_integration_max_error;
   }
   public double getVelocityIntegrationMaxError()
   {
      return velocity_integration_max_error_;
   }

   public void setPositionFeedbackMaxError(double position_feedback_max_error)
   {
      position_feedback_max_error_ = position_feedback_max_error;
   }
   public double getPositionFeedbackMaxError()
   {
      return position_feedback_max_error_;
   }

   public void setVelocityFeedbackMaxError(double velocity_feedback_max_error)
   {
      velocity_feedback_max_error_ = velocity_feedback_max_error;
   }
   public double getVelocityFeedbackMaxError()
   {
      return velocity_feedback_max_error_;
   }


   public static Supplier<JointDesiredOutputMessagePubSubType> getPubSubType()
   {
      return JointDesiredOutputMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return JointDesiredOutputMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(JointDesiredOutputMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.joint_name_, other.joint_name_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.control_mode_, other.control_mode_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.has_desired_torque_, other.has_desired_torque_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.has_desired_position_, other.has_desired_position_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.has_desired_velocity_, other.has_desired_velocity_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.has_desired_acceleration_, other.has_desired_acceleration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.has_stiffness_, other.has_stiffness_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.has_damping_, other.has_damping_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.has_master_gain_, other.has_master_gain_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.has_velocity_scaling_, other.has_velocity_scaling_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.has_position_integration_break_frequency_, other.has_position_integration_break_frequency_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.has_velocity_integration_break_frequency_, other.has_velocity_integration_break_frequency_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.has_position_integration_max_error_, other.has_position_integration_max_error_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.has_velocity_integration_max_error_, other.has_velocity_integration_max_error_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.has_position_feedback_max_error_, other.has_position_feedback_max_error_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.has_velocity_feedback_max_error_, other.has_velocity_feedback_max_error_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.desired_torque_, other.desired_torque_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.desired_position_, other.desired_position_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.desired_velocity_, other.desired_velocity_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.desired_acceleration_, other.desired_acceleration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.stiffness_, other.stiffness_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.damping_, other.damping_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.master_gain_, other.master_gain_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.velocity_scaling_, other.velocity_scaling_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.position_integration_break_frequency_, other.position_integration_break_frequency_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.velocity_integration_break_frequency_, other.velocity_integration_break_frequency_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.position_integration_max_error_, other.position_integration_max_error_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.velocity_integration_max_error_, other.velocity_integration_max_error_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.position_feedback_max_error_, other.position_feedback_max_error_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.velocity_feedback_max_error_, other.velocity_feedback_max_error_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof JointDesiredOutputMessage)) return false;

      JointDesiredOutputMessage otherMyClass = (JointDesiredOutputMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.joint_name_, otherMyClass.joint_name_)) return false;

      if(this.control_mode_ != otherMyClass.control_mode_) return false;

      if(this.has_desired_torque_ != otherMyClass.has_desired_torque_) return false;

      if(this.has_desired_position_ != otherMyClass.has_desired_position_) return false;

      if(this.has_desired_velocity_ != otherMyClass.has_desired_velocity_) return false;

      if(this.has_desired_acceleration_ != otherMyClass.has_desired_acceleration_) return false;

      if(this.has_stiffness_ != otherMyClass.has_stiffness_) return false;

      if(this.has_damping_ != otherMyClass.has_damping_) return false;

      if(this.has_master_gain_ != otherMyClass.has_master_gain_) return false;

      if(this.has_velocity_scaling_ != otherMyClass.has_velocity_scaling_) return false;

      if(this.has_position_integration_break_frequency_ != otherMyClass.has_position_integration_break_frequency_) return false;

      if(this.has_velocity_integration_break_frequency_ != otherMyClass.has_velocity_integration_break_frequency_) return false;

      if(this.has_position_integration_max_error_ != otherMyClass.has_position_integration_max_error_) return false;

      if(this.has_velocity_integration_max_error_ != otherMyClass.has_velocity_integration_max_error_) return false;

      if(this.has_position_feedback_max_error_ != otherMyClass.has_position_feedback_max_error_) return false;

      if(this.has_velocity_feedback_max_error_ != otherMyClass.has_velocity_feedback_max_error_) return false;

      if(this.desired_torque_ != otherMyClass.desired_torque_) return false;

      if(this.desired_position_ != otherMyClass.desired_position_) return false;

      if(this.desired_velocity_ != otherMyClass.desired_velocity_) return false;

      if(this.desired_acceleration_ != otherMyClass.desired_acceleration_) return false;

      if(this.stiffness_ != otherMyClass.stiffness_) return false;

      if(this.damping_ != otherMyClass.damping_) return false;

      if(this.master_gain_ != otherMyClass.master_gain_) return false;

      if(this.velocity_scaling_ != otherMyClass.velocity_scaling_) return false;

      if(this.position_integration_break_frequency_ != otherMyClass.position_integration_break_frequency_) return false;

      if(this.velocity_integration_break_frequency_ != otherMyClass.velocity_integration_break_frequency_) return false;

      if(this.position_integration_max_error_ != otherMyClass.position_integration_max_error_) return false;

      if(this.velocity_integration_max_error_ != otherMyClass.velocity_integration_max_error_) return false;

      if(this.position_feedback_max_error_ != otherMyClass.position_feedback_max_error_) return false;

      if(this.velocity_feedback_max_error_ != otherMyClass.velocity_feedback_max_error_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("JointDesiredOutputMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("joint_name=");
      builder.append(this.joint_name_);      builder.append(", ");
      builder.append("control_mode=");
      builder.append(this.control_mode_);      builder.append(", ");
      builder.append("has_desired_torque=");
      builder.append(this.has_desired_torque_);      builder.append(", ");
      builder.append("has_desired_position=");
      builder.append(this.has_desired_position_);      builder.append(", ");
      builder.append("has_desired_velocity=");
      builder.append(this.has_desired_velocity_);      builder.append(", ");
      builder.append("has_desired_acceleration=");
      builder.append(this.has_desired_acceleration_);      builder.append(", ");
      builder.append("has_stiffness=");
      builder.append(this.has_stiffness_);      builder.append(", ");
      builder.append("has_damping=");
      builder.append(this.has_damping_);      builder.append(", ");
      builder.append("has_master_gain=");
      builder.append(this.has_master_gain_);      builder.append(", ");
      builder.append("has_velocity_scaling=");
      builder.append(this.has_velocity_scaling_);      builder.append(", ");
      builder.append("has_position_integration_break_frequency=");
      builder.append(this.has_position_integration_break_frequency_);      builder.append(", ");
      builder.append("has_velocity_integration_break_frequency=");
      builder.append(this.has_velocity_integration_break_frequency_);      builder.append(", ");
      builder.append("has_position_integration_max_error=");
      builder.append(this.has_position_integration_max_error_);      builder.append(", ");
      builder.append("has_velocity_integration_max_error=");
      builder.append(this.has_velocity_integration_max_error_);      builder.append(", ");
      builder.append("has_position_feedback_max_error=");
      builder.append(this.has_position_feedback_max_error_);      builder.append(", ");
      builder.append("has_velocity_feedback_max_error=");
      builder.append(this.has_velocity_feedback_max_error_);      builder.append(", ");
      builder.append("desired_torque=");
      builder.append(this.desired_torque_);      builder.append(", ");
      builder.append("desired_position=");
      builder.append(this.desired_position_);      builder.append(", ");
      builder.append("desired_velocity=");
      builder.append(this.desired_velocity_);      builder.append(", ");
      builder.append("desired_acceleration=");
      builder.append(this.desired_acceleration_);      builder.append(", ");
      builder.append("stiffness=");
      builder.append(this.stiffness_);      builder.append(", ");
      builder.append("damping=");
      builder.append(this.damping_);      builder.append(", ");
      builder.append("master_gain=");
      builder.append(this.master_gain_);      builder.append(", ");
      builder.append("velocity_scaling=");
      builder.append(this.velocity_scaling_);      builder.append(", ");
      builder.append("position_integration_break_frequency=");
      builder.append(this.position_integration_break_frequency_);      builder.append(", ");
      builder.append("velocity_integration_break_frequency=");
      builder.append(this.velocity_integration_break_frequency_);      builder.append(", ");
      builder.append("position_integration_max_error=");
      builder.append(this.position_integration_max_error_);      builder.append(", ");
      builder.append("velocity_integration_max_error=");
      builder.append(this.velocity_integration_max_error_);      builder.append(", ");
      builder.append("position_feedback_max_error=");
      builder.append(this.position_feedback_max_error_);      builder.append(", ");
      builder.append("velocity_feedback_max_error=");
      builder.append(this.velocity_feedback_max_error_);
      builder.append("}");
      return builder.toString();
   }
}
