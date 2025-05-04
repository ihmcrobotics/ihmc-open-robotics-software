package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Message for commanding the Sake hands to perform various predefined grasps.
       * Also allows for custom grasps with set positions/torques
       */
public class SakeHandDesiredCommandMessage extends Packet<SakeHandDesiredCommandMessage> implements Settable<SakeHandDesiredCommandMessage>, EpsilonComparable<SakeHandDesiredCommandMessage>
{
   public static final byte ROBOT_SIDE_LEFT = (byte) 0;
   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;
   /**
            * Specifies the side of the robot of the hand being referred to
            */
   public byte robot_side_ = (byte) 255;
   /**
            * Request the gripper to perform a calibration sequence
            */
   public boolean request_calibration_;
   /**
            * Request to reset the gripper error state after overheating
            */
   public boolean request_reset_errors_;
   /**
            * Set to true to enable automatic cooldown
            */
   public boolean enable_automatic_cooldown_;
   /**
            * Set to true to disable automatic cooldown
            */
   public boolean disable_automatic_cooldown_;
   /**
            * Set to true to override the cooldown
            */
   public boolean override_cooldown_;
   /**
            * The desired dynamixel position, in radians
            */
   public double gripper_desired_position_;
   /**
            * The dynamixel torque limit setting in achieving the desired position
            * 0: dynamixel will not apply any force and will not achieve desired position
            * 300: A reasonable normal value
            * 1023: dynamixel max torque which will quickly overheat the motor
            */
   public int raw_gripper_torque_limit_;
   /**
            * Keeping the torque off when not needed can help keep the hand's temperature down
            */
   public boolean torque_on_;
   /**
            * The timestamp (in nanoseconds) at which this message was logged
            */
   public long log_timestamp_;

   public SakeHandDesiredCommandMessage()
   {
   }

   public SakeHandDesiredCommandMessage(SakeHandDesiredCommandMessage other)
   {
      this();
      set(other);
   }

   public void set(SakeHandDesiredCommandMessage other)
   {
      robot_side_ = other.robot_side_;

      request_calibration_ = other.request_calibration_;

      request_reset_errors_ = other.request_reset_errors_;

      enable_automatic_cooldown_ = other.enable_automatic_cooldown_;

      disable_automatic_cooldown_ = other.disable_automatic_cooldown_;

      override_cooldown_ = other.override_cooldown_;

      gripper_desired_position_ = other.gripper_desired_position_;

      raw_gripper_torque_limit_ = other.raw_gripper_torque_limit_;

      torque_on_ = other.torque_on_;

      log_timestamp_ = other.log_timestamp_;

   }

   /**
            * Specifies the side of the robot of the hand being referred to
            */
   public void setRobotSide(byte robot_side)
   {
      robot_side_ = robot_side;
   }
   /**
            * Specifies the side of the robot of the hand being referred to
            */
   public byte getRobotSide()
   {
      return robot_side_;
   }

   /**
            * Request the gripper to perform a calibration sequence
            */
   public void setRequestCalibration(boolean request_calibration)
   {
      request_calibration_ = request_calibration;
   }
   /**
            * Request the gripper to perform a calibration sequence
            */
   public boolean getRequestCalibration()
   {
      return request_calibration_;
   }

   /**
            * Request to reset the gripper error state after overheating
            */
   public void setRequestResetErrors(boolean request_reset_errors)
   {
      request_reset_errors_ = request_reset_errors;
   }
   /**
            * Request to reset the gripper error state after overheating
            */
   public boolean getRequestResetErrors()
   {
      return request_reset_errors_;
   }

   /**
            * Set to true to enable automatic cooldown
            */
   public void setEnableAutomaticCooldown(boolean enable_automatic_cooldown)
   {
      enable_automatic_cooldown_ = enable_automatic_cooldown;
   }
   /**
            * Set to true to enable automatic cooldown
            */
   public boolean getEnableAutomaticCooldown()
   {
      return enable_automatic_cooldown_;
   }

   /**
            * Set to true to disable automatic cooldown
            */
   public void setDisableAutomaticCooldown(boolean disable_automatic_cooldown)
   {
      disable_automatic_cooldown_ = disable_automatic_cooldown;
   }
   /**
            * Set to true to disable automatic cooldown
            */
   public boolean getDisableAutomaticCooldown()
   {
      return disable_automatic_cooldown_;
   }

   /**
            * Set to true to override the cooldown
            */
   public void setOverrideCooldown(boolean override_cooldown)
   {
      override_cooldown_ = override_cooldown;
   }
   /**
            * Set to true to override the cooldown
            */
   public boolean getOverrideCooldown()
   {
      return override_cooldown_;
   }

   /**
            * The desired dynamixel position, in radians
            */
   public void setGripperDesiredPosition(double gripper_desired_position)
   {
      gripper_desired_position_ = gripper_desired_position;
   }
   /**
            * The desired dynamixel position, in radians
            */
   public double getGripperDesiredPosition()
   {
      return gripper_desired_position_;
   }

   /**
            * The dynamixel torque limit setting in achieving the desired position
            * 0: dynamixel will not apply any force and will not achieve desired position
            * 300: A reasonable normal value
            * 1023: dynamixel max torque which will quickly overheat the motor
            */
   public void setRawGripperTorqueLimit(int raw_gripper_torque_limit)
   {
      raw_gripper_torque_limit_ = raw_gripper_torque_limit;
   }
   /**
            * The dynamixel torque limit setting in achieving the desired position
            * 0: dynamixel will not apply any force and will not achieve desired position
            * 300: A reasonable normal value
            * 1023: dynamixel max torque which will quickly overheat the motor
            */
   public int getRawGripperTorqueLimit()
   {
      return raw_gripper_torque_limit_;
   }

   /**
            * Keeping the torque off when not needed can help keep the hand's temperature down
            */
   public void setTorqueOn(boolean torque_on)
   {
      torque_on_ = torque_on;
   }
   /**
            * Keeping the torque off when not needed can help keep the hand's temperature down
            */
   public boolean getTorqueOn()
   {
      return torque_on_;
   }

   /**
            * The timestamp (in nanoseconds) at which this message was logged
            */
   public void setLogTimestamp(long log_timestamp)
   {
      log_timestamp_ = log_timestamp;
   }
   /**
            * The timestamp (in nanoseconds) at which this message was logged
            */
   public long getLogTimestamp()
   {
      return log_timestamp_;
   }


   public static Supplier<SakeHandDesiredCommandMessagePubSubType> getPubSubType()
   {
      return SakeHandDesiredCommandMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return SakeHandDesiredCommandMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(SakeHandDesiredCommandMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.request_calibration_, other.request_calibration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.request_reset_errors_, other.request_reset_errors_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.enable_automatic_cooldown_, other.enable_automatic_cooldown_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.disable_automatic_cooldown_, other.disable_automatic_cooldown_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.override_cooldown_, other.override_cooldown_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.gripper_desired_position_, other.gripper_desired_position_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.raw_gripper_torque_limit_, other.raw_gripper_torque_limit_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.torque_on_, other.torque_on_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.log_timestamp_, other.log_timestamp_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof SakeHandDesiredCommandMessage)) return false;

      SakeHandDesiredCommandMessage otherMyClass = (SakeHandDesiredCommandMessage) other;

      if(this.robot_side_ != otherMyClass.robot_side_) return false;

      if(this.request_calibration_ != otherMyClass.request_calibration_) return false;

      if(this.request_reset_errors_ != otherMyClass.request_reset_errors_) return false;

      if(this.enable_automatic_cooldown_ != otherMyClass.enable_automatic_cooldown_) return false;

      if(this.disable_automatic_cooldown_ != otherMyClass.disable_automatic_cooldown_) return false;

      if(this.override_cooldown_ != otherMyClass.override_cooldown_) return false;

      if(this.gripper_desired_position_ != otherMyClass.gripper_desired_position_) return false;

      if(this.raw_gripper_torque_limit_ != otherMyClass.raw_gripper_torque_limit_) return false;

      if(this.torque_on_ != otherMyClass.torque_on_) return false;

      if(this.log_timestamp_ != otherMyClass.log_timestamp_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("SakeHandDesiredCommandMessage {");
      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");
      builder.append("request_calibration=");
      builder.append(this.request_calibration_);      builder.append(", ");
      builder.append("request_reset_errors=");
      builder.append(this.request_reset_errors_);      builder.append(", ");
      builder.append("enable_automatic_cooldown=");
      builder.append(this.enable_automatic_cooldown_);      builder.append(", ");
      builder.append("disable_automatic_cooldown=");
      builder.append(this.disable_automatic_cooldown_);      builder.append(", ");
      builder.append("override_cooldown=");
      builder.append(this.override_cooldown_);      builder.append(", ");
      builder.append("gripper_desired_position=");
      builder.append(this.gripper_desired_position_);      builder.append(", ");
      builder.append("raw_gripper_torque_limit=");
      builder.append(this.raw_gripper_torque_limit_);      builder.append(", ");
      builder.append("torque_on=");
      builder.append(this.torque_on_);      builder.append(", ");
      builder.append("log_timestamp=");
      builder.append(this.log_timestamp_);
      builder.append("}");
      return builder.toString();
   }
}
