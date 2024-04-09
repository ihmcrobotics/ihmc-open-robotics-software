package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Message used to report the current joint angles for the fingers of the sake gripper.
       */
public class SakeHandStatusMessage extends Packet<SakeHandStatusMessage> implements Settable<SakeHandStatusMessage>, EpsilonComparable<SakeHandStatusMessage>
{
   public static final byte ROBOT_SIDE_LEFT = (byte) 0;
   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;
   /**
            * Specifies the side of the robot of the hand being referred to
            */
   public byte robot_side_ = (byte) 255;
   /**
            * Angle at which the hand is closed, in radians
            */
   public double position_upper_limit_;
   /**
            * Angle at which the hand is fully open, in radians
            */
   public double position_lower_limit_;
   /**
            * Temperature of the Dynamixel in Celsius
            */
   public int temperature_;
   /**
            * The current dynamixel position, in radians
            */
   public double current_position_;
   /**
            * The current dynamixel torque
            * 0: dynamixel will not apply any force and will not achieve desired position
            * 300: A reasonable normal value
            * 1023: dynamixel max torque which will quickly overheat the motor
            */
   public int raw_current_torque_;
   /**
            * The position the Dynamixel is trying to achieve, in radians
            */
   public double desired_position_status_;
   /**
            * Torque limit set on the Dynamixel
            */
   public double raw_torque_limit_status_;
   public boolean torque_on_status_;
   /**
            * Rotation velocity of the Dynamixel, in rad/s
            * Positive = closing hand (CCW rotation)
            * Negative = opening hand (CW rotation)
            */
   public double current_velocity_;
   /**
            * Dynamixel's error codes
            * See: https://emanual.robotis.com/docs/en/dxl/protocol1/#error
            */
   public int error_codes_;
   /**
            * Realtime tick of the Dynamixel
            * If this value isn't changing, communication with the hand is broken
            */
   public int realtime_tick_;
   public boolean is_calibrated_;
   public boolean needs_reset_;

   public SakeHandStatusMessage()
   {
   }

   public SakeHandStatusMessage(SakeHandStatusMessage other)
   {
      this();
      set(other);
   }

   public void set(SakeHandStatusMessage other)
   {
      robot_side_ = other.robot_side_;

      position_upper_limit_ = other.position_upper_limit_;

      position_lower_limit_ = other.position_lower_limit_;

      temperature_ = other.temperature_;

      current_position_ = other.current_position_;

      raw_current_torque_ = other.raw_current_torque_;

      desired_position_status_ = other.desired_position_status_;

      raw_torque_limit_status_ = other.raw_torque_limit_status_;

      torque_on_status_ = other.torque_on_status_;

      current_velocity_ = other.current_velocity_;

      error_codes_ = other.error_codes_;

      realtime_tick_ = other.realtime_tick_;

      is_calibrated_ = other.is_calibrated_;

      needs_reset_ = other.needs_reset_;

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
            * Angle at which the hand is closed, in radians
            */
   public void setPositionUpperLimit(double position_upper_limit)
   {
      position_upper_limit_ = position_upper_limit;
   }
   /**
            * Angle at which the hand is closed, in radians
            */
   public double getPositionUpperLimit()
   {
      return position_upper_limit_;
   }

   /**
            * Angle at which the hand is fully open, in radians
            */
   public void setPositionLowerLimit(double position_lower_limit)
   {
      position_lower_limit_ = position_lower_limit;
   }
   /**
            * Angle at which the hand is fully open, in radians
            */
   public double getPositionLowerLimit()
   {
      return position_lower_limit_;
   }

   /**
            * Temperature of the Dynamixel in Celsius
            */
   public void setTemperature(int temperature)
   {
      temperature_ = temperature;
   }
   /**
            * Temperature of the Dynamixel in Celsius
            */
   public int getTemperature()
   {
      return temperature_;
   }

   /**
            * The current dynamixel position, in radians
            */
   public void setCurrentPosition(double current_position)
   {
      current_position_ = current_position;
   }
   /**
            * The current dynamixel position, in radians
            */
   public double getCurrentPosition()
   {
      return current_position_;
   }

   /**
            * The current dynamixel torque
            * 0: dynamixel will not apply any force and will not achieve desired position
            * 300: A reasonable normal value
            * 1023: dynamixel max torque which will quickly overheat the motor
            */
   public void setRawCurrentTorque(int raw_current_torque)
   {
      raw_current_torque_ = raw_current_torque;
   }
   /**
            * The current dynamixel torque
            * 0: dynamixel will not apply any force and will not achieve desired position
            * 300: A reasonable normal value
            * 1023: dynamixel max torque which will quickly overheat the motor
            */
   public int getRawCurrentTorque()
   {
      return raw_current_torque_;
   }

   /**
            * The position the Dynamixel is trying to achieve, in radians
            */
   public void setDesiredPositionStatus(double desired_position_status)
   {
      desired_position_status_ = desired_position_status;
   }
   /**
            * The position the Dynamixel is trying to achieve, in radians
            */
   public double getDesiredPositionStatus()
   {
      return desired_position_status_;
   }

   /**
            * Torque limit set on the Dynamixel
            */
   public void setRawTorqueLimitStatus(double raw_torque_limit_status)
   {
      raw_torque_limit_status_ = raw_torque_limit_status;
   }
   /**
            * Torque limit set on the Dynamixel
            */
   public double getRawTorqueLimitStatus()
   {
      return raw_torque_limit_status_;
   }

   public void setTorqueOnStatus(boolean torque_on_status)
   {
      torque_on_status_ = torque_on_status;
   }
   public boolean getTorqueOnStatus()
   {
      return torque_on_status_;
   }

   /**
            * Rotation velocity of the Dynamixel, in rad/s
            * Positive = closing hand (CCW rotation)
            * Negative = opening hand (CW rotation)
            */
   public void setCurrentVelocity(double current_velocity)
   {
      current_velocity_ = current_velocity;
   }
   /**
            * Rotation velocity of the Dynamixel, in rad/s
            * Positive = closing hand (CCW rotation)
            * Negative = opening hand (CW rotation)
            */
   public double getCurrentVelocity()
   {
      return current_velocity_;
   }

   /**
            * Dynamixel's error codes
            * See: https://emanual.robotis.com/docs/en/dxl/protocol1/#error
            */
   public void setErrorCodes(int error_codes)
   {
      error_codes_ = error_codes;
   }
   /**
            * Dynamixel's error codes
            * See: https://emanual.robotis.com/docs/en/dxl/protocol1/#error
            */
   public int getErrorCodes()
   {
      return error_codes_;
   }

   /**
            * Realtime tick of the Dynamixel
            * If this value isn't changing, communication with the hand is broken
            */
   public void setRealtimeTick(int realtime_tick)
   {
      realtime_tick_ = realtime_tick;
   }
   /**
            * Realtime tick of the Dynamixel
            * If this value isn't changing, communication with the hand is broken
            */
   public int getRealtimeTick()
   {
      return realtime_tick_;
   }

   public void setIsCalibrated(boolean is_calibrated)
   {
      is_calibrated_ = is_calibrated;
   }
   public boolean getIsCalibrated()
   {
      return is_calibrated_;
   }

   public void setNeedsReset(boolean needs_reset)
   {
      needs_reset_ = needs_reset;
   }
   public boolean getNeedsReset()
   {
      return needs_reset_;
   }


   public static Supplier<SakeHandStatusMessagePubSubType> getPubSubType()
   {
      return SakeHandStatusMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return SakeHandStatusMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(SakeHandStatusMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.position_upper_limit_, other.position_upper_limit_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.position_lower_limit_, other.position_lower_limit_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.temperature_, other.temperature_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_position_, other.current_position_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.raw_current_torque_, other.raw_current_torque_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.desired_position_status_, other.desired_position_status_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.raw_torque_limit_status_, other.raw_torque_limit_status_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.torque_on_status_, other.torque_on_status_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_velocity_, other.current_velocity_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.error_codes_, other.error_codes_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.realtime_tick_, other.realtime_tick_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.is_calibrated_, other.is_calibrated_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.needs_reset_, other.needs_reset_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof SakeHandStatusMessage)) return false;

      SakeHandStatusMessage otherMyClass = (SakeHandStatusMessage) other;

      if(this.robot_side_ != otherMyClass.robot_side_) return false;

      if(this.position_upper_limit_ != otherMyClass.position_upper_limit_) return false;

      if(this.position_lower_limit_ != otherMyClass.position_lower_limit_) return false;

      if(this.temperature_ != otherMyClass.temperature_) return false;

      if(this.current_position_ != otherMyClass.current_position_) return false;

      if(this.raw_current_torque_ != otherMyClass.raw_current_torque_) return false;

      if(this.desired_position_status_ != otherMyClass.desired_position_status_) return false;

      if(this.raw_torque_limit_status_ != otherMyClass.raw_torque_limit_status_) return false;

      if(this.torque_on_status_ != otherMyClass.torque_on_status_) return false;

      if(this.current_velocity_ != otherMyClass.current_velocity_) return false;

      if(this.error_codes_ != otherMyClass.error_codes_) return false;

      if(this.realtime_tick_ != otherMyClass.realtime_tick_) return false;

      if(this.is_calibrated_ != otherMyClass.is_calibrated_) return false;

      if(this.needs_reset_ != otherMyClass.needs_reset_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("SakeHandStatusMessage {");
      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");
      builder.append("position_upper_limit=");
      builder.append(this.position_upper_limit_);      builder.append(", ");
      builder.append("position_lower_limit=");
      builder.append(this.position_lower_limit_);      builder.append(", ");
      builder.append("temperature=");
      builder.append(this.temperature_);      builder.append(", ");
      builder.append("current_position=");
      builder.append(this.current_position_);      builder.append(", ");
      builder.append("raw_current_torque=");
      builder.append(this.raw_current_torque_);      builder.append(", ");
      builder.append("desired_position_status=");
      builder.append(this.desired_position_status_);      builder.append(", ");
      builder.append("raw_torque_limit_status=");
      builder.append(this.raw_torque_limit_status_);      builder.append(", ");
      builder.append("torque_on_status=");
      builder.append(this.torque_on_status_);      builder.append(", ");
      builder.append("current_velocity=");
      builder.append(this.current_velocity_);      builder.append(", ");
      builder.append("error_codes=");
      builder.append(this.error_codes_);      builder.append(", ");
      builder.append("realtime_tick=");
      builder.append(this.realtime_tick_);      builder.append(", ");
      builder.append("is_calibrated=");
      builder.append(this.is_calibrated_);      builder.append(", ");
      builder.append("needs_reset=");
      builder.append(this.needs_reset_);
      builder.append("}");
      return builder.toString();
   }
}
