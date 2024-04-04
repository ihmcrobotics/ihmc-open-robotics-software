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
            * Temperature of the Dynamixel in Celsius
            */
   public int temperature_;
   /**
            * The current dynamixel position, normalized to the gripper range of motion
            * 0.0 (fingers touching) -> 1.0 (open 210 degrees between fingers)
            */
   public double normalized_current_position_;
   /**
            * The current dynamixel torque
            * 0.0: dynamixel will not apply any force and will not achieve desired position
            * 0.3: A reasonable normal value
            * 1.0: dynamixel max torque which will quickly overheat the motor
            */
   public double normalized_current_torque_;
   public double normalized_desired_position_;
   public double normalized_torque_limit_;
   public boolean torque_on_status_;
   /**
            * RPM of the Dynamixel
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

      temperature_ = other.temperature_;

      normalized_current_position_ = other.normalized_current_position_;

      normalized_current_torque_ = other.normalized_current_torque_;

      normalized_desired_position_ = other.normalized_desired_position_;

      normalized_torque_limit_ = other.normalized_torque_limit_;

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
            * The current dynamixel position, normalized to the gripper range of motion
            * 0.0 (fingers touching) -> 1.0 (open 210 degrees between fingers)
            */
   public void setNormalizedCurrentPosition(double normalized_current_position)
   {
      normalized_current_position_ = normalized_current_position;
   }
   /**
            * The current dynamixel position, normalized to the gripper range of motion
            * 0.0 (fingers touching) -> 1.0 (open 210 degrees between fingers)
            */
   public double getNormalizedCurrentPosition()
   {
      return normalized_current_position_;
   }

   /**
            * The current dynamixel torque
            * 0.0: dynamixel will not apply any force and will not achieve desired position
            * 0.3: A reasonable normal value
            * 1.0: dynamixel max torque which will quickly overheat the motor
            */
   public void setNormalizedCurrentTorque(double normalized_current_torque)
   {
      normalized_current_torque_ = normalized_current_torque;
   }
   /**
            * The current dynamixel torque
            * 0.0: dynamixel will not apply any force and will not achieve desired position
            * 0.3: A reasonable normal value
            * 1.0: dynamixel max torque which will quickly overheat the motor
            */
   public double getNormalizedCurrentTorque()
   {
      return normalized_current_torque_;
   }

   public void setNormalizedDesiredPosition(double normalized_desired_position)
   {
      normalized_desired_position_ = normalized_desired_position;
   }
   public double getNormalizedDesiredPosition()
   {
      return normalized_desired_position_;
   }

   public void setNormalizedTorqueLimit(double normalized_torque_limit)
   {
      normalized_torque_limit_ = normalized_torque_limit;
   }
   public double getNormalizedTorqueLimit()
   {
      return normalized_torque_limit_;
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
            * RPM of the Dynamixel
            * Positive = closing hand (CCW rotation)
            * Negative = opening hand (CW rotation)
            */
   public void setCurrentVelocity(double current_velocity)
   {
      current_velocity_ = current_velocity;
   }
   /**
            * RPM of the Dynamixel
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

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.temperature_, other.temperature_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.normalized_current_position_, other.normalized_current_position_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.normalized_current_torque_, other.normalized_current_torque_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.normalized_desired_position_, other.normalized_desired_position_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.normalized_torque_limit_, other.normalized_torque_limit_, epsilon)) return false;

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

      if(this.temperature_ != otherMyClass.temperature_) return false;

      if(this.normalized_current_position_ != otherMyClass.normalized_current_position_) return false;

      if(this.normalized_current_torque_ != otherMyClass.normalized_current_torque_) return false;

      if(this.normalized_desired_position_ != otherMyClass.normalized_desired_position_) return false;

      if(this.normalized_torque_limit_ != otherMyClass.normalized_torque_limit_) return false;

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
      builder.append("temperature=");
      builder.append(this.temperature_);      builder.append(", ");
      builder.append("normalized_current_position=");
      builder.append(this.normalized_current_position_);      builder.append(", ");
      builder.append("normalized_current_torque=");
      builder.append(this.normalized_current_torque_);      builder.append(", ");
      builder.append("normalized_desired_position=");
      builder.append(this.normalized_desired_position_);      builder.append(", ");
      builder.append("normalized_torque_limit=");
      builder.append(this.normalized_torque_limit_);      builder.append(", ");
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
