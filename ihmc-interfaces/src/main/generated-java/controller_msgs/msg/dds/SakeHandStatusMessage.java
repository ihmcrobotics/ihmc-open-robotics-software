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
   public double temperature_;
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

   public void setTemperature(double temperature)
   {
      temperature_ = temperature;
   }
   public double getTemperature()
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
      builder.append("is_calibrated=");
      builder.append(this.is_calibrated_);      builder.append(", ");
      builder.append("needs_reset=");
      builder.append(this.needs_reset_);
      builder.append("}");
      return builder.toString();
   }
}
