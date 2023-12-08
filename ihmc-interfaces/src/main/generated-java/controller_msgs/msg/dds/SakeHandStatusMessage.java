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
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   public byte robot_side_ = (byte) 255;
   /**
            * 0.0 open, 1.0 closed
            */
   public double normalized_desired_position_;
   /**
            * 0.0 min, 1.0 max
            */
   public double normalized_desired_torque_;
   /**
            * Celsius divided by 100
            */
   public double normalized_temperature_;
   /**
            * 0.0 open, 1.0 closed
            */
   public double normalized_measured_position_;
   /**
            * 0.0 min, 1.0 max
            */
   public double normalized_measured_torque_;
   /**
            * -1.0 = opening, 1.0 = closing. 0.0 = not moving
            */
   public double normalized_measured_velocity_;
   /**
            * error message as reported by Dynamixel. See: https://emanual.robotis.com/docs/en/dxl/protocol1/#error
            */
   public java.lang.StringBuilder error_message_;
   /**
            * tick value from Dynamixel. Increments from 0 to 32767, then repeats
            */
   public int realtime_tick_;

   public SakeHandStatusMessage()
   {
      error_message_ = new java.lang.StringBuilder(255);
   }

   public SakeHandStatusMessage(SakeHandStatusMessage other)
   {
      this();
      set(other);
   }

   public void set(SakeHandStatusMessage other)
   {
      sequence_id_ = other.sequence_id_;

      robot_side_ = other.robot_side_;

      normalized_desired_position_ = other.normalized_desired_position_;

      normalized_desired_torque_ = other.normalized_desired_torque_;

      normalized_temperature_ = other.normalized_temperature_;

      normalized_measured_position_ = other.normalized_measured_position_;

      normalized_measured_torque_ = other.normalized_measured_torque_;

      normalized_measured_velocity_ = other.normalized_measured_velocity_;

      error_message_.setLength(0);
      error_message_.append(other.error_message_);

      realtime_tick_ = other.realtime_tick_;

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

   public void setRobotSide(byte robot_side)
   {
      robot_side_ = robot_side;
   }
   public byte getRobotSide()
   {
      return robot_side_;
   }

   /**
            * 0.0 open, 1.0 closed
            */
   public void setNormalizedDesiredPosition(double normalized_desired_position)
   {
      normalized_desired_position_ = normalized_desired_position;
   }
   /**
            * 0.0 open, 1.0 closed
            */
   public double getNormalizedDesiredPosition()
   {
      return normalized_desired_position_;
   }

   /**
            * 0.0 min, 1.0 max
            */
   public void setNormalizedDesiredTorque(double normalized_desired_torque)
   {
      normalized_desired_torque_ = normalized_desired_torque;
   }
   /**
            * 0.0 min, 1.0 max
            */
   public double getNormalizedDesiredTorque()
   {
      return normalized_desired_torque_;
   }

   /**
            * Celsius divided by 100
            */
   public void setNormalizedTemperature(double normalized_temperature)
   {
      normalized_temperature_ = normalized_temperature;
   }
   /**
            * Celsius divided by 100
            */
   public double getNormalizedTemperature()
   {
      return normalized_temperature_;
   }

   /**
            * 0.0 open, 1.0 closed
            */
   public void setNormalizedMeasuredPosition(double normalized_measured_position)
   {
      normalized_measured_position_ = normalized_measured_position;
   }
   /**
            * 0.0 open, 1.0 closed
            */
   public double getNormalizedMeasuredPosition()
   {
      return normalized_measured_position_;
   }

   /**
            * 0.0 min, 1.0 max
            */
   public void setNormalizedMeasuredTorque(double normalized_measured_torque)
   {
      normalized_measured_torque_ = normalized_measured_torque;
   }
   /**
            * 0.0 min, 1.0 max
            */
   public double getNormalizedMeasuredTorque()
   {
      return normalized_measured_torque_;
   }

   /**
            * -1.0 = opening, 1.0 = closing. 0.0 = not moving
            */
   public void setNormalizedMeasuredVelocity(double normalized_measured_velocity)
   {
      normalized_measured_velocity_ = normalized_measured_velocity;
   }
   /**
            * -1.0 = opening, 1.0 = closing. 0.0 = not moving
            */
   public double getNormalizedMeasuredVelocity()
   {
      return normalized_measured_velocity_;
   }

   /**
            * error message as reported by Dynamixel. See: https://emanual.robotis.com/docs/en/dxl/protocol1/#error
            */
   public void setErrorMessage(java.lang.String error_message)
   {
      error_message_.setLength(0);
      error_message_.append(error_message);
   }

   /**
            * error message as reported by Dynamixel. See: https://emanual.robotis.com/docs/en/dxl/protocol1/#error
            */
   public java.lang.String getErrorMessageAsString()
   {
      return getErrorMessage().toString();
   }
   /**
            * error message as reported by Dynamixel. See: https://emanual.robotis.com/docs/en/dxl/protocol1/#error
            */
   public java.lang.StringBuilder getErrorMessage()
   {
      return error_message_;
   }

   /**
            * tick value from Dynamixel. Increments from 0 to 32767, then repeats
            */
   public void setRealtimeTick(int realtime_tick)
   {
      realtime_tick_ = realtime_tick;
   }
   /**
            * tick value from Dynamixel. Increments from 0 to 32767, then repeats
            */
   public int getRealtimeTick()
   {
      return realtime_tick_;
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

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.normalized_desired_position_, other.normalized_desired_position_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.normalized_desired_torque_, other.normalized_desired_torque_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.normalized_temperature_, other.normalized_temperature_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.normalized_measured_position_, other.normalized_measured_position_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.normalized_measured_torque_, other.normalized_measured_torque_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.normalized_measured_velocity_, other.normalized_measured_velocity_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.error_message_, other.error_message_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.realtime_tick_, other.realtime_tick_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof SakeHandStatusMessage)) return false;

      SakeHandStatusMessage otherMyClass = (SakeHandStatusMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.robot_side_ != otherMyClass.robot_side_) return false;

      if(this.normalized_desired_position_ != otherMyClass.normalized_desired_position_) return false;

      if(this.normalized_desired_torque_ != otherMyClass.normalized_desired_torque_) return false;

      if(this.normalized_temperature_ != otherMyClass.normalized_temperature_) return false;

      if(this.normalized_measured_position_ != otherMyClass.normalized_measured_position_) return false;

      if(this.normalized_measured_torque_ != otherMyClass.normalized_measured_torque_) return false;

      if(this.normalized_measured_velocity_ != otherMyClass.normalized_measured_velocity_) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.error_message_, otherMyClass.error_message_)) return false;

      if(this.realtime_tick_ != otherMyClass.realtime_tick_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("SakeHandStatusMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");
      builder.append("normalized_desired_position=");
      builder.append(this.normalized_desired_position_);      builder.append(", ");
      builder.append("normalized_desired_torque=");
      builder.append(this.normalized_desired_torque_);      builder.append(", ");
      builder.append("normalized_temperature=");
      builder.append(this.normalized_temperature_);      builder.append(", ");
      builder.append("normalized_measured_position=");
      builder.append(this.normalized_measured_position_);      builder.append(", ");
      builder.append("normalized_measured_torque=");
      builder.append(this.normalized_measured_torque_);      builder.append(", ");
      builder.append("normalized_measured_velocity=");
      builder.append(this.normalized_measured_velocity_);      builder.append(", ");
      builder.append("error_message=");
      builder.append(this.error_message_);      builder.append(", ");
      builder.append("realtime_tick=");
      builder.append(this.realtime_tick_);
      builder.append("}");
      return builder.toString();
   }
}
