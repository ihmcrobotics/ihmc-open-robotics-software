package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class EtherSnacksSakeHandStatusMessage extends Packet<EtherSnacksSakeHandStatusMessage> implements Settable<EtherSnacksSakeHandStatusMessage>, EpsilonComparable<EtherSnacksSakeHandStatusMessage>
{
   public static final byte ROBOT_SIDE_LEFT = (byte) 0;
   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;
   public byte robot_side_ = (byte) 255;
   public double desired_position_;
   public boolean torque_on_;
   public double torque_limit_;
   public double measured_position_;
   public double measured_torque_;
   public int measured_temperature_;
   public double measured_velocity_;
   public int error_codes_;
   public int realtime_tick_;

   public EtherSnacksSakeHandStatusMessage()
   {
   }

   public EtherSnacksSakeHandStatusMessage(EtherSnacksSakeHandStatusMessage other)
   {
      this();
      set(other);
   }

   public void set(EtherSnacksSakeHandStatusMessage other)
   {
      robot_side_ = other.robot_side_;

      desired_position_ = other.desired_position_;

      torque_on_ = other.torque_on_;

      torque_limit_ = other.torque_limit_;

      measured_position_ = other.measured_position_;

      measured_torque_ = other.measured_torque_;

      measured_temperature_ = other.measured_temperature_;

      measured_velocity_ = other.measured_velocity_;

      error_codes_ = other.error_codes_;

      realtime_tick_ = other.realtime_tick_;

   }

   public void setRobotSide(byte robot_side)
   {
      robot_side_ = robot_side;
   }
   public byte getRobotSide()
   {
      return robot_side_;
   }

   public void setDesiredPosition(double desired_position)
   {
      desired_position_ = desired_position;
   }
   public double getDesiredPosition()
   {
      return desired_position_;
   }

   public void setTorqueOn(boolean torque_on)
   {
      torque_on_ = torque_on;
   }
   public boolean getTorqueOn()
   {
      return torque_on_;
   }

   public void setTorqueLimit(double torque_limit)
   {
      torque_limit_ = torque_limit;
   }
   public double getTorqueLimit()
   {
      return torque_limit_;
   }

   public void setMeasuredPosition(double measured_position)
   {
      measured_position_ = measured_position;
   }
   public double getMeasuredPosition()
   {
      return measured_position_;
   }

   public void setMeasuredTorque(double measured_torque)
   {
      measured_torque_ = measured_torque;
   }
   public double getMeasuredTorque()
   {
      return measured_torque_;
   }

   public void setMeasuredTemperature(int measured_temperature)
   {
      measured_temperature_ = measured_temperature;
   }
   public int getMeasuredTemperature()
   {
      return measured_temperature_;
   }

   public void setMeasuredVelocity(double measured_velocity)
   {
      measured_velocity_ = measured_velocity;
   }
   public double getMeasuredVelocity()
   {
      return measured_velocity_;
   }

   public void setErrorCodes(int error_codes)
   {
      error_codes_ = error_codes;
   }
   public int getErrorCodes()
   {
      return error_codes_;
   }

   public void setRealtimeTick(int realtime_tick)
   {
      realtime_tick_ = realtime_tick;
   }
   public int getRealtimeTick()
   {
      return realtime_tick_;
   }


   public static Supplier<EtherSnacksSakeHandStatusMessagePubSubType> getPubSubType()
   {
      return EtherSnacksSakeHandStatusMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return EtherSnacksSakeHandStatusMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(EtherSnacksSakeHandStatusMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.desired_position_, other.desired_position_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.torque_on_, other.torque_on_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.torque_limit_, other.torque_limit_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.measured_position_, other.measured_position_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.measured_torque_, other.measured_torque_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.measured_temperature_, other.measured_temperature_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.measured_velocity_, other.measured_velocity_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.error_codes_, other.error_codes_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.realtime_tick_, other.realtime_tick_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof EtherSnacksSakeHandStatusMessage)) return false;

      EtherSnacksSakeHandStatusMessage otherMyClass = (EtherSnacksSakeHandStatusMessage) other;

      if(this.robot_side_ != otherMyClass.robot_side_) return false;

      if(this.desired_position_ != otherMyClass.desired_position_) return false;

      if(this.torque_on_ != otherMyClass.torque_on_) return false;

      if(this.torque_limit_ != otherMyClass.torque_limit_) return false;

      if(this.measured_position_ != otherMyClass.measured_position_) return false;

      if(this.measured_torque_ != otherMyClass.measured_torque_) return false;

      if(this.measured_temperature_ != otherMyClass.measured_temperature_) return false;

      if(this.measured_velocity_ != otherMyClass.measured_velocity_) return false;

      if(this.error_codes_ != otherMyClass.error_codes_) return false;

      if(this.realtime_tick_ != otherMyClass.realtime_tick_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("EtherSnacksSakeHandStatusMessage {");
      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");
      builder.append("desired_position=");
      builder.append(this.desired_position_);      builder.append(", ");
      builder.append("torque_on=");
      builder.append(this.torque_on_);      builder.append(", ");
      builder.append("torque_limit=");
      builder.append(this.torque_limit_);      builder.append(", ");
      builder.append("measured_position=");
      builder.append(this.measured_position_);      builder.append(", ");
      builder.append("measured_torque=");
      builder.append(this.measured_torque_);      builder.append(", ");
      builder.append("measured_temperature=");
      builder.append(this.measured_temperature_);      builder.append(", ");
      builder.append("measured_velocity=");
      builder.append(this.measured_velocity_);      builder.append(", ");
      builder.append("error_codes=");
      builder.append(this.error_codes_);      builder.append(", ");
      builder.append("realtime_tick=");
      builder.append(this.realtime_tick_);
      builder.append("}");
      return builder.toString();
   }
}
