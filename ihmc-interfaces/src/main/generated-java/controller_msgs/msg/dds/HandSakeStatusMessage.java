package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Message used to report the current joint angles for the fingers of the sake gripper.
       */
public class HandSakeStatusMessage extends Packet<HandSakeStatusMessage> implements Settable<HandSakeStatusMessage>, EpsilonComparable<HandSakeStatusMessage>
{
   public static final byte ROBOT_SIDE_LEFT = (byte) 0;
   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   public byte robot_side_ = (byte) 255;
   public double temperature_;
   /**
            * 0.0 min, 1.0 max
            */
   public double torque_ratio_;
   /**
            * 0.0 closed, 1.0 open
            */
   public double postion_ratio_;
   public boolean is_in_error_state_;

   public HandSakeStatusMessage()
   {
   }

   public HandSakeStatusMessage(HandSakeStatusMessage other)
   {
      this();
      set(other);
   }

   public void set(HandSakeStatusMessage other)
   {
      sequence_id_ = other.sequence_id_;

      robot_side_ = other.robot_side_;

      temperature_ = other.temperature_;

      torque_ratio_ = other.torque_ratio_;

      postion_ratio_ = other.postion_ratio_;

      is_in_error_state_ = other.is_in_error_state_;

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

   public void setTemperature(double temperature)
   {
      temperature_ = temperature;
   }
   public double getTemperature()
   {
      return temperature_;
   }

   /**
            * 0.0 min, 1.0 max
            */
   public void setTorqueRatio(double torque_ratio)
   {
      torque_ratio_ = torque_ratio;
   }
   /**
            * 0.0 min, 1.0 max
            */
   public double getTorqueRatio()
   {
      return torque_ratio_;
   }

   /**
            * 0.0 closed, 1.0 open
            */
   public void setPostionRatio(double postion_ratio)
   {
      postion_ratio_ = postion_ratio;
   }
   /**
            * 0.0 closed, 1.0 open
            */
   public double getPostionRatio()
   {
      return postion_ratio_;
   }

   public void setIsInErrorState(boolean is_in_error_state)
   {
      is_in_error_state_ = is_in_error_state;
   }
   public boolean getIsInErrorState()
   {
      return is_in_error_state_;
   }


   public static Supplier<HandSakeStatusMessagePubSubType> getPubSubType()
   {
      return HandSakeStatusMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return HandSakeStatusMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(HandSakeStatusMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.temperature_, other.temperature_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.torque_ratio_, other.torque_ratio_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.postion_ratio_, other.postion_ratio_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.is_in_error_state_, other.is_in_error_state_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof HandSakeStatusMessage)) return false;

      HandSakeStatusMessage otherMyClass = (HandSakeStatusMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.robot_side_ != otherMyClass.robot_side_) return false;

      if(this.temperature_ != otherMyClass.temperature_) return false;

      if(this.torque_ratio_ != otherMyClass.torque_ratio_) return false;

      if(this.postion_ratio_ != otherMyClass.postion_ratio_) return false;

      if(this.is_in_error_state_ != otherMyClass.is_in_error_state_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("HandSakeStatusMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");
      builder.append("temperature=");
      builder.append(this.temperature_);      builder.append(", ");
      builder.append("torque_ratio=");
      builder.append(this.torque_ratio_);      builder.append(", ");
      builder.append("postion_ratio=");
      builder.append(this.postion_ratio_);      builder.append(", ");
      builder.append("is_in_error_state=");
      builder.append(this.is_in_error_state_);
      builder.append("}");
      return builder.toString();
   }
}
