package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class EtherSnacksSakeHandCommandMessage extends Packet<EtherSnacksSakeHandCommandMessage> implements Settable<EtherSnacksSakeHandCommandMessage>, EpsilonComparable<EtherSnacksSakeHandCommandMessage>
{
   public static final byte ROBOT_SIDE_LEFT = (byte) 0;
   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;
   public byte robot_side_ = (byte) 255;
   public double desired_position_;
   public double torque_limit_;
   public boolean torque_on_;

   public EtherSnacksSakeHandCommandMessage()
   {
   }

   public EtherSnacksSakeHandCommandMessage(EtherSnacksSakeHandCommandMessage other)
   {
      this();
      set(other);
   }

   public void set(EtherSnacksSakeHandCommandMessage other)
   {
      robot_side_ = other.robot_side_;

      desired_position_ = other.desired_position_;

      torque_limit_ = other.torque_limit_;

      torque_on_ = other.torque_on_;

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

   public void setTorqueLimit(double torque_limit)
   {
      torque_limit_ = torque_limit;
   }
   public double getTorqueLimit()
   {
      return torque_limit_;
   }

   public void setTorqueOn(boolean torque_on)
   {
      torque_on_ = torque_on;
   }
   public boolean getTorqueOn()
   {
      return torque_on_;
   }


   public static Supplier<EtherSnacksSakeHandCommandMessagePubSubType> getPubSubType()
   {
      return EtherSnacksSakeHandCommandMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return EtherSnacksSakeHandCommandMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(EtherSnacksSakeHandCommandMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.desired_position_, other.desired_position_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.torque_limit_, other.torque_limit_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.torque_on_, other.torque_on_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof EtherSnacksSakeHandCommandMessage)) return false;

      EtherSnacksSakeHandCommandMessage otherMyClass = (EtherSnacksSakeHandCommandMessage) other;

      if(this.robot_side_ != otherMyClass.robot_side_) return false;

      if(this.desired_position_ != otherMyClass.desired_position_) return false;

      if(this.torque_limit_ != otherMyClass.torque_limit_) return false;

      if(this.torque_on_ != otherMyClass.torque_on_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("EtherSnacksSakeHandCommandMessage {");
      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");
      builder.append("desired_position=");
      builder.append(this.desired_position_);      builder.append(", ");
      builder.append("torque_limit=");
      builder.append(this.torque_limit_);      builder.append(", ");
      builder.append("torque_on=");
      builder.append(this.torque_on_);
      builder.append("}");
      return builder.toString();
   }
}
