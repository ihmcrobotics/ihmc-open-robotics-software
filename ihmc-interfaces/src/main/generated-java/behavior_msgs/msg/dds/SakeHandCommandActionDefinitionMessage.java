package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class SakeHandCommandActionDefinitionMessage extends Packet<SakeHandCommandActionDefinitionMessage> implements Settable<SakeHandCommandActionDefinitionMessage>, EpsilonComparable<SakeHandCommandActionDefinitionMessage>
{
   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.ActionNodeDefinitionMessage definition_;
   /**
            * Specifies the side of the robot that this message refers to.
            */
   public byte robot_side_ = (byte) 255;
   /**
            * Desired angle between the fingers from 0 to 210 degrees (specified in radians)
            */
   public double hand_open_angle_;
   /**
            * Torque limit specified as fingertip grip force. Valid up to 29 Newtons. Safe value is 8.7 N.
            */
   public double fingertip_grip_force_limit_;

   public SakeHandCommandActionDefinitionMessage()
   {
      definition_ = new behavior_msgs.msg.dds.ActionNodeDefinitionMessage();
   }

   public SakeHandCommandActionDefinitionMessage(SakeHandCommandActionDefinitionMessage other)
   {
      this();
      set(other);
   }

   public void set(SakeHandCommandActionDefinitionMessage other)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      robot_side_ = other.robot_side_;

      hand_open_angle_ = other.hand_open_angle_;

      fingertip_grip_force_limit_ = other.fingertip_grip_force_limit_;

   }


   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.ActionNodeDefinitionMessage getDefinition()
   {
      return definition_;
   }

   /**
            * Specifies the side of the robot that this message refers to.
            */
   public void setRobotSide(byte robot_side)
   {
      robot_side_ = robot_side;
   }
   /**
            * Specifies the side of the robot that this message refers to.
            */
   public byte getRobotSide()
   {
      return robot_side_;
   }

   /**
            * Desired angle between the fingers from 0 to 210 degrees (specified in radians)
            */
   public void setHandOpenAngle(double hand_open_angle)
   {
      hand_open_angle_ = hand_open_angle;
   }
   /**
            * Desired angle between the fingers from 0 to 210 degrees (specified in radians)
            */
   public double getHandOpenAngle()
   {
      return hand_open_angle_;
   }

   /**
            * Torque limit specified as fingertip grip force. Valid up to 29 Newtons. Safe value is 8.7 N.
            */
   public void setFingertipGripForceLimit(double fingertip_grip_force_limit)
   {
      fingertip_grip_force_limit_ = fingertip_grip_force_limit;
   }
   /**
            * Torque limit specified as fingertip grip force. Valid up to 29 Newtons. Safe value is 8.7 N.
            */
   public double getFingertipGripForceLimit()
   {
      return fingertip_grip_force_limit_;
   }


   public static Supplier<SakeHandCommandActionDefinitionMessagePubSubType> getPubSubType()
   {
      return SakeHandCommandActionDefinitionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return SakeHandCommandActionDefinitionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(SakeHandCommandActionDefinitionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.hand_open_angle_, other.hand_open_angle_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.fingertip_grip_force_limit_, other.fingertip_grip_force_limit_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof SakeHandCommandActionDefinitionMessage)) return false;

      SakeHandCommandActionDefinitionMessage otherMyClass = (SakeHandCommandActionDefinitionMessage) other;

      if (!this.definition_.equals(otherMyClass.definition_)) return false;
      if(this.robot_side_ != otherMyClass.robot_side_) return false;

      if(this.hand_open_angle_ != otherMyClass.hand_open_angle_) return false;

      if(this.fingertip_grip_force_limit_ != otherMyClass.fingertip_grip_force_limit_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("SakeHandCommandActionDefinitionMessage {");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");
      builder.append("hand_open_angle=");
      builder.append(this.hand_open_angle_);      builder.append(", ");
      builder.append("fingertip_grip_force_limit=");
      builder.append(this.fingertip_grip_force_limit_);
      builder.append("}");
      return builder.toString();
   }
}
