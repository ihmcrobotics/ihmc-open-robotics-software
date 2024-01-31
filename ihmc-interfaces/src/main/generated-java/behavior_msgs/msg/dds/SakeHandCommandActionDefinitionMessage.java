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
            * Hand configuration type
            */
   public long configuration_;
   /**
            * Goal position as ratio from 0.0 (closed) to 1.0 (open)
            */
   public double position_ratio_;
   /**
            * Goal torque as ratio from 0.0 (closed) to 1.0 (open)
            */
   public double torque_ratio_;

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

      configuration_ = other.configuration_;

      position_ratio_ = other.position_ratio_;

      torque_ratio_ = other.torque_ratio_;

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
            * Hand configuration type
            */
   public void setConfiguration(long configuration)
   {
      configuration_ = configuration;
   }
   /**
            * Hand configuration type
            */
   public long getConfiguration()
   {
      return configuration_;
   }

   /**
            * Goal position as ratio from 0.0 (closed) to 1.0 (open)
            */
   public void setPositionRatio(double position_ratio)
   {
      position_ratio_ = position_ratio;
   }
   /**
            * Goal position as ratio from 0.0 (closed) to 1.0 (open)
            */
   public double getPositionRatio()
   {
      return position_ratio_;
   }

   /**
            * Goal torque as ratio from 0.0 (closed) to 1.0 (open)
            */
   public void setTorqueRatio(double torque_ratio)
   {
      torque_ratio_ = torque_ratio;
   }
   /**
            * Goal torque as ratio from 0.0 (closed) to 1.0 (open)
            */
   public double getTorqueRatio()
   {
      return torque_ratio_;
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

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.configuration_, other.configuration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.position_ratio_, other.position_ratio_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.torque_ratio_, other.torque_ratio_, epsilon)) return false;


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

      if(this.configuration_ != otherMyClass.configuration_) return false;

      if(this.position_ratio_ != otherMyClass.position_ratio_) return false;

      if(this.torque_ratio_ != otherMyClass.torque_ratio_) return false;


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
      builder.append("configuration=");
      builder.append(this.configuration_);      builder.append(", ");
      builder.append("position_ratio=");
      builder.append(this.position_ratio_);      builder.append(", ");
      builder.append("torque_ratio=");
      builder.append(this.torque_ratio_);
      builder.append("}");
      return builder.toString();
   }
}
