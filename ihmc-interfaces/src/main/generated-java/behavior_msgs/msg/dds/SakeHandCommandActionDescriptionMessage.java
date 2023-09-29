package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class SakeHandCommandActionDescriptionMessage extends Packet<SakeHandCommandActionDescriptionMessage> implements Settable<SakeHandCommandActionDescriptionMessage>, EpsilonComparable<SakeHandCommandActionDescriptionMessage>
{
   /**
            * Used for syncing action sequences
            */
   public behavior_msgs.msg.dds.ActionInformationMessage action_information_;
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
   /**
            * Whether the next action can be executed at the same time of this one
            */
   public boolean execute_with_next_action_;

   public SakeHandCommandActionDescriptionMessage()
   {
      action_information_ = new behavior_msgs.msg.dds.ActionInformationMessage();
   }

   public SakeHandCommandActionDescriptionMessage(SakeHandCommandActionDescriptionMessage other)
   {
      this();
      set(other);
   }

   public void set(SakeHandCommandActionDescriptionMessage other)
   {
      behavior_msgs.msg.dds.ActionInformationMessagePubSubType.staticCopy(other.action_information_, action_information_);
      robot_side_ = other.robot_side_;

      configuration_ = other.configuration_;

      position_ratio_ = other.position_ratio_;

      torque_ratio_ = other.torque_ratio_;

      execute_with_next_action_ = other.execute_with_next_action_;

   }


   /**
            * Used for syncing action sequences
            */
   public behavior_msgs.msg.dds.ActionInformationMessage getActionInformation()
   {
      return action_information_;
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

   /**
            * Whether the next action can be executed at the same time of this one
            */
   public void setExecuteWithNextAction(boolean execute_with_next_action)
   {
      execute_with_next_action_ = execute_with_next_action;
   }
   /**
            * Whether the next action can be executed at the same time of this one
            */
   public boolean getExecuteWithNextAction()
   {
      return execute_with_next_action_;
   }


   public static Supplier<SakeHandCommandActionDescriptionMessagePubSubType> getPubSubType()
   {
      return SakeHandCommandActionDescriptionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return SakeHandCommandActionDescriptionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(SakeHandCommandActionDescriptionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.action_information_.epsilonEquals(other.action_information_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.configuration_, other.configuration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.position_ratio_, other.position_ratio_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.torque_ratio_, other.torque_ratio_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.execute_with_next_action_, other.execute_with_next_action_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof SakeHandCommandActionDescriptionMessage)) return false;

      SakeHandCommandActionDescriptionMessage otherMyClass = (SakeHandCommandActionDescriptionMessage) other;

      if (!this.action_information_.equals(otherMyClass.action_information_)) return false;
      if(this.robot_side_ != otherMyClass.robot_side_) return false;

      if(this.configuration_ != otherMyClass.configuration_) return false;

      if(this.position_ratio_ != otherMyClass.position_ratio_) return false;

      if(this.torque_ratio_ != otherMyClass.torque_ratio_) return false;

      if(this.execute_with_next_action_ != otherMyClass.execute_with_next_action_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("SakeHandCommandActionDescriptionMessage {");
      builder.append("action_information=");
      builder.append(this.action_information_);      builder.append(", ");
      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");
      builder.append("configuration=");
      builder.append(this.configuration_);      builder.append(", ");
      builder.append("position_ratio=");
      builder.append(this.position_ratio_);      builder.append(", ");
      builder.append("torque_ratio=");
      builder.append(this.torque_ratio_);      builder.append(", ");
      builder.append("execute_with_next_action=");
      builder.append(this.execute_with_next_action_);
      builder.append("}");
      return builder.toString();
   }
}
