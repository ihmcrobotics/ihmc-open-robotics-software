package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class HandConfigurationActionMessage extends Packet<HandConfigurationActionMessage> implements Settable<HandConfigurationActionMessage>, EpsilonComparable<HandConfigurationActionMessage>
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
            * The grip type
            */
   public long grip_;
   /**
            * Whether the next action can be executed at the same time of this one
            */
   public boolean execute_with_next_action_;

   public HandConfigurationActionMessage()
   {
      action_information_ = new behavior_msgs.msg.dds.ActionInformationMessage();
   }

   public HandConfigurationActionMessage(HandConfigurationActionMessage other)
   {
      this();
      set(other);
   }

   public void set(HandConfigurationActionMessage other)
   {
      behavior_msgs.msg.dds.ActionInformationMessagePubSubType.staticCopy(other.action_information_, action_information_);
      robot_side_ = other.robot_side_;

      grip_ = other.grip_;

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
            * The grip type
            */
   public void setGrip(long grip)
   {
      grip_ = grip;
   }
   /**
            * The grip type
            */
   public long getGrip()
   {
      return grip_;
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


   public static Supplier<HandConfigurationActionMessagePubSubType> getPubSubType()
   {
      return HandConfigurationActionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return HandConfigurationActionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(HandConfigurationActionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.action_information_.epsilonEquals(other.action_information_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.grip_, other.grip_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.execute_with_next_action_, other.execute_with_next_action_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof HandConfigurationActionMessage)) return false;

      HandConfigurationActionMessage otherMyClass = (HandConfigurationActionMessage) other;

      if (!this.action_information_.equals(otherMyClass.action_information_)) return false;
      if(this.robot_side_ != otherMyClass.robot_side_) return false;

      if(this.grip_ != otherMyClass.grip_) return false;

      if(this.execute_with_next_action_ != otherMyClass.execute_with_next_action_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("HandConfigurationActionMessage {");
      builder.append("action_information=");
      builder.append(this.action_information_);      builder.append(", ");
      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");
      builder.append("grip=");
      builder.append(this.grip_);      builder.append(", ");
      builder.append("execute_with_next_action=");
      builder.append(this.execute_with_next_action_);
      builder.append("}");
      return builder.toString();
   }
}
