package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class HandWrenchActionDefinitionMessage extends Packet<HandWrenchActionDefinitionMessage> implements Settable<HandWrenchActionDefinitionMessage>, EpsilonComparable<HandWrenchActionDefinitionMessage>
{
   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.BehaviorActionDefinitionMessage action_definition_;
   /**
            * Specifies the side of the robot that this message refers to.
            */
   public byte robot_side_ = (byte) 255;
   /**
            * The trajectory duration
            */
   public double trajectory_duration_;
   /**
            * Amount of force to use
            */
   public double force_;

   public HandWrenchActionDefinitionMessage()
   {
      action_definition_ = new behavior_msgs.msg.dds.BehaviorActionDefinitionMessage();
   }

   public HandWrenchActionDefinitionMessage(HandWrenchActionDefinitionMessage other)
   {
      this();
      set(other);
   }

   public void set(HandWrenchActionDefinitionMessage other)
   {
      behavior_msgs.msg.dds.BehaviorActionDefinitionMessagePubSubType.staticCopy(other.action_definition_, action_definition_);
      robot_side_ = other.robot_side_;

      trajectory_duration_ = other.trajectory_duration_;

      force_ = other.force_;

   }


   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.BehaviorActionDefinitionMessage getActionDefinition()
   {
      return action_definition_;
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
            * The trajectory duration
            */
   public void setTrajectoryDuration(double trajectory_duration)
   {
      trajectory_duration_ = trajectory_duration;
   }
   /**
            * The trajectory duration
            */
   public double getTrajectoryDuration()
   {
      return trajectory_duration_;
   }

   /**
            * Amount of force to use
            */
   public void setForce(double force)
   {
      force_ = force;
   }
   /**
            * Amount of force to use
            */
   public double getForce()
   {
      return force_;
   }


   public static Supplier<HandWrenchActionDefinitionMessagePubSubType> getPubSubType()
   {
      return HandWrenchActionDefinitionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return HandWrenchActionDefinitionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(HandWrenchActionDefinitionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.action_definition_.epsilonEquals(other.action_definition_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.trajectory_duration_, other.trajectory_duration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.force_, other.force_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof HandWrenchActionDefinitionMessage)) return false;

      HandWrenchActionDefinitionMessage otherMyClass = (HandWrenchActionDefinitionMessage) other;

      if (!this.action_definition_.equals(otherMyClass.action_definition_)) return false;
      if(this.robot_side_ != otherMyClass.robot_side_) return false;

      if(this.trajectory_duration_ != otherMyClass.trajectory_duration_) return false;

      if(this.force_ != otherMyClass.force_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("HandWrenchActionDefinitionMessage {");
      builder.append("action_definition=");
      builder.append(this.action_definition_);      builder.append(", ");
      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");
      builder.append("trajectory_duration=");
      builder.append(this.trajectory_duration_);      builder.append(", ");
      builder.append("force=");
      builder.append(this.force_);
      builder.append("}");
      return builder.toString();
   }
}
