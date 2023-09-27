package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class HandWrenchActionDescriptionMessage extends Packet<HandWrenchActionDescriptionMessage> implements Settable<HandWrenchActionDescriptionMessage>, EpsilonComparable<HandWrenchActionDescriptionMessage>
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
            * The trajectory duration
            */
   public double trajectory_duration_;
   /**
            * Amount of force to use
            */
   public double force_;

   public HandWrenchActionDescriptionMessage()
   {
      action_information_ = new behavior_msgs.msg.dds.ActionInformationMessage();
   }

   public HandWrenchActionDescriptionMessage(HandWrenchActionDescriptionMessage other)
   {
      this();
      set(other);
   }

   public void set(HandWrenchActionDescriptionMessage other)
   {
      behavior_msgs.msg.dds.ActionInformationMessagePubSubType.staticCopy(other.action_information_, action_information_);
      robot_side_ = other.robot_side_;

      trajectory_duration_ = other.trajectory_duration_;

      force_ = other.force_;

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


   public static Supplier<HandWrenchActionDescriptionMessagePubSubType> getPubSubType()
   {
      return HandWrenchActionDescriptionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return HandWrenchActionDescriptionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(HandWrenchActionDescriptionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.action_information_.epsilonEquals(other.action_information_, epsilon)) return false;
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
      if(!(other instanceof HandWrenchActionDescriptionMessage)) return false;

      HandWrenchActionDescriptionMessage otherMyClass = (HandWrenchActionDescriptionMessage) other;

      if (!this.action_information_.equals(otherMyClass.action_information_)) return false;
      if(this.robot_side_ != otherMyClass.robot_side_) return false;

      if(this.trajectory_duration_ != otherMyClass.trajectory_duration_) return false;

      if(this.force_ != otherMyClass.force_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("HandWrenchActionDescriptionMessage {");
      builder.append("action_information=");
      builder.append(this.action_information_);      builder.append(", ");
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
