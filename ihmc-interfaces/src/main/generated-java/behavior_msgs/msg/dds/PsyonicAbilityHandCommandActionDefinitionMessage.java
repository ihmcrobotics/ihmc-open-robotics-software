package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class PsyonicAbilityHandCommandActionDefinitionMessage extends Packet<PsyonicAbilityHandCommandActionDefinitionMessage> implements Settable<PsyonicAbilityHandCommandActionDefinitionMessage>, EpsilonComparable<PsyonicAbilityHandCommandActionDefinitionMessage>
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
            * The string value of the legacy grip type
            * https://github.com/ihmcrobotics/psyonic-ability-hand-java/blob/main/src/main/java/us/ihmc/abilityhand/AbilityHandLegacyGripCommand.java#L63
            */
   public java.lang.StringBuilder grip_type_;
   /**
            * The string value of the legacy grip speed
            * https://github.com/ihmcrobotics/psyonic-ability-hand-java/blob/main/src/main/java/us/ihmc/abilityhand/AbilityHandLegacyGripCommand.java#L58
            */
   public java.lang.StringBuilder grip_speed_;

   public PsyonicAbilityHandCommandActionDefinitionMessage()
   {
      definition_ = new behavior_msgs.msg.dds.ActionNodeDefinitionMessage();
      grip_type_ = new java.lang.StringBuilder(255);
      grip_speed_ = new java.lang.StringBuilder(255);
   }

   public PsyonicAbilityHandCommandActionDefinitionMessage(PsyonicAbilityHandCommandActionDefinitionMessage other)
   {
      this();
      set(other);
   }

   public void set(PsyonicAbilityHandCommandActionDefinitionMessage other)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      robot_side_ = other.robot_side_;

      grip_type_.setLength(0);
      grip_type_.append(other.grip_type_);

      grip_speed_.setLength(0);
      grip_speed_.append(other.grip_speed_);

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
            * The string value of the legacy grip type
            * https://github.com/ihmcrobotics/psyonic-ability-hand-java/blob/main/src/main/java/us/ihmc/abilityhand/AbilityHandLegacyGripCommand.java#L63
            */
   public void setGripType(java.lang.String grip_type)
   {
      grip_type_.setLength(0);
      grip_type_.append(grip_type);
   }

   /**
            * The string value of the legacy grip type
            * https://github.com/ihmcrobotics/psyonic-ability-hand-java/blob/main/src/main/java/us/ihmc/abilityhand/AbilityHandLegacyGripCommand.java#L63
            */
   public java.lang.String getGripTypeAsString()
   {
      return getGripType().toString();
   }
   /**
            * The string value of the legacy grip type
            * https://github.com/ihmcrobotics/psyonic-ability-hand-java/blob/main/src/main/java/us/ihmc/abilityhand/AbilityHandLegacyGripCommand.java#L63
            */
   public java.lang.StringBuilder getGripType()
   {
      return grip_type_;
   }

   /**
            * The string value of the legacy grip speed
            * https://github.com/ihmcrobotics/psyonic-ability-hand-java/blob/main/src/main/java/us/ihmc/abilityhand/AbilityHandLegacyGripCommand.java#L58
            */
   public void setGripSpeed(java.lang.String grip_speed)
   {
      grip_speed_.setLength(0);
      grip_speed_.append(grip_speed);
   }

   /**
            * The string value of the legacy grip speed
            * https://github.com/ihmcrobotics/psyonic-ability-hand-java/blob/main/src/main/java/us/ihmc/abilityhand/AbilityHandLegacyGripCommand.java#L58
            */
   public java.lang.String getGripSpeedAsString()
   {
      return getGripSpeed().toString();
   }
   /**
            * The string value of the legacy grip speed
            * https://github.com/ihmcrobotics/psyonic-ability-hand-java/blob/main/src/main/java/us/ihmc/abilityhand/AbilityHandLegacyGripCommand.java#L58
            */
   public java.lang.StringBuilder getGripSpeed()
   {
      return grip_speed_;
   }


   public static Supplier<PsyonicAbilityHandCommandActionDefinitionMessagePubSubType> getPubSubType()
   {
      return PsyonicAbilityHandCommandActionDefinitionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return PsyonicAbilityHandCommandActionDefinitionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(PsyonicAbilityHandCommandActionDefinitionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.grip_type_, other.grip_type_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.grip_speed_, other.grip_speed_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof PsyonicAbilityHandCommandActionDefinitionMessage)) return false;

      PsyonicAbilityHandCommandActionDefinitionMessage otherMyClass = (PsyonicAbilityHandCommandActionDefinitionMessage) other;

      if (!this.definition_.equals(otherMyClass.definition_)) return false;
      if(this.robot_side_ != otherMyClass.robot_side_) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.grip_type_, otherMyClass.grip_type_)) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.grip_speed_, otherMyClass.grip_speed_)) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("PsyonicAbilityHandCommandActionDefinitionMessage {");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");
      builder.append("grip_type=");
      builder.append(this.grip_type_);      builder.append(", ");
      builder.append("grip_speed=");
      builder.append(this.grip_speed_);
      builder.append("}");
      return builder.toString();
   }
}
