package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class AbilityHandLegacyGripCommandMessage extends Packet<AbilityHandLegacyGripCommandMessage> implements Settable<AbilityHandLegacyGripCommandMessage>, EpsilonComparable<AbilityHandLegacyGripCommandMessage>
{
   /**
            * https://github.com/ihmcrobotics/psyonic-ability-hand-java/blob/main/src/main/java/us/ihmc/abilityhand/AbilityHandLegacyGripCommand.java
            */
   public byte legacy_grip_type_;
   public java.lang.StringBuilder legacy_grip_speed_;

   public AbilityHandLegacyGripCommandMessage()
   {
      legacy_grip_speed_ = new java.lang.StringBuilder(255);
   }

   public AbilityHandLegacyGripCommandMessage(AbilityHandLegacyGripCommandMessage other)
   {
      this();
      set(other);
   }

   public void set(AbilityHandLegacyGripCommandMessage other)
   {
      legacy_grip_type_ = other.legacy_grip_type_;

      legacy_grip_speed_.setLength(0);
      legacy_grip_speed_.append(other.legacy_grip_speed_);

   }

   /**
            * https://github.com/ihmcrobotics/psyonic-ability-hand-java/blob/main/src/main/java/us/ihmc/abilityhand/AbilityHandLegacyGripCommand.java
            */
   public void setLegacyGripType(byte legacy_grip_type)
   {
      legacy_grip_type_ = legacy_grip_type;
   }
   /**
            * https://github.com/ihmcrobotics/psyonic-ability-hand-java/blob/main/src/main/java/us/ihmc/abilityhand/AbilityHandLegacyGripCommand.java
            */
   public byte getLegacyGripType()
   {
      return legacy_grip_type_;
   }

   public void setLegacyGripSpeed(java.lang.String legacy_grip_speed)
   {
      legacy_grip_speed_.setLength(0);
      legacy_grip_speed_.append(legacy_grip_speed);
   }

   public java.lang.String getLegacyGripSpeedAsString()
   {
      return getLegacyGripSpeed().toString();
   }
   public java.lang.StringBuilder getLegacyGripSpeed()
   {
      return legacy_grip_speed_;
   }


   public static Supplier<AbilityHandLegacyGripCommandMessagePubSubType> getPubSubType()
   {
      return AbilityHandLegacyGripCommandMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return AbilityHandLegacyGripCommandMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(AbilityHandLegacyGripCommandMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.legacy_grip_type_, other.legacy_grip_type_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.legacy_grip_speed_, other.legacy_grip_speed_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof AbilityHandLegacyGripCommandMessage)) return false;

      AbilityHandLegacyGripCommandMessage otherMyClass = (AbilityHandLegacyGripCommandMessage) other;

      if(this.legacy_grip_type_ != otherMyClass.legacy_grip_type_) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.legacy_grip_speed_, otherMyClass.legacy_grip_speed_)) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AbilityHandLegacyGripCommandMessage {");
      builder.append("legacy_grip_type=");
      builder.append(this.legacy_grip_type_);      builder.append(", ");
      builder.append("legacy_grip_speed=");
      builder.append(this.legacy_grip_speed_);
      builder.append("}");
      return builder.toString();
   }
}
