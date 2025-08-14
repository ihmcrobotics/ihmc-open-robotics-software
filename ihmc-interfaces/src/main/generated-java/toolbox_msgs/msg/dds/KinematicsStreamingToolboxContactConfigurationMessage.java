package toolbox_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC Kinematics Streaming Toolbox module.
       * It contains auxiliary information that allows to customize the contact configuration based on user input.
       */
public class KinematicsStreamingToolboxContactConfigurationMessage extends Packet<KinematicsStreamingToolboxContactConfigurationMessage> implements Settable<KinematicsStreamingToolboxContactConfigurationMessage>, EpsilonComparable<KinematicsStreamingToolboxContactConfigurationMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * The contact configuration of the left foot
            */
   public boolean left_foot_in_contact_;
   /**
            * The contact configuration of the right foot
            */
   public boolean right_foot_in_contact_;

   public KinematicsStreamingToolboxContactConfigurationMessage()
   {
   }

   public KinematicsStreamingToolboxContactConfigurationMessage(KinematicsStreamingToolboxContactConfigurationMessage other)
   {
      this();
      set(other);
   }

   public void set(KinematicsStreamingToolboxContactConfigurationMessage other)
   {
      sequence_id_ = other.sequence_id_;

      left_foot_in_contact_ = other.left_foot_in_contact_;

      right_foot_in_contact_ = other.right_foot_in_contact_;

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

   /**
            * The contact configuration of the left foot
            */
   public void setLeftFootInContact(boolean left_foot_in_contact)
   {
      left_foot_in_contact_ = left_foot_in_contact;
   }
   /**
            * The contact configuration of the left foot
            */
   public boolean getLeftFootInContact()
   {
      return left_foot_in_contact_;
   }

   /**
            * The contact configuration of the right foot
            */
   public void setRightFootInContact(boolean right_foot_in_contact)
   {
      right_foot_in_contact_ = right_foot_in_contact;
   }
   /**
            * The contact configuration of the right foot
            */
   public boolean getRightFootInContact()
   {
      return right_foot_in_contact_;
   }


   public static Supplier<KinematicsStreamingToolboxContactConfigurationMessagePubSubType> getPubSubType()
   {
      return KinematicsStreamingToolboxContactConfigurationMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return KinematicsStreamingToolboxContactConfigurationMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(KinematicsStreamingToolboxContactConfigurationMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.left_foot_in_contact_, other.left_foot_in_contact_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.right_foot_in_contact_, other.right_foot_in_contact_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof KinematicsStreamingToolboxContactConfigurationMessage)) return false;

      KinematicsStreamingToolboxContactConfigurationMessage otherMyClass = (KinematicsStreamingToolboxContactConfigurationMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.left_foot_in_contact_ != otherMyClass.left_foot_in_contact_) return false;

      if(this.right_foot_in_contact_ != otherMyClass.right_foot_in_contact_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("KinematicsStreamingToolboxContactConfigurationMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("left_foot_in_contact=");
      builder.append(this.left_foot_in_contact_);      builder.append(", ");
      builder.append("right_foot_in_contact=");
      builder.append(this.right_foot_in_contact_);
      builder.append("}");
      return builder.toString();
   }
}
