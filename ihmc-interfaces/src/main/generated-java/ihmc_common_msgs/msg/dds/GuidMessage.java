package ihmc_common_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Represents a DDS-RTPS Guid
       */
public class GuidMessage extends Packet<GuidMessage> implements Settable<GuidMessage>, EpsilonComparable<GuidMessage>
{
   /**
            * Prefix
            */
   public byte[] prefix_;
   /**
            * Entity
            */
   public byte[] entity_;

   public GuidMessage()
   {
      prefix_ = new byte[12];

      entity_ = new byte[4];

   }

   public GuidMessage(GuidMessage other)
   {
      this();
      set(other);
   }

   public void set(GuidMessage other)
   {
      for(int i1 = 0; i1 < prefix_.length; ++i1)
      {
            prefix_[i1] = other.prefix_[i1];

      }

      for(int i3 = 0; i3 < entity_.length; ++i3)
      {
            entity_[i3] = other.entity_[i3];

      }

   }


   /**
            * Prefix
            */
   public byte[] getPrefix()
   {
      return prefix_;
   }


   /**
            * Entity
            */
   public byte[] getEntity()
   {
      return entity_;
   }


   public static Supplier<GuidMessagePubSubType> getPubSubType()
   {
      return GuidMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return GuidMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(GuidMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      for(int i5 = 0; i5 < prefix_.length; ++i5)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.prefix_[i5], other.prefix_[i5], epsilon)) return false;
      }

      for(int i7 = 0; i7 < entity_.length; ++i7)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.entity_[i7], other.entity_[i7], epsilon)) return false;
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof GuidMessage)) return false;

      GuidMessage otherMyClass = (GuidMessage) other;

      for(int i9 = 0; i9 < prefix_.length; ++i9)
      {
                if(this.prefix_[i9] != otherMyClass.prefix_[i9]) return false;

      }
      for(int i11 = 0; i11 < entity_.length; ++i11)
      {
                if(this.entity_[i11] != otherMyClass.entity_[i11]) return false;

      }

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("GuidMessage {");
      builder.append("prefix=");
      builder.append(java.util.Arrays.toString(this.prefix_));      builder.append(", ");
      builder.append("entity=");
      builder.append(java.util.Arrays.toString(this.entity_));
      builder.append("}");
      return builder.toString();
   }
}
