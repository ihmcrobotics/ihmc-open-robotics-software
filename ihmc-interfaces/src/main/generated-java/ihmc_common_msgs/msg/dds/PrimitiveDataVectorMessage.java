package ihmc_common_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Ordered vector of data as primitives
       * The number and ordering of values is assumed to be defined ahead of time
       * so it is not necessary to send indicies.
       */
public class PrimitiveDataVectorMessage extends Packet<PrimitiveDataVectorMessage> implements Settable<PrimitiveDataVectorMessage>, EpsilonComparable<PrimitiveDataVectorMessage>
{
   /**
            * double values
            */
   public us.ihmc.idl.IDLSequence.Double  double_values_;
   /**
            * integer values
            */
   public us.ihmc.idl.IDLSequence.Integer  integer_values_;
   /**
            * boolean values
            */
   public us.ihmc.idl.IDLSequence.Boolean  boolean_values_;

   public PrimitiveDataVectorMessage()
   {
      double_values_ = new us.ihmc.idl.IDLSequence.Double (200, "type_6");

      integer_values_ = new us.ihmc.idl.IDLSequence.Integer (200, "type_2");

      boolean_values_ = new us.ihmc.idl.IDLSequence.Boolean (200, "type_7");

   }

   public PrimitiveDataVectorMessage(PrimitiveDataVectorMessage other)
   {
      this();
      set(other);
   }

   public void set(PrimitiveDataVectorMessage other)
   {
      double_values_.set(other.double_values_);
      integer_values_.set(other.integer_values_);
      boolean_values_.set(other.boolean_values_);
   }


   /**
            * double values
            */
   public us.ihmc.idl.IDLSequence.Double  getDoubleValues()
   {
      return double_values_;
   }


   /**
            * integer values
            */
   public us.ihmc.idl.IDLSequence.Integer  getIntegerValues()
   {
      return integer_values_;
   }


   /**
            * boolean values
            */
   public us.ihmc.idl.IDLSequence.Boolean  getBooleanValues()
   {
      return boolean_values_;
   }


   public static Supplier<PrimitiveDataVectorMessagePubSubType> getPubSubType()
   {
      return PrimitiveDataVectorMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return PrimitiveDataVectorMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(PrimitiveDataVectorMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsDoubleSequence(this.double_values_, other.double_values_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsIntegerSequence(this.integer_values_, other.integer_values_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBooleanSequence(this.boolean_values_, other.boolean_values_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof PrimitiveDataVectorMessage)) return false;

      PrimitiveDataVectorMessage otherMyClass = (PrimitiveDataVectorMessage) other;

      if (!this.double_values_.equals(otherMyClass.double_values_)) return false;
      if (!this.integer_values_.equals(otherMyClass.integer_values_)) return false;
      if (!this.boolean_values_.equals(otherMyClass.boolean_values_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("PrimitiveDataVectorMessage {");
      builder.append("double_values=");
      builder.append(this.double_values_);      builder.append(", ");
      builder.append("integer_values=");
      builder.append(this.integer_values_);      builder.append(", ");
      builder.append("boolean_values=");
      builder.append(this.boolean_values_);
      builder.append("}");
      return builder.toString();
   }
}
