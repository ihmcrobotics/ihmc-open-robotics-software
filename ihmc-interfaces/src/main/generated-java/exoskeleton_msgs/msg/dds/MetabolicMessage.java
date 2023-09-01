package exoskeleton_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the Exoskeleton Metabolic Estimation Interface
       * This message acts as a go-between between Java and Python.
       */
public class MetabolicMessage extends Packet<MetabolicMessage> implements Settable<MetabolicMessage>, EpsilonComparable<MetabolicMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Current metabolic rate plus the change from the previous metabolic rate
            */
   public double metabolic_rate_;
   public double delta_metabolic_rate_;

   public MetabolicMessage()
   {
   }

   public MetabolicMessage(MetabolicMessage other)
   {
      this();
      set(other);
   }

   public void set(MetabolicMessage other)
   {
      sequence_id_ = other.sequence_id_;

      metabolic_rate_ = other.metabolic_rate_;

      delta_metabolic_rate_ = other.delta_metabolic_rate_;

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
            * Current metabolic rate plus the change from the previous metabolic rate
            */
   public void setMetabolicRate(double metabolic_rate)
   {
      metabolic_rate_ = metabolic_rate;
   }
   /**
            * Current metabolic rate plus the change from the previous metabolic rate
            */
   public double getMetabolicRate()
   {
      return metabolic_rate_;
   }

   public void setDeltaMetabolicRate(double delta_metabolic_rate)
   {
      delta_metabolic_rate_ = delta_metabolic_rate;
   }
   public double getDeltaMetabolicRate()
   {
      return delta_metabolic_rate_;
   }


   public static Supplier<MetabolicMessagePubSubType> getPubSubType()
   {
      return MetabolicMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return MetabolicMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(MetabolicMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.metabolic_rate_, other.metabolic_rate_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.delta_metabolic_rate_, other.delta_metabolic_rate_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof MetabolicMessage)) return false;

      MetabolicMessage otherMyClass = (MetabolicMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.metabolic_rate_ != otherMyClass.metabolic_rate_) return false;

      if(this.delta_metabolic_rate_ != otherMyClass.delta_metabolic_rate_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("MetabolicMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("metabolic_rate=");
      builder.append(this.metabolic_rate_);      builder.append(", ");
      builder.append("delta_metabolic_rate=");
      builder.append(this.delta_metabolic_rate_);
      builder.append("}");
      return builder.toString();
   }
}
