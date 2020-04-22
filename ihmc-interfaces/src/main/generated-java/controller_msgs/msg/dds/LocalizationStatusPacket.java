package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Message part of the localization module
       */
public class LocalizationStatusPacket extends Packet<LocalizationStatusPacket> implements Settable<LocalizationStatusPacket>, EpsilonComparable<LocalizationStatusPacket>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public double overlap_;

   public java.lang.StringBuilder status_;

   public LocalizationStatusPacket()
   {



      status_ = new java.lang.StringBuilder(255);

   }

   public LocalizationStatusPacket(LocalizationStatusPacket other)
   {
      this();
      set(other);
   }

   public void set(LocalizationStatusPacket other)
   {

      sequence_id_ = other.sequence_id_;


      overlap_ = other.overlap_;


      status_.setLength(0);
      status_.append(other.status_);

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


   public void setOverlap(double overlap)
   {
      overlap_ = overlap;
   }
   public double getOverlap()
   {
      return overlap_;
   }


   public void setStatus(java.lang.String status)
   {
      status_.setLength(0);
      status_.append(status);
   }

   public java.lang.String getStatusAsString()
   {
      return getStatus().toString();
   }
   public java.lang.StringBuilder getStatus()
   {
      return status_;
   }


   public static Supplier<LocalizationStatusPacketPubSubType> getPubSubType()
   {
      return LocalizationStatusPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return LocalizationStatusPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(LocalizationStatusPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.overlap_, other.overlap_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.status_, other.status_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof LocalizationStatusPacket)) return false;

      LocalizationStatusPacket otherMyClass = (LocalizationStatusPacket) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.overlap_ != otherMyClass.overlap_) return false;


      if (!us.ihmc.idl.IDLTools.equals(this.status_, otherMyClass.status_)) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("LocalizationStatusPacket {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("overlap=");
      builder.append(this.overlap_);      builder.append(", ");

      builder.append("status=");
      builder.append(this.status_);
      builder.append("}");
      return builder.toString();
   }
}
