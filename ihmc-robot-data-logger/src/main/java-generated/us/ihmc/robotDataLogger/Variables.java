package us.ihmc.robotDataLogger;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class Variables extends Packet<Variables> implements Settable<Variables>, EpsilonComparable<Variables>
{
   public us.ihmc.robotDataLogger.HandshakeFileType handshakeFileType_;
   public java.lang.StringBuilder handshake_;
   // Handshake file name
   public java.lang.StringBuilder data_;
   // Data file name
   public java.lang.StringBuilder summary_;
   // Summary file name
   public java.lang.StringBuilder index_;
   // Variable index file
   public boolean timestamped_;
   // Does the index contain timestamps
   public boolean compressed_;

   public Variables()
   {
      handshake_ = new java.lang.StringBuilder(255);
      data_ = new java.lang.StringBuilder(255);
      summary_ = new java.lang.StringBuilder(255);
      index_ = new java.lang.StringBuilder(255);
   }

   public Variables(Variables other)
   {
      this();
      set(other);
   }

   public void set(Variables other)
   {
      handshakeFileType_ = other.handshakeFileType_;

      handshake_.setLength(0);
      handshake_.append(other.handshake_);

      data_.setLength(0);
      data_.append(other.data_);

      summary_.setLength(0);
      summary_.append(other.summary_);

      index_.setLength(0);
      index_.append(other.index_);

      timestamped_ = other.timestamped_;

      compressed_ = other.compressed_;

   }

   public void setHandshakeFileType(us.ihmc.robotDataLogger.HandshakeFileType handshakeFileType)
   {
      handshakeFileType_ = handshakeFileType;
   }
   public us.ihmc.robotDataLogger.HandshakeFileType getHandshakeFileType()
   {
      return handshakeFileType_;
   }

   public void setHandshake(java.lang.String handshake)
   {
      handshake_.setLength(0);
      handshake_.append(handshake);
   }

   public java.lang.String getHandshakeAsString()
   {
      return getHandshake().toString();
   }
   public java.lang.StringBuilder getHandshake()
   {
      return handshake_;
   }

   // Handshake file name
   public void setData(java.lang.String data)
   {
      data_.setLength(0);
      data_.append(data);
   }

   // Handshake file name
   public java.lang.String getDataAsString()
   {
      return getData().toString();
   }
   // Handshake file name
   public java.lang.StringBuilder getData()
   {
      return data_;
   }

   // Data file name
   public void setSummary(java.lang.String summary)
   {
      summary_.setLength(0);
      summary_.append(summary);
   }

   // Data file name
   public java.lang.String getSummaryAsString()
   {
      return getSummary().toString();
   }
   // Data file name
   public java.lang.StringBuilder getSummary()
   {
      return summary_;
   }

   // Summary file name
   public void setIndex(java.lang.String index)
   {
      index_.setLength(0);
      index_.append(index);
   }

   // Summary file name
   public java.lang.String getIndexAsString()
   {
      return getIndex().toString();
   }
   // Summary file name
   public java.lang.StringBuilder getIndex()
   {
      return index_;
   }

   // Variable index file
   public void setTimestamped(boolean timestamped)
   {
      timestamped_ = timestamped;
   }
   // Variable index file
   public boolean getTimestamped()
   {
      return timestamped_;
   }

   // Does the index contain timestamps
   public void setCompressed(boolean compressed)
   {
      compressed_ = compressed;
   }
   // Does the index contain timestamps
   public boolean getCompressed()
   {
      return compressed_;
   }


   public static Supplier<VariablesPubSubType> getPubSubType()
   {
      return VariablesPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return VariablesPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(Variables other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsEnum(this.handshakeFileType_, other.handshakeFileType_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.handshake_, other.handshake_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.data_, other.data_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.summary_, other.summary_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.index_, other.index_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.timestamped_, other.timestamped_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.compressed_, other.compressed_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof Variables)) return false;

      Variables otherMyClass = (Variables) other;

      if(this.handshakeFileType_ != otherMyClass.handshakeFileType_) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.handshake_, otherMyClass.handshake_)) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.data_, otherMyClass.data_)) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.summary_, otherMyClass.summary_)) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.index_, otherMyClass.index_)) return false;

      if(this.timestamped_ != otherMyClass.timestamped_) return false;

      if(this.compressed_ != otherMyClass.compressed_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("Variables {");
      builder.append("handshakeFileType=");
      builder.append(this.handshakeFileType_);      builder.append(", ");
      builder.append("handshake=");
      builder.append(this.handshake_);      builder.append(", ");
      builder.append("data=");
      builder.append(this.data_);      builder.append(", ");
      builder.append("summary=");
      builder.append(this.summary_);      builder.append(", ");
      builder.append("index=");
      builder.append(this.index_);      builder.append(", ");
      builder.append("timestamped=");
      builder.append(this.timestamped_);      builder.append(", ");
      builder.append("compressed=");
      builder.append(this.compressed_);
      builder.append("}");
      return builder.toString();
   }
}
