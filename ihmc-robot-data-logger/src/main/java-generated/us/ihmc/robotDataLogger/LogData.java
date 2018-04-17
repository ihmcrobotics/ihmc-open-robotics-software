package us.ihmc.robotDataLogger;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

public class LogData extends Packet<LogData> implements Settable<LogData>, EpsilonComparable<LogData>
{
   public long uid_;
   public long timestamp_;
   public long transmitTime_;
   public us.ihmc.robotDataLogger.LogDataType type_;
   public int registry_;
   public int offset_;
   public int numberOfVariables_;
   public us.ihmc.idl.IDLSequence.Byte  data_;
   public us.ihmc.idl.IDLSequence.Double  jointStates_;

   public LogData()
   {
      data_ = new us.ihmc.idl.IDLSequence.Byte (100, "type_9");

      jointStates_ = new us.ihmc.idl.IDLSequence.Double (100, "type_6");

   }

   public LogData(LogData other)
   {
      this();
      set(other);
   }

   public void set(LogData other)
   {
      uid_ = other.uid_;

      timestamp_ = other.timestamp_;

      transmitTime_ = other.transmitTime_;

      type_ = other.type_;

      registry_ = other.registry_;

      offset_ = other.offset_;

      numberOfVariables_ = other.numberOfVariables_;

      data_.set(other.data_);
      jointStates_.set(other.jointStates_);
   }

   public void setUid(long uid)
   {
      uid_ = uid;
   }
   public long getUid()
   {
      return uid_;
   }

   public void setTimestamp(long timestamp)
   {
      timestamp_ = timestamp;
   }
   public long getTimestamp()
   {
      return timestamp_;
   }

   public void setTransmitTime(long transmitTime)
   {
      transmitTime_ = transmitTime;
   }
   public long getTransmitTime()
   {
      return transmitTime_;
   }

   public void setType(us.ihmc.robotDataLogger.LogDataType type)
   {
      type_ = type;
   }
   public us.ihmc.robotDataLogger.LogDataType getType()
   {
      return type_;
   }

   public void setRegistry(int registry)
   {
      registry_ = registry;
   }
   public int getRegistry()
   {
      return registry_;
   }

   public void setOffset(int offset)
   {
      offset_ = offset;
   }
   public int getOffset()
   {
      return offset_;
   }

   public void setNumberOfVariables(int numberOfVariables)
   {
      numberOfVariables_ = numberOfVariables;
   }
   public int getNumberOfVariables()
   {
      return numberOfVariables_;
   }


   public us.ihmc.idl.IDLSequence.Byte  getData()
   {
      return data_;
   }


   public us.ihmc.idl.IDLSequence.Double  getJointStates()
   {
      return jointStates_;
   }


   @Override
   public boolean epsilonEquals(LogData other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.uid_, other.uid_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.timestamp_, other.timestamp_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.transmitTime_, other.transmitTime_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsEnum(this.type_, other.type_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.registry_, other.registry_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.offset_, other.offset_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.numberOfVariables_, other.numberOfVariables_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.data_, other.data_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsDoubleSequence(this.jointStates_, other.jointStates_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof LogData)) return false;

      LogData otherMyClass = (LogData) other;

      if(this.uid_ != otherMyClass.uid_) return false;

      if(this.timestamp_ != otherMyClass.timestamp_) return false;

      if(this.transmitTime_ != otherMyClass.transmitTime_) return false;

      if(this.type_ != otherMyClass.type_) return false;

      if(this.registry_ != otherMyClass.registry_) return false;

      if(this.offset_ != otherMyClass.offset_) return false;

      if(this.numberOfVariables_ != otherMyClass.numberOfVariables_) return false;

      if (!this.data_.equals(otherMyClass.data_)) return false;
      if (!this.jointStates_.equals(otherMyClass.jointStates_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("LogData {");
      builder.append("uid=");
      builder.append(this.uid_);      builder.append(", ");
      builder.append("timestamp=");
      builder.append(this.timestamp_);      builder.append(", ");
      builder.append("transmitTime=");
      builder.append(this.transmitTime_);      builder.append(", ");
      builder.append("type=");
      builder.append(this.type_);      builder.append(", ");
      builder.append("registry=");
      builder.append(this.registry_);      builder.append(", ");
      builder.append("offset=");
      builder.append(this.offset_);      builder.append(", ");
      builder.append("numberOfVariables=");
      builder.append(this.numberOfVariables_);      builder.append(", ");
      builder.append("data=");
      builder.append(this.data_);      builder.append(", ");
      builder.append("jointStates=");
      builder.append(this.jointStates_);
      builder.append("}");
      return builder.toString();
   }
}
