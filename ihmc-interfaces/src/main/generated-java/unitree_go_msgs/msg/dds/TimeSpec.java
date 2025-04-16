package unitree_go_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class TimeSpec extends Packet<TimeSpec> implements Settable<TimeSpec>, EpsilonComparable<TimeSpec>
{
   /**
            * Time indicates a specific point in time, relative to a clock's 0 point.
            * The seconds component, valid over all int32 values.
            */
   public int sec_;
   /**
            * The nanoseconds component, valid in the range [0, 10e9).
            */
   public long nanosec_;

   public TimeSpec()
   {
   }

   public TimeSpec(TimeSpec other)
   {
      this();
      set(other);
   }

   public void set(TimeSpec other)
   {
      sec_ = other.sec_;

      nanosec_ = other.nanosec_;

   }

   /**
            * Time indicates a specific point in time, relative to a clock's 0 point.
            * The seconds component, valid over all int32 values.
            */
   public void setSec(int sec)
   {
      sec_ = sec;
   }
   /**
            * Time indicates a specific point in time, relative to a clock's 0 point.
            * The seconds component, valid over all int32 values.
            */
   public int getSec()
   {
      return sec_;
   }

   /**
            * The nanoseconds component, valid in the range [0, 10e9).
            */
   public void setNanosec(long nanosec)
   {
      nanosec_ = nanosec;
   }
   /**
            * The nanoseconds component, valid in the range [0, 10e9).
            */
   public long getNanosec()
   {
      return nanosec_;
   }


   public static Supplier<TimeSpecPubSubType> getPubSubType()
   {
      return TimeSpecPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return TimeSpecPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(TimeSpec other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sec_, other.sec_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.nanosec_, other.nanosec_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof TimeSpec)) return false;

      TimeSpec otherMyClass = (TimeSpec) other;

      if(this.sec_ != otherMyClass.sec_) return false;

      if(this.nanosec_ != otherMyClass.nanosec_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("TimeSpec {");
      builder.append("sec=");
      builder.append(this.sec_);      builder.append(", ");
      builder.append("nanosec=");
      builder.append(this.nanosec_);
      builder.append("}");
      return builder.toString();
   }
}
