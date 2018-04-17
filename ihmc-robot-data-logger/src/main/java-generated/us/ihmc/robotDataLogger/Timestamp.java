package us.ihmc.robotDataLogger;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

public class Timestamp extends Packet<Timestamp> implements Settable<Timestamp>, EpsilonComparable<Timestamp>
{
   public long timestamp_;

   public Timestamp()
   {
   }

   public Timestamp(Timestamp other)
   {
      this();
      set(other);
   }

   public void set(Timestamp other)
   {
      timestamp_ = other.timestamp_;

   }

   public void setTimestamp(long timestamp)
   {
      timestamp_ = timestamp;
   }
   public long getTimestamp()
   {
      return timestamp_;
   }


   @Override
   public boolean epsilonEquals(Timestamp other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.timestamp_, other.timestamp_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof Timestamp)) return false;

      Timestamp otherMyClass = (Timestamp) other;

      if(this.timestamp_ != otherMyClass.timestamp_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("Timestamp {");
      builder.append("timestamp=");
      builder.append(this.timestamp_);
      builder.append("}");
      return builder.toString();
   }
}
