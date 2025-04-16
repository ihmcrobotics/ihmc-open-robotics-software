package unitree_h_one_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class BmsState extends Packet<BmsState> implements Settable<BmsState>, EpsilonComparable<BmsState>
{
   public byte version_high_;
   public byte version_low_;
   public byte fn_;
   public int current_;
   public byte soc_;
   public byte soh_;
   public short[] temperature_;
   public int cycle_;
   public int manufacturer_date_;

   public BmsState()
   {
      temperature_ = new short[12];

   }

   public BmsState(BmsState other)
   {
      this();
      set(other);
   }

   public void set(BmsState other)
   {
      version_high_ = other.version_high_;

      version_low_ = other.version_low_;

      fn_ = other.fn_;

      current_ = other.current_;

      soc_ = other.soc_;

      soh_ = other.soh_;

      for(int i1 = 0; i1 < temperature_.length; ++i1)
      {
            temperature_[i1] = other.temperature_[i1];

      }

      cycle_ = other.cycle_;

      manufacturer_date_ = other.manufacturer_date_;

   }

   public void setVersionHigh(byte version_high)
   {
      version_high_ = version_high;
   }
   public byte getVersionHigh()
   {
      return version_high_;
   }

   public void setVersionLow(byte version_low)
   {
      version_low_ = version_low;
   }
   public byte getVersionLow()
   {
      return version_low_;
   }

   public void setFn(byte fn)
   {
      fn_ = fn;
   }
   public byte getFn()
   {
      return fn_;
   }

   public void setCurrent(int current)
   {
      current_ = current;
   }
   public int getCurrent()
   {
      return current_;
   }

   public void setSoc(byte soc)
   {
      soc_ = soc;
   }
   public byte getSoc()
   {
      return soc_;
   }

   public void setSoh(byte soh)
   {
      soh_ = soh;
   }
   public byte getSoh()
   {
      return soh_;
   }


   public short[] getTemperature()
   {
      return temperature_;
   }

   public void setCycle(int cycle)
   {
      cycle_ = cycle;
   }
   public int getCycle()
   {
      return cycle_;
   }

   public void setManufacturerDate(int manufacturer_date)
   {
      manufacturer_date_ = manufacturer_date;
   }
   public int getManufacturerDate()
   {
      return manufacturer_date_;
   }


   public static Supplier<BmsStatePubSubType> getPubSubType()
   {
      return BmsStatePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return BmsStatePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(BmsState other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.version_high_, other.version_high_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.version_low_, other.version_low_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.fn_, other.fn_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_, other.current_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.soc_, other.soc_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.soh_, other.soh_, epsilon)) return false;

      for(int i3 = 0; i3 < temperature_.length; ++i3)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.temperature_[i3], other.temperature_[i3], epsilon)) return false;
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.cycle_, other.cycle_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.manufacturer_date_, other.manufacturer_date_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof BmsState)) return false;

      BmsState otherMyClass = (BmsState) other;

      if(this.version_high_ != otherMyClass.version_high_) return false;

      if(this.version_low_ != otherMyClass.version_low_) return false;

      if(this.fn_ != otherMyClass.fn_) return false;

      if(this.current_ != otherMyClass.current_) return false;

      if(this.soc_ != otherMyClass.soc_) return false;

      if(this.soh_ != otherMyClass.soh_) return false;

      for(int i5 = 0; i5 < temperature_.length; ++i5)
      {
                if(this.temperature_[i5] != otherMyClass.temperature_[i5]) return false;

      }
      if(this.cycle_ != otherMyClass.cycle_) return false;

      if(this.manufacturer_date_ != otherMyClass.manufacturer_date_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BmsState {");
      builder.append("version_high=");
      builder.append(this.version_high_);      builder.append(", ");
      builder.append("version_low=");
      builder.append(this.version_low_);      builder.append(", ");
      builder.append("fn=");
      builder.append(this.fn_);      builder.append(", ");
      builder.append("current=");
      builder.append(this.current_);      builder.append(", ");
      builder.append("soc=");
      builder.append(this.soc_);      builder.append(", ");
      builder.append("soh=");
      builder.append(this.soh_);      builder.append(", ");
      builder.append("temperature=");
      builder.append(java.util.Arrays.toString(this.temperature_));      builder.append(", ");
      builder.append("cycle=");
      builder.append(this.cycle_);      builder.append(", ");
      builder.append("manufacturer_date=");
      builder.append(this.manufacturer_date_);
      builder.append("}");
      return builder.toString();
   }
}
