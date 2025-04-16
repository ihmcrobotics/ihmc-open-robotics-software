package unitree_go_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class BmsState extends Packet<BmsState> implements Settable<BmsState>, EpsilonComparable<BmsState>
{
   public byte version_high_;
   public byte version_low_;
   public byte status_;
   public byte soc_;
   public int current_;
   public int cycle_;
   public byte[] bq_ntc_;
   public byte[] mcu_ntc_;

   public BmsState()
   {
      bq_ntc_ = new byte[2];

      mcu_ntc_ = new byte[2];

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

      status_ = other.status_;

      soc_ = other.soc_;

      current_ = other.current_;

      cycle_ = other.cycle_;

      for(int i1 = 0; i1 < bq_ntc_.length; ++i1)
      {
            bq_ntc_[i1] = other.bq_ntc_[i1];

      }

      for(int i3 = 0; i3 < mcu_ntc_.length; ++i3)
      {
            mcu_ntc_[i3] = other.mcu_ntc_[i3];

      }

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

   public void setStatus(byte status)
   {
      status_ = status;
   }
   public byte getStatus()
   {
      return status_;
   }

   public void setSoc(byte soc)
   {
      soc_ = soc;
   }
   public byte getSoc()
   {
      return soc_;
   }

   public void setCurrent(int current)
   {
      current_ = current;
   }
   public int getCurrent()
   {
      return current_;
   }

   public void setCycle(int cycle)
   {
      cycle_ = cycle;
   }
   public int getCycle()
   {
      return cycle_;
   }


   public byte[] getBqNtc()
   {
      return bq_ntc_;
   }


   public byte[] getMcuNtc()
   {
      return mcu_ntc_;
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

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.status_, other.status_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.soc_, other.soc_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_, other.current_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.cycle_, other.cycle_, epsilon)) return false;

      for(int i5 = 0; i5 < bq_ntc_.length; ++i5)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.bq_ntc_[i5], other.bq_ntc_[i5], epsilon)) return false;
      }

      for(int i7 = 0; i7 < mcu_ntc_.length; ++i7)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.mcu_ntc_[i7], other.mcu_ntc_[i7], epsilon)) return false;
      }


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

      if(this.status_ != otherMyClass.status_) return false;

      if(this.soc_ != otherMyClass.soc_) return false;

      if(this.current_ != otherMyClass.current_) return false;

      if(this.cycle_ != otherMyClass.cycle_) return false;

      for(int i9 = 0; i9 < bq_ntc_.length; ++i9)
      {
                if(this.bq_ntc_[i9] != otherMyClass.bq_ntc_[i9]) return false;

      }
      for(int i11 = 0; i11 < mcu_ntc_.length; ++i11)
      {
                if(this.mcu_ntc_[i11] != otherMyClass.mcu_ntc_[i11]) return false;

      }

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
      builder.append("status=");
      builder.append(this.status_);      builder.append(", ");
      builder.append("soc=");
      builder.append(this.soc_);      builder.append(", ");
      builder.append("current=");
      builder.append(this.current_);      builder.append(", ");
      builder.append("cycle=");
      builder.append(this.cycle_);      builder.append(", ");
      builder.append("bq_ntc=");
      builder.append(java.util.Arrays.toString(this.bq_ntc_));      builder.append(", ");
      builder.append("mcu_ntc=");
      builder.append(java.util.Arrays.toString(this.mcu_ntc_));
      builder.append("}");
      return builder.toString();
   }
}
