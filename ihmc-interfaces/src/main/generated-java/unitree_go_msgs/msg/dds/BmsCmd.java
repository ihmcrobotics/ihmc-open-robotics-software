package unitree_go_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class BmsCmd extends Packet<BmsCmd> implements Settable<BmsCmd>, EpsilonComparable<BmsCmd>
{
   public byte off_;
   public byte[] reserve_;

   public BmsCmd()
   {
      reserve_ = new byte[3];

   }

   public BmsCmd(BmsCmd other)
   {
      this();
      set(other);
   }

   public void set(BmsCmd other)
   {
      off_ = other.off_;

      for(int i1 = 0; i1 < reserve_.length; ++i1)
      {
            reserve_[i1] = other.reserve_[i1];

      }

   }

   public void setOff(byte off)
   {
      off_ = off;
   }
   public byte getOff()
   {
      return off_;
   }


   public byte[] getReserve()
   {
      return reserve_;
   }


   public static Supplier<BmsCmdPubSubType> getPubSubType()
   {
      return BmsCmdPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return BmsCmdPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(BmsCmd other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.off_, other.off_, epsilon)) return false;

      for(int i3 = 0; i3 < reserve_.length; ++i3)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.reserve_[i3], other.reserve_[i3], epsilon)) return false;
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof BmsCmd)) return false;

      BmsCmd otherMyClass = (BmsCmd) other;

      if(this.off_ != otherMyClass.off_) return false;

      for(int i5 = 0; i5 < reserve_.length; ++i5)
      {
                if(this.reserve_[i5] != otherMyClass.reserve_[i5]) return false;

      }

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BmsCmd {");
      builder.append("off=");
      builder.append(this.off_);      builder.append(", ");
      builder.append("reserve=");
      builder.append(java.util.Arrays.toString(this.reserve_));
      builder.append("}");
      return builder.toString();
   }
}
