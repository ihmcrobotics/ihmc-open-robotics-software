package unitree_go_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class LowCmd extends Packet<LowCmd> implements Settable<LowCmd>, EpsilonComparable<LowCmd>
{
   public byte[] head_;
   public byte level_flag_;
   public byte frame_reserve_;
   public int bandwidth_;
   public unitree_go_msgs.msg.dds.MotorCmd[] motor_cmd_;
   public unitree_go_msgs.msg.dds.BmsCmd bms_cmd_;
   public byte[] wireless_remote_;
   public byte[] led_;
   public byte[] fan_;
   public byte gpio_;
   public long reserve_;
   public long crc_;

   public LowCmd()
   {
      head_ = new byte[2];

      motor_cmd_ = new unitree_go_msgs.msg.dds.MotorCmd[20];

      for(int i1 = 0; i1 < motor_cmd_.length; ++i1)
      {
          motor_cmd_[i1] = new unitree_go_msgs.msg.dds.MotorCmd();
      }
      bms_cmd_ = new unitree_go_msgs.msg.dds.BmsCmd();
      wireless_remote_ = new byte[40];

      led_ = new byte[12];

      fan_ = new byte[2];

   }

   public LowCmd(LowCmd other)
   {
      this();
      set(other);
   }

   public void set(LowCmd other)
   {
      for(int i3 = 0; i3 < head_.length; ++i3)
      {
            head_[i3] = other.head_[i3];

      }

      level_flag_ = other.level_flag_;

      frame_reserve_ = other.frame_reserve_;

      bandwidth_ = other.bandwidth_;

      for(int i5 = 0; i5 < motor_cmd_.length; ++i5)
      {
            unitree_go_msgs.msg.dds.MotorCmdPubSubType.staticCopy(other.motor_cmd_[i5], motor_cmd_[i5]);}

      unitree_go_msgs.msg.dds.BmsCmdPubSubType.staticCopy(other.bms_cmd_, bms_cmd_);
      for(int i7 = 0; i7 < wireless_remote_.length; ++i7)
      {
            wireless_remote_[i7] = other.wireless_remote_[i7];

      }

      for(int i9 = 0; i9 < led_.length; ++i9)
      {
            led_[i9] = other.led_[i9];

      }

      for(int i11 = 0; i11 < fan_.length; ++i11)
      {
            fan_[i11] = other.fan_[i11];

      }

      gpio_ = other.gpio_;

      reserve_ = other.reserve_;

      crc_ = other.crc_;

   }


   public byte[] getHead()
   {
      return head_;
   }

   public void setLevelFlag(byte level_flag)
   {
      level_flag_ = level_flag;
   }
   public byte getLevelFlag()
   {
      return level_flag_;
   }

   public void setFrameReserve(byte frame_reserve)
   {
      frame_reserve_ = frame_reserve;
   }
   public byte getFrameReserve()
   {
      return frame_reserve_;
   }

   public void setBandwidth(int bandwidth)
   {
      bandwidth_ = bandwidth;
   }
   public int getBandwidth()
   {
      return bandwidth_;
   }


   public unitree_go_msgs.msg.dds.MotorCmd[] getMotorCmd()
   {
      return motor_cmd_;
   }


   public unitree_go_msgs.msg.dds.BmsCmd getBmsCmd()
   {
      return bms_cmd_;
   }


   public byte[] getWirelessRemote()
   {
      return wireless_remote_;
   }


   public byte[] getLed()
   {
      return led_;
   }


   public byte[] getFan()
   {
      return fan_;
   }

   public void setGpio(byte gpio)
   {
      gpio_ = gpio;
   }
   public byte getGpio()
   {
      return gpio_;
   }

   public void setReserve(long reserve)
   {
      reserve_ = reserve;
   }
   public long getReserve()
   {
      return reserve_;
   }

   public void setCrc(long crc)
   {
      crc_ = crc;
   }
   public long getCrc()
   {
      return crc_;
   }


   public static Supplier<LowCmdPubSubType> getPubSubType()
   {
      return LowCmdPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return LowCmdPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(LowCmd other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      for(int i13 = 0; i13 < head_.length; ++i13)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.head_[i13], other.head_[i13], epsilon)) return false;
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.level_flag_, other.level_flag_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.frame_reserve_, other.frame_reserve_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.bandwidth_, other.bandwidth_, epsilon)) return false;

      for(int i15 = 0; i15 < motor_cmd_.length; ++i15)
      {
              if (!this.motor_cmd_[i15].epsilonEquals(other.motor_cmd_[i15], epsilon)) return false;
      }

      if (!this.bms_cmd_.epsilonEquals(other.bms_cmd_, epsilon)) return false;
      for(int i17 = 0; i17 < wireless_remote_.length; ++i17)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.wireless_remote_[i17], other.wireless_remote_[i17], epsilon)) return false;
      }

      for(int i19 = 0; i19 < led_.length; ++i19)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.led_[i19], other.led_[i19], epsilon)) return false;
      }

      for(int i21 = 0; i21 < fan_.length; ++i21)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.fan_[i21], other.fan_[i21], epsilon)) return false;
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.gpio_, other.gpio_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.reserve_, other.reserve_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.crc_, other.crc_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof LowCmd)) return false;

      LowCmd otherMyClass = (LowCmd) other;

      for(int i23 = 0; i23 < head_.length; ++i23)
      {
                if(this.head_[i23] != otherMyClass.head_[i23]) return false;

      }
      if(this.level_flag_ != otherMyClass.level_flag_) return false;

      if(this.frame_reserve_ != otherMyClass.frame_reserve_) return false;

      if(this.bandwidth_ != otherMyClass.bandwidth_) return false;

      for(int i25 = 0; i25 < motor_cmd_.length; ++i25)
      {
                if (!this.motor_cmd_[i25].equals(otherMyClass.motor_cmd_[i25])) return false;
      }
      if (!this.bms_cmd_.equals(otherMyClass.bms_cmd_)) return false;
      for(int i27 = 0; i27 < wireless_remote_.length; ++i27)
      {
                if(this.wireless_remote_[i27] != otherMyClass.wireless_remote_[i27]) return false;

      }
      for(int i29 = 0; i29 < led_.length; ++i29)
      {
                if(this.led_[i29] != otherMyClass.led_[i29]) return false;

      }
      for(int i31 = 0; i31 < fan_.length; ++i31)
      {
                if(this.fan_[i31] != otherMyClass.fan_[i31]) return false;

      }
      if(this.gpio_ != otherMyClass.gpio_) return false;

      if(this.reserve_ != otherMyClass.reserve_) return false;

      if(this.crc_ != otherMyClass.crc_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("LowCmd {");
      builder.append("head=");
      builder.append(java.util.Arrays.toString(this.head_));      builder.append(", ");
      builder.append("level_flag=");
      builder.append(this.level_flag_);      builder.append(", ");
      builder.append("frame_reserve=");
      builder.append(this.frame_reserve_);      builder.append(", ");
      builder.append("bandwidth=");
      builder.append(this.bandwidth_);      builder.append(", ");
      builder.append("motor_cmd=");
      builder.append(java.util.Arrays.toString(this.motor_cmd_));      builder.append(", ");
      builder.append("bms_cmd=");
      builder.append(this.bms_cmd_);      builder.append(", ");
      builder.append("wireless_remote=");
      builder.append(java.util.Arrays.toString(this.wireless_remote_));      builder.append(", ");
      builder.append("led=");
      builder.append(java.util.Arrays.toString(this.led_));      builder.append(", ");
      builder.append("fan=");
      builder.append(java.util.Arrays.toString(this.fan_));      builder.append(", ");
      builder.append("gpio=");
      builder.append(this.gpio_);      builder.append(", ");
      builder.append("reserve=");
      builder.append(this.reserve_);      builder.append(", ");
      builder.append("crc=");
      builder.append(this.crc_);
      builder.append("}");
      return builder.toString();
   }
}
