package unitree_go_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class LowState extends Packet<LowState> implements Settable<LowState>, EpsilonComparable<LowState>
{
   public byte[] head_;
   public byte level_flag_;
   public byte frame_reserve_;
   public int bandwidth_;
   public unitree_go_msgs.msg.dds.IMUState imu_state_;
   public unitree_go_msgs.msg.dds.MotorState[] motor_state_;
   public unitree_go_msgs.msg.dds.BmsState bms_state_;
   public short[] foot_force_;
   public short[] foot_force_est_;
   public long tick_;
   public byte[] wireless_remote_;
   public byte bit_flag_;
   public float adc_reel_;
   public byte temperature_ntc1_;
   public byte temperature_ntc2_;
   public float power_v_;
   public float power_a_;
   public long reserve_;
   public long crc_;

   public LowState()
   {
      head_ = new byte[2];

      imu_state_ = new unitree_go_msgs.msg.dds.IMUState();
      motor_state_ = new unitree_go_msgs.msg.dds.MotorState[20];

      for(int i1 = 0; i1 < motor_state_.length; ++i1)
      {
          motor_state_[i1] = new unitree_go_msgs.msg.dds.MotorState();
      }
      bms_state_ = new unitree_go_msgs.msg.dds.BmsState();
      foot_force_ = new short[4];

      foot_force_est_ = new short[4];

      wireless_remote_ = new byte[40];

   }

   public LowState(LowState other)
   {
      this();
      set(other);
   }

   public void set(LowState other)
   {
      for(int i3 = 0; i3 < head_.length; ++i3)
      {
            head_[i3] = other.head_[i3];

      }

      level_flag_ = other.level_flag_;

      frame_reserve_ = other.frame_reserve_;

      bandwidth_ = other.bandwidth_;

      unitree_go_msgs.msg.dds.IMUStatePubSubType.staticCopy(other.imu_state_, imu_state_);
      for(int i5 = 0; i5 < motor_state_.length; ++i5)
      {
            unitree_go_msgs.msg.dds.MotorStatePubSubType.staticCopy(other.motor_state_[i5], motor_state_[i5]);}

      unitree_go_msgs.msg.dds.BmsStatePubSubType.staticCopy(other.bms_state_, bms_state_);
      for(int i7 = 0; i7 < foot_force_.length; ++i7)
      {
            foot_force_[i7] = other.foot_force_[i7];

      }

      for(int i9 = 0; i9 < foot_force_est_.length; ++i9)
      {
            foot_force_est_[i9] = other.foot_force_est_[i9];

      }

      tick_ = other.tick_;

      for(int i11 = 0; i11 < wireless_remote_.length; ++i11)
      {
            wireless_remote_[i11] = other.wireless_remote_[i11];

      }

      bit_flag_ = other.bit_flag_;

      adc_reel_ = other.adc_reel_;

      temperature_ntc1_ = other.temperature_ntc1_;

      temperature_ntc2_ = other.temperature_ntc2_;

      power_v_ = other.power_v_;

      power_a_ = other.power_a_;

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


   public unitree_go_msgs.msg.dds.IMUState getImuState()
   {
      return imu_state_;
   }


   public unitree_go_msgs.msg.dds.MotorState[] getMotorState()
   {
      return motor_state_;
   }


   public unitree_go_msgs.msg.dds.BmsState getBmsState()
   {
      return bms_state_;
   }


   public short[] getFootForce()
   {
      return foot_force_;
   }


   public short[] getFootForceEst()
   {
      return foot_force_est_;
   }

   public void setTick(long tick)
   {
      tick_ = tick;
   }
   public long getTick()
   {
      return tick_;
   }


   public byte[] getWirelessRemote()
   {
      return wireless_remote_;
   }

   public void setBitFlag(byte bit_flag)
   {
      bit_flag_ = bit_flag;
   }
   public byte getBitFlag()
   {
      return bit_flag_;
   }

   public void setAdcReel(float adc_reel)
   {
      adc_reel_ = adc_reel;
   }
   public float getAdcReel()
   {
      return adc_reel_;
   }

   public void setTemperatureNtc1(byte temperature_ntc1)
   {
      temperature_ntc1_ = temperature_ntc1;
   }
   public byte getTemperatureNtc1()
   {
      return temperature_ntc1_;
   }

   public void setTemperatureNtc2(byte temperature_ntc2)
   {
      temperature_ntc2_ = temperature_ntc2;
   }
   public byte getTemperatureNtc2()
   {
      return temperature_ntc2_;
   }

   public void setPowerV(float power_v)
   {
      power_v_ = power_v;
   }
   public float getPowerV()
   {
      return power_v_;
   }

   public void setPowerA(float power_a)
   {
      power_a_ = power_a;
   }
   public float getPowerA()
   {
      return power_a_;
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


   public static Supplier<LowStatePubSubType> getPubSubType()
   {
      return LowStatePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return LowStatePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(LowState other, double epsilon)
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

      if (!this.imu_state_.epsilonEquals(other.imu_state_, epsilon)) return false;
      for(int i15 = 0; i15 < motor_state_.length; ++i15)
      {
              if (!this.motor_state_[i15].epsilonEquals(other.motor_state_[i15], epsilon)) return false;
      }

      if (!this.bms_state_.epsilonEquals(other.bms_state_, epsilon)) return false;
      for(int i17 = 0; i17 < foot_force_.length; ++i17)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.foot_force_[i17], other.foot_force_[i17], epsilon)) return false;
      }

      for(int i19 = 0; i19 < foot_force_est_.length; ++i19)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.foot_force_est_[i19], other.foot_force_est_[i19], epsilon)) return false;
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.tick_, other.tick_, epsilon)) return false;

      for(int i21 = 0; i21 < wireless_remote_.length; ++i21)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.wireless_remote_[i21], other.wireless_remote_[i21], epsilon)) return false;
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.bit_flag_, other.bit_flag_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.adc_reel_, other.adc_reel_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.temperature_ntc1_, other.temperature_ntc1_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.temperature_ntc2_, other.temperature_ntc2_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.power_v_, other.power_v_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.power_a_, other.power_a_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.reserve_, other.reserve_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.crc_, other.crc_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof LowState)) return false;

      LowState otherMyClass = (LowState) other;

      for(int i23 = 0; i23 < head_.length; ++i23)
      {
                if(this.head_[i23] != otherMyClass.head_[i23]) return false;

      }
      if(this.level_flag_ != otherMyClass.level_flag_) return false;

      if(this.frame_reserve_ != otherMyClass.frame_reserve_) return false;

      if(this.bandwidth_ != otherMyClass.bandwidth_) return false;

      if (!this.imu_state_.equals(otherMyClass.imu_state_)) return false;
      for(int i25 = 0; i25 < motor_state_.length; ++i25)
      {
                if (!this.motor_state_[i25].equals(otherMyClass.motor_state_[i25])) return false;
      }
      if (!this.bms_state_.equals(otherMyClass.bms_state_)) return false;
      for(int i27 = 0; i27 < foot_force_.length; ++i27)
      {
                if(this.foot_force_[i27] != otherMyClass.foot_force_[i27]) return false;

      }
      for(int i29 = 0; i29 < foot_force_est_.length; ++i29)
      {
                if(this.foot_force_est_[i29] != otherMyClass.foot_force_est_[i29]) return false;

      }
      if(this.tick_ != otherMyClass.tick_) return false;

      for(int i31 = 0; i31 < wireless_remote_.length; ++i31)
      {
                if(this.wireless_remote_[i31] != otherMyClass.wireless_remote_[i31]) return false;

      }
      if(this.bit_flag_ != otherMyClass.bit_flag_) return false;

      if(this.adc_reel_ != otherMyClass.adc_reel_) return false;

      if(this.temperature_ntc1_ != otherMyClass.temperature_ntc1_) return false;

      if(this.temperature_ntc2_ != otherMyClass.temperature_ntc2_) return false;

      if(this.power_v_ != otherMyClass.power_v_) return false;

      if(this.power_a_ != otherMyClass.power_a_) return false;

      if(this.reserve_ != otherMyClass.reserve_) return false;

      if(this.crc_ != otherMyClass.crc_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("LowState {");
      builder.append("head=");
      builder.append(java.util.Arrays.toString(this.head_));      builder.append(", ");
      builder.append("level_flag=");
      builder.append(this.level_flag_);      builder.append(", ");
      builder.append("frame_reserve=");
      builder.append(this.frame_reserve_);      builder.append(", ");
      builder.append("bandwidth=");
      builder.append(this.bandwidth_);      builder.append(", ");
      builder.append("imu_state=");
      builder.append(this.imu_state_);      builder.append(", ");
      builder.append("motor_state=");
      builder.append(java.util.Arrays.toString(this.motor_state_));      builder.append(", ");
      builder.append("bms_state=");
      builder.append(this.bms_state_);      builder.append(", ");
      builder.append("foot_force=");
      builder.append(java.util.Arrays.toString(this.foot_force_));      builder.append(", ");
      builder.append("foot_force_est=");
      builder.append(java.util.Arrays.toString(this.foot_force_est_));      builder.append(", ");
      builder.append("tick=");
      builder.append(this.tick_);      builder.append(", ");
      builder.append("wireless_remote=");
      builder.append(java.util.Arrays.toString(this.wireless_remote_));      builder.append(", ");
      builder.append("bit_flag=");
      builder.append(this.bit_flag_);      builder.append(", ");
      builder.append("adc_reel=");
      builder.append(this.adc_reel_);      builder.append(", ");
      builder.append("temperature_ntc1=");
      builder.append(this.temperature_ntc1_);      builder.append(", ");
      builder.append("temperature_ntc2=");
      builder.append(this.temperature_ntc2_);      builder.append(", ");
      builder.append("power_v=");
      builder.append(this.power_v_);      builder.append(", ");
      builder.append("power_a=");
      builder.append(this.power_a_);      builder.append(", ");
      builder.append("reserve=");
      builder.append(this.reserve_);      builder.append(", ");
      builder.append("crc=");
      builder.append(this.crc_);
      builder.append("}");
      return builder.toString();
   }
}
