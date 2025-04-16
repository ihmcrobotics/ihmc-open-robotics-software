package unitree_h_one_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class LowState extends Packet<LowState> implements Settable<LowState>, EpsilonComparable<LowState>
{
   public byte mode_pr_;
   public byte mode_machine_;
   public long tick_;
   public unitree_h_one_msgs.msg.dds.IMUState imu_state_;
   public unitree_h_one_msgs.msg.dds.MotorState[] motor_state_;
   public byte[] wireless_remote_;
   public long crc_;

   public LowState()
   {
      imu_state_ = new unitree_h_one_msgs.msg.dds.IMUState();
      motor_state_ = new unitree_h_one_msgs.msg.dds.MotorState[35];

      for(int i1 = 0; i1 < motor_state_.length; ++i1)
      {
          motor_state_[i1] = new unitree_h_one_msgs.msg.dds.MotorState();
      }
      wireless_remote_ = new byte[40];

   }

   public LowState(LowState other)
   {
      this();
      set(other);
   }

   public void set(LowState other)
   {
      mode_pr_ = other.mode_pr_;

      mode_machine_ = other.mode_machine_;

      tick_ = other.tick_;

      unitree_h_one_msgs.msg.dds.IMUStatePubSubType.staticCopy(other.imu_state_, imu_state_);
      for(int i3 = 0; i3 < motor_state_.length; ++i3)
      {
            unitree_h_one_msgs.msg.dds.MotorStatePubSubType.staticCopy(other.motor_state_[i3], motor_state_[i3]);}

      for(int i5 = 0; i5 < wireless_remote_.length; ++i5)
      {
            wireless_remote_[i5] = other.wireless_remote_[i5];

      }

      crc_ = other.crc_;

   }

   public void setModePr(byte mode_pr)
   {
      mode_pr_ = mode_pr;
   }
   public byte getModePr()
   {
      return mode_pr_;
   }

   public void setModeMachine(byte mode_machine)
   {
      mode_machine_ = mode_machine;
   }
   public byte getModeMachine()
   {
      return mode_machine_;
   }

   public void setTick(long tick)
   {
      tick_ = tick;
   }
   public long getTick()
   {
      return tick_;
   }


   public unitree_h_one_msgs.msg.dds.IMUState getImuState()
   {
      return imu_state_;
   }


   public unitree_h_one_msgs.msg.dds.MotorState[] getMotorState()
   {
      return motor_state_;
   }


   public byte[] getWirelessRemote()
   {
      return wireless_remote_;
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

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.mode_pr_, other.mode_pr_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.mode_machine_, other.mode_machine_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.tick_, other.tick_, epsilon)) return false;

      if (!this.imu_state_.epsilonEquals(other.imu_state_, epsilon)) return false;
      for(int i7 = 0; i7 < motor_state_.length; ++i7)
      {
              if (!this.motor_state_[i7].epsilonEquals(other.motor_state_[i7], epsilon)) return false;
      }

      for(int i9 = 0; i9 < wireless_remote_.length; ++i9)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.wireless_remote_[i9], other.wireless_remote_[i9], epsilon)) return false;
      }

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

      if(this.mode_pr_ != otherMyClass.mode_pr_) return false;

      if(this.mode_machine_ != otherMyClass.mode_machine_) return false;

      if(this.tick_ != otherMyClass.tick_) return false;

      if (!this.imu_state_.equals(otherMyClass.imu_state_)) return false;
      for(int i11 = 0; i11 < motor_state_.length; ++i11)
      {
                if (!this.motor_state_[i11].equals(otherMyClass.motor_state_[i11])) return false;
      }
      for(int i13 = 0; i13 < wireless_remote_.length; ++i13)
      {
                if(this.wireless_remote_[i13] != otherMyClass.wireless_remote_[i13]) return false;

      }
      if(this.crc_ != otherMyClass.crc_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("LowState {");
      builder.append("mode_pr=");
      builder.append(this.mode_pr_);      builder.append(", ");
      builder.append("mode_machine=");
      builder.append(this.mode_machine_);      builder.append(", ");
      builder.append("tick=");
      builder.append(this.tick_);      builder.append(", ");
      builder.append("imu_state=");
      builder.append(this.imu_state_);      builder.append(", ");
      builder.append("motor_state=");
      builder.append(java.util.Arrays.toString(this.motor_state_));      builder.append(", ");
      builder.append("wireless_remote=");
      builder.append(java.util.Arrays.toString(this.wireless_remote_));      builder.append(", ");
      builder.append("crc=");
      builder.append(this.crc_);
      builder.append("}");
      return builder.toString();
   }
}
