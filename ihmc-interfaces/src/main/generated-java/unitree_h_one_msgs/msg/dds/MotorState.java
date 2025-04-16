package unitree_h_one_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class MotorState extends Packet<MotorState> implements Settable<MotorState>, EpsilonComparable<MotorState>
{
   public byte mode_;
   public float q_;
   public float dq_;
   public float ddq_;
   public float tau_est_;
   public short[] temperature_;
   public float vol_;
   public long motorstate_;

   public MotorState()
   {
      temperature_ = new short[2];

   }

   public MotorState(MotorState other)
   {
      this();
      set(other);
   }

   public void set(MotorState other)
   {
      mode_ = other.mode_;

      q_ = other.q_;

      dq_ = other.dq_;

      ddq_ = other.ddq_;

      tau_est_ = other.tau_est_;

      for(int i1 = 0; i1 < temperature_.length; ++i1)
      {
            temperature_[i1] = other.temperature_[i1];

      }

      vol_ = other.vol_;

      motorstate_ = other.motorstate_;

   }

   public void setMode(byte mode)
   {
      mode_ = mode;
   }
   public byte getMode()
   {
      return mode_;
   }

   public void setQ(float q)
   {
      q_ = q;
   }
   public float getQ()
   {
      return q_;
   }

   public void setDq(float dq)
   {
      dq_ = dq;
   }
   public float getDq()
   {
      return dq_;
   }

   public void setDdq(float ddq)
   {
      ddq_ = ddq;
   }
   public float getDdq()
   {
      return ddq_;
   }

   public void setTauEst(float tau_est)
   {
      tau_est_ = tau_est;
   }
   public float getTauEst()
   {
      return tau_est_;
   }


   public short[] getTemperature()
   {
      return temperature_;
   }

   public void setVol(float vol)
   {
      vol_ = vol;
   }
   public float getVol()
   {
      return vol_;
   }

   public void setMotorstate(long motorstate)
   {
      motorstate_ = motorstate;
   }
   public long getMotorstate()
   {
      return motorstate_;
   }


   public static Supplier<MotorStatePubSubType> getPubSubType()
   {
      return MotorStatePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return MotorStatePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(MotorState other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.mode_, other.mode_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.q_, other.q_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.dq_, other.dq_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.ddq_, other.ddq_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.tau_est_, other.tau_est_, epsilon)) return false;

      for(int i3 = 0; i3 < temperature_.length; ++i3)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.temperature_[i3], other.temperature_[i3], epsilon)) return false;
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.vol_, other.vol_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.motorstate_, other.motorstate_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof MotorState)) return false;

      MotorState otherMyClass = (MotorState) other;

      if(this.mode_ != otherMyClass.mode_) return false;

      if(this.q_ != otherMyClass.q_) return false;

      if(this.dq_ != otherMyClass.dq_) return false;

      if(this.ddq_ != otherMyClass.ddq_) return false;

      if(this.tau_est_ != otherMyClass.tau_est_) return false;

      for(int i5 = 0; i5 < temperature_.length; ++i5)
      {
                if(this.temperature_[i5] != otherMyClass.temperature_[i5]) return false;

      }
      if(this.vol_ != otherMyClass.vol_) return false;

      if(this.motorstate_ != otherMyClass.motorstate_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("MotorState {");
      builder.append("mode=");
      builder.append(this.mode_);      builder.append(", ");
      builder.append("q=");
      builder.append(this.q_);      builder.append(", ");
      builder.append("dq=");
      builder.append(this.dq_);      builder.append(", ");
      builder.append("ddq=");
      builder.append(this.ddq_);      builder.append(", ");
      builder.append("tau_est=");
      builder.append(this.tau_est_);      builder.append(", ");
      builder.append("temperature=");
      builder.append(java.util.Arrays.toString(this.temperature_));      builder.append(", ");
      builder.append("vol=");
      builder.append(this.vol_);      builder.append(", ");
      builder.append("motorstate=");
      builder.append(this.motorstate_);
      builder.append("}");
      return builder.toString();
   }
}
