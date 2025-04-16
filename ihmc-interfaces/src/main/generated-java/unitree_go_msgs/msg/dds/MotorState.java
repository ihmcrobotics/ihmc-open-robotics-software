package unitree_go_msgs.msg.dds;

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
   public float q_raw_;
   public float dq_raw_;
   public float ddq_raw_;
   public byte temperature_;
   public long lost_;

   public MotorState()
   {
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

      q_raw_ = other.q_raw_;

      dq_raw_ = other.dq_raw_;

      ddq_raw_ = other.ddq_raw_;

      temperature_ = other.temperature_;

      lost_ = other.lost_;

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

   public void setQRaw(float q_raw)
   {
      q_raw_ = q_raw;
   }
   public float getQRaw()
   {
      return q_raw_;
   }

   public void setDqRaw(float dq_raw)
   {
      dq_raw_ = dq_raw;
   }
   public float getDqRaw()
   {
      return dq_raw_;
   }

   public void setDdqRaw(float ddq_raw)
   {
      ddq_raw_ = ddq_raw;
   }
   public float getDdqRaw()
   {
      return ddq_raw_;
   }

   public void setTemperature(byte temperature)
   {
      temperature_ = temperature;
   }
   public byte getTemperature()
   {
      return temperature_;
   }

   public void setLost(long lost)
   {
      lost_ = lost;
   }
   public long getLost()
   {
      return lost_;
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

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.q_raw_, other.q_raw_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.dq_raw_, other.dq_raw_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.ddq_raw_, other.ddq_raw_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.temperature_, other.temperature_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.lost_, other.lost_, epsilon)) return false;


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

      if(this.q_raw_ != otherMyClass.q_raw_) return false;

      if(this.dq_raw_ != otherMyClass.dq_raw_) return false;

      if(this.ddq_raw_ != otherMyClass.ddq_raw_) return false;

      if(this.temperature_ != otherMyClass.temperature_) return false;

      if(this.lost_ != otherMyClass.lost_) return false;


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
      builder.append("q_raw=");
      builder.append(this.q_raw_);      builder.append(", ");
      builder.append("dq_raw=");
      builder.append(this.dq_raw_);      builder.append(", ");
      builder.append("ddq_raw=");
      builder.append(this.ddq_raw_);      builder.append(", ");
      builder.append("temperature=");
      builder.append(this.temperature_);      builder.append(", ");
      builder.append("lost=");
      builder.append(this.lost_);
      builder.append("}");
      return builder.toString();
   }
}
