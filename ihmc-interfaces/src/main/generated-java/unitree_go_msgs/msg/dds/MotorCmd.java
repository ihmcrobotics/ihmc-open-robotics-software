package unitree_go_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class MotorCmd extends Packet<MotorCmd> implements Settable<MotorCmd>, EpsilonComparable<MotorCmd>
{
   public byte mode_;
   public float q_;
   public float dq_;
   public float tau_;
   public float kp_;
   public float kd_;

   public MotorCmd()
   {
   }

   public MotorCmd(MotorCmd other)
   {
      this();
      set(other);
   }

   public void set(MotorCmd other)
   {
      mode_ = other.mode_;

      q_ = other.q_;

      dq_ = other.dq_;

      tau_ = other.tau_;

      kp_ = other.kp_;

      kd_ = other.kd_;

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

   public void setTau(float tau)
   {
      tau_ = tau;
   }
   public float getTau()
   {
      return tau_;
   }

   public void setKp(float kp)
   {
      kp_ = kp;
   }
   public float getKp()
   {
      return kp_;
   }

   public void setKd(float kd)
   {
      kd_ = kd;
   }
   public float getKd()
   {
      return kd_;
   }


   public static Supplier<MotorCmdPubSubType> getPubSubType()
   {
      return MotorCmdPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return MotorCmdPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(MotorCmd other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.mode_, other.mode_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.q_, other.q_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.dq_, other.dq_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.tau_, other.tau_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.kp_, other.kp_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.kd_, other.kd_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof MotorCmd)) return false;

      MotorCmd otherMyClass = (MotorCmd) other;

      if(this.mode_ != otherMyClass.mode_) return false;

      if(this.q_ != otherMyClass.q_) return false;

      if(this.dq_ != otherMyClass.dq_) return false;

      if(this.tau_ != otherMyClass.tau_) return false;

      if(this.kp_ != otherMyClass.kp_) return false;

      if(this.kd_ != otherMyClass.kd_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("MotorCmd {");
      builder.append("mode=");
      builder.append(this.mode_);      builder.append(", ");
      builder.append("q=");
      builder.append(this.q_);      builder.append(", ");
      builder.append("dq=");
      builder.append(this.dq_);      builder.append(", ");
      builder.append("tau=");
      builder.append(this.tau_);      builder.append(", ");
      builder.append("kp=");
      builder.append(this.kp_);      builder.append(", ");
      builder.append("kd=");
      builder.append(this.kd_);
      builder.append("}");
      return builder.toString();
   }
}
