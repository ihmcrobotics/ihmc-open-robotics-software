package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * this represents the gains for a PID controller
       */
public class PIDGainsMessage extends Packet<PIDGainsMessage> implements Settable<PIDGainsMessage>, EpsilonComparable<PIDGainsMessage>
{
   public double kp_;
   public double kd_;
   public double ki_;
   public double zeta_;

   public PIDGainsMessage()
   {
   }

   public PIDGainsMessage(PIDGainsMessage other)
   {
      this();
      set(other);
   }

   public void set(PIDGainsMessage other)
   {
      kp_ = other.kp_;

      kd_ = other.kd_;

      ki_ = other.ki_;

      zeta_ = other.zeta_;

   }

   public void setKp(double kp)
   {
      kp_ = kp;
   }
   public double getKp()
   {
      return kp_;
   }

   public void setKd(double kd)
   {
      kd_ = kd;
   }
   public double getKd()
   {
      return kd_;
   }

   public void setKi(double ki)
   {
      ki_ = ki;
   }
   public double getKi()
   {
      return ki_;
   }

   public void setZeta(double zeta)
   {
      zeta_ = zeta;
   }
   public double getZeta()
   {
      return zeta_;
   }


   public static Supplier<PIDGainsMessagePubSubType> getPubSubType()
   {
      return PIDGainsMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return PIDGainsMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(PIDGainsMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.kp_, other.kp_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.kd_, other.kd_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.ki_, other.ki_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.zeta_, other.zeta_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof PIDGainsMessage)) return false;

      PIDGainsMessage otherMyClass = (PIDGainsMessage) other;

      if(this.kp_ != otherMyClass.kp_) return false;

      if(this.kd_ != otherMyClass.kd_) return false;

      if(this.ki_ != otherMyClass.ki_) return false;

      if(this.zeta_ != otherMyClass.zeta_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("PIDGainsMessage {");
      builder.append("kp=");
      builder.append(this.kp_);      builder.append(", ");
      builder.append("kd=");
      builder.append(this.kd_);      builder.append(", ");
      builder.append("ki=");
      builder.append(this.ki_);      builder.append(", ");
      builder.append("zeta=");
      builder.append(this.zeta_);
      builder.append("}");
      return builder.toString();
   }
}
