package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * A message for allowing operator to observe estop and master gain values
       */
public class EStopMasterGainStatusMessage extends Packet<EStopMasterGainStatusMessage> implements Settable<EStopMasterGainStatusMessage>, EpsilonComparable<EStopMasterGainStatusMessage>
{
   /**
            * Current SOFT-E-STOP value
            */
   public boolean estop_;
   /**
            * Current masterGain value
            */
   public double master_gain_;

   public EStopMasterGainStatusMessage()
   {
   }

   public EStopMasterGainStatusMessage(EStopMasterGainStatusMessage other)
   {
      this();
      set(other);
   }

   public void set(EStopMasterGainStatusMessage other)
   {
      estop_ = other.estop_;

      master_gain_ = other.master_gain_;

   }

   /**
            * Current SOFT-E-STOP value
            */
   public void setEstop(boolean estop)
   {
      estop_ = estop;
   }
   /**
            * Current SOFT-E-STOP value
            */
   public boolean getEstop()
   {
      return estop_;
   }

   /**
            * Current masterGain value
            */
   public void setMasterGain(double master_gain)
   {
      master_gain_ = master_gain;
   }
   /**
            * Current masterGain value
            */
   public double getMasterGain()
   {
      return master_gain_;
   }


   public static Supplier<EStopMasterGainStatusMessagePubSubType> getPubSubType()
   {
      return EStopMasterGainStatusMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return EStopMasterGainStatusMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(EStopMasterGainStatusMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.estop_, other.estop_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.master_gain_, other.master_gain_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof EStopMasterGainStatusMessage)) return false;

      EStopMasterGainStatusMessage otherMyClass = (EStopMasterGainStatusMessage) other;

      if(this.estop_ != otherMyClass.estop_) return false;

      if(this.master_gain_ != otherMyClass.master_gain_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("EStopMasterGainStatusMessage {");
      builder.append("estop=");
      builder.append(this.estop_);      builder.append(", ");
      builder.append("master_gain=");
      builder.append(this.master_gain_);
      builder.append("}");
      return builder.toString();
   }
}
