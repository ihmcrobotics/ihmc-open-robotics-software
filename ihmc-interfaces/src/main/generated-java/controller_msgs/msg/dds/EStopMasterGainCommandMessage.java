package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * A message for operating the e-stop and master gain controller variables
       */
public class EStopMasterGainCommandMessage extends Packet<EStopMasterGainCommandMessage> implements Settable<EStopMasterGainCommandMessage>, EpsilonComparable<EStopMasterGainCommandMessage>
{
   /**
            * SOFT-E-STOP to set
            */
   public boolean estop_;
   /**
            * AvatarLowLevelOutputProcessor masterGain variable to set
            */
   public double master_gain_;

   public EStopMasterGainCommandMessage()
   {
   }

   public EStopMasterGainCommandMessage(EStopMasterGainCommandMessage other)
   {
      this();
      set(other);
   }

   public void set(EStopMasterGainCommandMessage other)
   {
      estop_ = other.estop_;

      master_gain_ = other.master_gain_;

   }

   /**
            * SOFT-E-STOP to set
            */
   public void setEstop(boolean estop)
   {
      estop_ = estop;
   }
   /**
            * SOFT-E-STOP to set
            */
   public boolean getEstop()
   {
      return estop_;
   }

   /**
            * AvatarLowLevelOutputProcessor masterGain variable to set
            */
   public void setMasterGain(double master_gain)
   {
      master_gain_ = master_gain;
   }
   /**
            * AvatarLowLevelOutputProcessor masterGain variable to set
            */
   public double getMasterGain()
   {
      return master_gain_;
   }


   public static Supplier<EStopMasterGainCommandMessagePubSubType> getPubSubType()
   {
      return EStopMasterGainCommandMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return EStopMasterGainCommandMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(EStopMasterGainCommandMessage other, double epsilon)
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
      if(!(other instanceof EStopMasterGainCommandMessage)) return false;

      EStopMasterGainCommandMessage otherMyClass = (EStopMasterGainCommandMessage) other;

      if(this.estop_ != otherMyClass.estop_) return false;

      if(this.master_gain_ != otherMyClass.master_gain_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("EStopMasterGainCommandMessage {");
      builder.append("estop=");
      builder.append(this.estop_);      builder.append(", ");
      builder.append("master_gain=");
      builder.append(this.master_gain_);
      builder.append("}");
      return builder.toString();
   }
}
