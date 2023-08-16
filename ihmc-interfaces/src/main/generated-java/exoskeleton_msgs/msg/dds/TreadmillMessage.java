package exoskeleton_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the Tufftread to Eva Exoskeleton Interface
       * This message acts as a go-between between the Tufftread treadmill controller and Eva
       */
public class TreadmillMessage extends Packet<TreadmillMessage> implements Settable<TreadmillMessage>, EpsilonComparable<TreadmillMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * bool to activate the treadmill
            */
   public boolean activate_;
   /**
            * byte to decide current action
            */
   public byte action_;
   /**
            * This is the data being sent to and from the treadmill
            */
   public double speed_;
   public double incline_;

   public TreadmillMessage()
   {
   }

   public TreadmillMessage(TreadmillMessage other)
   {
      this();
      set(other);
   }

   public void set(TreadmillMessage other)
   {
      sequence_id_ = other.sequence_id_;

      activate_ = other.activate_;

      action_ = other.action_;

      speed_ = other.speed_;

      incline_ = other.incline_;

   }

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public void setSequenceId(long sequence_id)
   {
      sequence_id_ = sequence_id;
   }
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long getSequenceId()
   {
      return sequence_id_;
   }

   /**
            * bool to activate the treadmill
            */
   public void setActivate(boolean activate)
   {
      activate_ = activate;
   }
   /**
            * bool to activate the treadmill
            */
   public boolean getActivate()
   {
      return activate_;
   }

   /**
            * byte to decide current action
            */
   public void setAction(byte action)
   {
      action_ = action;
   }
   /**
            * byte to decide current action
            */
   public byte getAction()
   {
      return action_;
   }

   /**
            * This is the data being sent to and from the treadmill
            */
   public void setSpeed(double speed)
   {
      speed_ = speed;
   }
   /**
            * This is the data being sent to and from the treadmill
            */
   public double getSpeed()
   {
      return speed_;
   }

   public void setIncline(double incline)
   {
      incline_ = incline;
   }
   public double getIncline()
   {
      return incline_;
   }


   public static Supplier<TreadmillMessagePubSubType> getPubSubType()
   {
      return TreadmillMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return TreadmillMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(TreadmillMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.activate_, other.activate_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.action_, other.action_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.speed_, other.speed_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.incline_, other.incline_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof TreadmillMessage)) return false;

      TreadmillMessage otherMyClass = (TreadmillMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.activate_ != otherMyClass.activate_) return false;

      if(this.action_ != otherMyClass.action_) return false;

      if(this.speed_ != otherMyClass.speed_) return false;

      if(this.incline_ != otherMyClass.incline_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("TreadmillMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("activate=");
      builder.append(this.activate_);      builder.append(", ");
      builder.append("action=");
      builder.append(this.action_);      builder.append(", ");
      builder.append("speed=");
      builder.append(this.speed_);      builder.append(", ");
      builder.append("incline=");
      builder.append(this.incline_);
      builder.append("}");
      return builder.toString();
   }
}
