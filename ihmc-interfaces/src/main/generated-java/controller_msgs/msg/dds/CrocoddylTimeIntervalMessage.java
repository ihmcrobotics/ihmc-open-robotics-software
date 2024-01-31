package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This is a message that holds time interval details.
       * 
       * The time interval is defined by:
       * * the time at the beginning of the interval, and
       * * the duration of the interval.
       * 
       * The time interval information allow us to compute the beginning of
       * the reference state and control.
       */
public class CrocoddylTimeIntervalMessage extends Packet<CrocoddylTimeIntervalMessage> implements Settable<CrocoddylTimeIntervalMessage>, EpsilonComparable<CrocoddylTimeIntervalMessage>
{
   /**
            * This represents the time at the beginning of the interval
            */
   public double time_;
   /**
            * This represents the duration of the interval
            */
   public double duration_;

   public CrocoddylTimeIntervalMessage()
   {
   }

   public CrocoddylTimeIntervalMessage(CrocoddylTimeIntervalMessage other)
   {
      this();
      set(other);
   }

   public void set(CrocoddylTimeIntervalMessage other)
   {
      time_ = other.time_;

      duration_ = other.duration_;

   }

   /**
            * This represents the time at the beginning of the interval
            */
   public void setTime(double time)
   {
      time_ = time;
   }
   /**
            * This represents the time at the beginning of the interval
            */
   public double getTime()
   {
      return time_;
   }

   /**
            * This represents the duration of the interval
            */
   public void setDuration(double duration)
   {
      duration_ = duration;
   }
   /**
            * This represents the duration of the interval
            */
   public double getDuration()
   {
      return duration_;
   }


   public static Supplier<CrocoddylTimeIntervalMessagePubSubType> getPubSubType()
   {
      return CrocoddylTimeIntervalMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return CrocoddylTimeIntervalMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(CrocoddylTimeIntervalMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.time_, other.time_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.duration_, other.duration_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof CrocoddylTimeIntervalMessage)) return false;

      CrocoddylTimeIntervalMessage otherMyClass = (CrocoddylTimeIntervalMessage) other;

      if(this.time_ != otherMyClass.time_) return false;

      if(this.duration_ != otherMyClass.duration_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("CrocoddylTimeIntervalMessage {");
      builder.append("time=");
      builder.append(this.time_);      builder.append(", ");
      builder.append("duration=");
      builder.append(this.duration_);
      builder.append("}");
      return builder.toString();
   }
}
