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
   public static final byte START_BELT_TIMEOUT_ENABLED = (byte) 0;
   public static final byte START_BELT_TIMEOUT_DISABLED = (byte) 1;
   public static final byte STOP_BELT = (byte) 2;
   public static final byte SET_SPEED = (byte) 3;
   public static final byte SET_ELEVATION = (byte) 4;
   public static final byte AUTO_STOP = (byte) 5;
   public static final byte AUTO_MINIMUM = (byte) 6;
   public static final byte ACK_FLAG_TOGGLE = (byte) 7;
   public static final byte BELT_STATUS = (byte) 8;
   public static final byte GET_CURRENT_SPEED = (byte) 9;
   public static final byte GET_CURRENT_ELEVATION = (byte) 10;
   public static final byte GET_COMMANDED_SPEED = (byte) 11;
   public static final byte GET_COMMANDED_ELEVATION = (byte) 12;
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * bool to dictate activation of communication between the treadmill and the computer. If true, activate and if false, deactivate
            */
   public boolean activate_;
   /**
            * byte to decide current action based on the previously defined byte values
            */
   public byte action_;
   /**
            * This is the data being sent to and from the treadmill. Speed is in MPH and incline is in percent incline
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
            * bool to dictate activation of communication between the treadmill and the computer. If true, activate and if false, deactivate
            */
   public void setActivate(boolean activate)
   {
      activate_ = activate;
   }
   /**
            * bool to dictate activation of communication between the treadmill and the computer. If true, activate and if false, deactivate
            */
   public boolean getActivate()
   {
      return activate_;
   }

   /**
            * byte to decide current action based on the previously defined byte values
            */
   public void setAction(byte action)
   {
      action_ = action;
   }
   /**
            * byte to decide current action based on the previously defined byte values
            */
   public byte getAction()
   {
      return action_;
   }

   /**
            * This is the data being sent to and from the treadmill. Speed is in MPH and incline is in percent incline
            */
   public void setSpeed(double speed)
   {
      speed_ = speed;
   }
   /**
            * This is the data being sent to and from the treadmill. Speed is in MPH and incline is in percent incline
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
