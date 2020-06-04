package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class PilotAlarmPacket extends Packet<PilotAlarmPacket> implements Settable<PilotAlarmPacket>, EpsilonComparable<PilotAlarmPacket>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public double beep_rate_;

   public boolean enable_tone_;

   public PilotAlarmPacket()
   {




   }

   public PilotAlarmPacket(PilotAlarmPacket other)
   {
      this();
      set(other);
   }

   public void set(PilotAlarmPacket other)
   {

      sequence_id_ = other.sequence_id_;


      beep_rate_ = other.beep_rate_;


      enable_tone_ = other.enable_tone_;

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


   public void setBeepRate(double beep_rate)
   {
      beep_rate_ = beep_rate;
   }
   public double getBeepRate()
   {
      return beep_rate_;
   }


   public void setEnableTone(boolean enable_tone)
   {
      enable_tone_ = enable_tone;
   }
   public boolean getEnableTone()
   {
      return enable_tone_;
   }


   public static Supplier<PilotAlarmPacketPubSubType> getPubSubType()
   {
      return PilotAlarmPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return PilotAlarmPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(PilotAlarmPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.beep_rate_, other.beep_rate_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.enable_tone_, other.enable_tone_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof PilotAlarmPacket)) return false;

      PilotAlarmPacket otherMyClass = (PilotAlarmPacket) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.beep_rate_ != otherMyClass.beep_rate_) return false;


      if(this.enable_tone_ != otherMyClass.enable_tone_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("PilotAlarmPacket {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("beep_rate=");
      builder.append(this.beep_rate_);      builder.append(", ");

      builder.append("enable_tone=");
      builder.append(this.enable_tone_);
      builder.append("}");
      return builder.toString();
   }
}
