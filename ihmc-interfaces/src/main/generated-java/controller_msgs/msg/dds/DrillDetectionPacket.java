package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class DrillDetectionPacket extends Packet<DrillDetectionPacket> implements Settable<DrillDetectionPacket>, EpsilonComparable<DrillDetectionPacket>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public boolean is_drill_on_;

   public DrillDetectionPacket()
   {



   }

   public DrillDetectionPacket(DrillDetectionPacket other)
   {
      this();
      set(other);
   }

   public void set(DrillDetectionPacket other)
   {

      sequence_id_ = other.sequence_id_;


      is_drill_on_ = other.is_drill_on_;

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


   public void setIsDrillOn(boolean is_drill_on)
   {
      is_drill_on_ = is_drill_on;
   }
   public boolean getIsDrillOn()
   {
      return is_drill_on_;
   }


   public static Supplier<DrillDetectionPacketPubSubType> getPubSubType()
   {
      return DrillDetectionPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return DrillDetectionPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(DrillDetectionPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.is_drill_on_, other.is_drill_on_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof DrillDetectionPacket)) return false;

      DrillDetectionPacket otherMyClass = (DrillDetectionPacket) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.is_drill_on_ != otherMyClass.is_drill_on_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("DrillDetectionPacket {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("is_drill_on=");
      builder.append(this.is_drill_on_);
      builder.append("}");
      return builder.toString();
   }
}
