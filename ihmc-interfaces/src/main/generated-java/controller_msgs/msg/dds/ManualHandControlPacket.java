package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Atlas specific message
       */
public class ManualHandControlPacket extends Packet<ManualHandControlPacket> implements Settable<ManualHandControlPacket>, EpsilonComparable<ManualHandControlPacket>
{

   public static final byte ROBOT_SIDE_LEFT = (byte) 0;

   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;

   public static final int VELOCITY = 0;

   public static final int POSITION = 1;

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public byte robot_side_ = (byte) 255;

   public double index_;

   public double middle_;

   public double thumb_;

   public double spread_;

   public int control_type_;

   public ManualHandControlPacket()
   {








   }

   public ManualHandControlPacket(ManualHandControlPacket other)
   {
      this();
      set(other);
   }

   public void set(ManualHandControlPacket other)
   {

      sequence_id_ = other.sequence_id_;


      robot_side_ = other.robot_side_;


      index_ = other.index_;


      middle_ = other.middle_;


      thumb_ = other.thumb_;


      spread_ = other.spread_;


      control_type_ = other.control_type_;

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


   public void setRobotSide(byte robot_side)
   {
      robot_side_ = robot_side;
   }
   public byte getRobotSide()
   {
      return robot_side_;
   }


   public void setIndex(double index)
   {
      index_ = index;
   }
   public double getIndex()
   {
      return index_;
   }


   public void setMiddle(double middle)
   {
      middle_ = middle;
   }
   public double getMiddle()
   {
      return middle_;
   }


   public void setThumb(double thumb)
   {
      thumb_ = thumb;
   }
   public double getThumb()
   {
      return thumb_;
   }


   public void setSpread(double spread)
   {
      spread_ = spread;
   }
   public double getSpread()
   {
      return spread_;
   }


   public void setControlType(int control_type)
   {
      control_type_ = control_type;
   }
   public int getControlType()
   {
      return control_type_;
   }


   public static Supplier<ManualHandControlPacketPubSubType> getPubSubType()
   {
      return ManualHandControlPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ManualHandControlPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ManualHandControlPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.index_, other.index_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.middle_, other.middle_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.thumb_, other.thumb_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.spread_, other.spread_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.control_type_, other.control_type_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ManualHandControlPacket)) return false;

      ManualHandControlPacket otherMyClass = (ManualHandControlPacket) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.robot_side_ != otherMyClass.robot_side_) return false;


      if(this.index_ != otherMyClass.index_) return false;


      if(this.middle_ != otherMyClass.middle_) return false;


      if(this.thumb_ != otherMyClass.thumb_) return false;


      if(this.spread_ != otherMyClass.spread_) return false;


      if(this.control_type_ != otherMyClass.control_type_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ManualHandControlPacket {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");

      builder.append("index=");
      builder.append(this.index_);      builder.append(", ");

      builder.append("middle=");
      builder.append(this.middle_);      builder.append(", ");

      builder.append("thumb=");
      builder.append(this.thumb_);      builder.append(", ");

      builder.append("spread=");
      builder.append(this.spread_);      builder.append(", ");

      builder.append("control_type=");
      builder.append(this.control_type_);
      builder.append("}");
      return builder.toString();
   }
}
