package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC height map module
       */
public class TerrainMapMessage extends Packet<TerrainMapMessage> implements Settable<TerrainMapMessage>, EpsilonComparable<TerrainMapMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Height map message
            */
   public perception_msgs.msg.dds.HeightMapMessage height_map_;
   /**
            * Traversability score, on a 0 to 1 scale
            */
   public us.ihmc.idl.IDLSequence.Float  traversability_score_;
   /**
            * Classifying traversability according to SnapResult
            */
   public us.ihmc.idl.IDLSequence.Byte  traversability_class_;
   /**
            * The raw data for the snap normal x value, which are stored as chars.
            */
   public us.ihmc.idl.IDLSequence.Byte  snapped_normal_x_data_;
   /**
            * The raw data for the snap normal y value, which are stored as chars.
            */
   public us.ihmc.idl.IDLSequence.Byte  snapped_normal_y_data_;
   /**
            * The raw data for the snap normal z value, which are stored as chars.
            */
   public us.ihmc.idl.IDLSequence.Byte  snapped_normal_z_data_;

   public TerrainMapMessage()
   {
      height_map_ = new perception_msgs.msg.dds.HeightMapMessage();
      traversability_score_ = new us.ihmc.idl.IDLSequence.Float (255000, "type_5");

      traversability_class_ = new us.ihmc.idl.IDLSequence.Byte (255000, "type_9");

      snapped_normal_x_data_ = new us.ihmc.idl.IDLSequence.Byte (255000, "type_9");

      snapped_normal_y_data_ = new us.ihmc.idl.IDLSequence.Byte (255000, "type_9");

      snapped_normal_z_data_ = new us.ihmc.idl.IDLSequence.Byte (255000, "type_9");

   }

   public TerrainMapMessage(TerrainMapMessage other)
   {
      this();
      set(other);
   }

   public void set(TerrainMapMessage other)
   {
      sequence_id_ = other.sequence_id_;

      perception_msgs.msg.dds.HeightMapMessagePubSubType.staticCopy(other.height_map_, height_map_);
      traversability_score_.set(other.traversability_score_);
      traversability_class_.set(other.traversability_class_);
      snapped_normal_x_data_.set(other.snapped_normal_x_data_);
      snapped_normal_y_data_.set(other.snapped_normal_y_data_);
      snapped_normal_z_data_.set(other.snapped_normal_z_data_);
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
            * Height map message
            */
   public perception_msgs.msg.dds.HeightMapMessage getHeightMap()
   {
      return height_map_;
   }


   /**
            * Traversability score, on a 0 to 1 scale
            */
   public us.ihmc.idl.IDLSequence.Float  getTraversabilityScore()
   {
      return traversability_score_;
   }


   /**
            * Classifying traversability according to SnapResult
            */
   public us.ihmc.idl.IDLSequence.Byte  getTraversabilityClass()
   {
      return traversability_class_;
   }


   /**
            * The raw data for the snap normal x value, which are stored as chars.
            */
   public us.ihmc.idl.IDLSequence.Byte  getSnappedNormalXData()
   {
      return snapped_normal_x_data_;
   }


   /**
            * The raw data for the snap normal y value, which are stored as chars.
            */
   public us.ihmc.idl.IDLSequence.Byte  getSnappedNormalYData()
   {
      return snapped_normal_y_data_;
   }


   /**
            * The raw data for the snap normal z value, which are stored as chars.
            */
   public us.ihmc.idl.IDLSequence.Byte  getSnappedNormalZData()
   {
      return snapped_normal_z_data_;
   }


   public static Supplier<TerrainMapMessagePubSubType> getPubSubType()
   {
      return TerrainMapMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return TerrainMapMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(TerrainMapMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!this.height_map_.epsilonEquals(other.height_map_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.traversability_score_, other.traversability_score_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.traversability_class_, other.traversability_class_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.snapped_normal_x_data_, other.snapped_normal_x_data_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.snapped_normal_y_data_, other.snapped_normal_y_data_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.snapped_normal_z_data_, other.snapped_normal_z_data_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof TerrainMapMessage)) return false;

      TerrainMapMessage otherMyClass = (TerrainMapMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if (!this.height_map_.equals(otherMyClass.height_map_)) return false;
      if (!this.traversability_score_.equals(otherMyClass.traversability_score_)) return false;
      if (!this.traversability_class_.equals(otherMyClass.traversability_class_)) return false;
      if (!this.snapped_normal_x_data_.equals(otherMyClass.snapped_normal_x_data_)) return false;
      if (!this.snapped_normal_y_data_.equals(otherMyClass.snapped_normal_y_data_)) return false;
      if (!this.snapped_normal_z_data_.equals(otherMyClass.snapped_normal_z_data_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("TerrainMapMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("height_map=");
      builder.append(this.height_map_);      builder.append(", ");
      builder.append("traversability_score=");
      builder.append(this.traversability_score_);      builder.append(", ");
      builder.append("traversability_class=");
      builder.append(this.traversability_class_);      builder.append(", ");
      builder.append("snapped_normal_x_data=");
      builder.append(this.snapped_normal_x_data_);      builder.append(", ");
      builder.append("snapped_normal_y_data=");
      builder.append(this.snapped_normal_y_data_);      builder.append(", ");
      builder.append("snapped_normal_z_data=");
      builder.append(this.snapped_normal_z_data_);
      builder.append("}");
      return builder.toString();
   }
}
