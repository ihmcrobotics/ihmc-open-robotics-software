package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is obsolete
       */
public class PointCloudWorldPacket extends Packet<PointCloudWorldPacket> implements Settable<PointCloudWorldPacket>, EpsilonComparable<PointCloudWorldPacket>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public long timestamp_;

   public us.ihmc.idl.IDLSequence.Float  ground_quad_tree_support_;

   public us.ihmc.idl.IDLSequence.Float  decaying_world_scan_;

   public float default_ground_height_;

   public PointCloudWorldPacket()
   {



      ground_quad_tree_support_ = new us.ihmc.idl.IDLSequence.Float (100, "type_5");


      decaying_world_scan_ = new us.ihmc.idl.IDLSequence.Float (100, "type_5");



   }

   public PointCloudWorldPacket(PointCloudWorldPacket other)
   {
      this();
      set(other);
   }

   public void set(PointCloudWorldPacket other)
   {

      sequence_id_ = other.sequence_id_;


      timestamp_ = other.timestamp_;


      ground_quad_tree_support_.set(other.ground_quad_tree_support_);

      decaying_world_scan_.set(other.decaying_world_scan_);

      default_ground_height_ = other.default_ground_height_;

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


   public void setTimestamp(long timestamp)
   {
      timestamp_ = timestamp;
   }
   public long getTimestamp()
   {
      return timestamp_;
   }



   public us.ihmc.idl.IDLSequence.Float  getGroundQuadTreeSupport()
   {
      return ground_quad_tree_support_;
   }



   public us.ihmc.idl.IDLSequence.Float  getDecayingWorldScan()
   {
      return decaying_world_scan_;
   }


   public void setDefaultGroundHeight(float default_ground_height)
   {
      default_ground_height_ = default_ground_height;
   }
   public float getDefaultGroundHeight()
   {
      return default_ground_height_;
   }


   public static Supplier<PointCloudWorldPacketPubSubType> getPubSubType()
   {
      return PointCloudWorldPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return PointCloudWorldPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(PointCloudWorldPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.timestamp_, other.timestamp_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.ground_quad_tree_support_, other.ground_quad_tree_support_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.decaying_world_scan_, other.decaying_world_scan_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.default_ground_height_, other.default_ground_height_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof PointCloudWorldPacket)) return false;

      PointCloudWorldPacket otherMyClass = (PointCloudWorldPacket) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.timestamp_ != otherMyClass.timestamp_) return false;


      if (!this.ground_quad_tree_support_.equals(otherMyClass.ground_quad_tree_support_)) return false;

      if (!this.decaying_world_scan_.equals(otherMyClass.decaying_world_scan_)) return false;

      if(this.default_ground_height_ != otherMyClass.default_ground_height_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("PointCloudWorldPacket {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("timestamp=");
      builder.append(this.timestamp_);      builder.append(", ");

      builder.append("ground_quad_tree_support=");
      builder.append(this.ground_quad_tree_support_);      builder.append(", ");

      builder.append("decaying_world_scan=");
      builder.append(this.decaying_world_scan_);      builder.append(", ");

      builder.append("default_ground_height=");
      builder.append(this.default_ground_height_);
      builder.append("}");
      return builder.toString();
   }
}
