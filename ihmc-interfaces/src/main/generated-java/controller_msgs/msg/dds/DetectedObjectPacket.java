package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC behavior module.
       */
public class DetectedObjectPacket extends Packet<DetectedObjectPacket> implements Settable<DetectedObjectPacket>, EpsilonComparable<DetectedObjectPacket>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public us.ihmc.euclid.geometry.Pose3D pose_;

   public int id_;

   public DetectedObjectPacket()
   {


      pose_ = new us.ihmc.euclid.geometry.Pose3D();


   }

   public DetectedObjectPacket(DetectedObjectPacket other)
   {
      this();
      set(other);
   }

   public void set(DetectedObjectPacket other)
   {

      sequence_id_ = other.sequence_id_;


      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.pose_, pose_);

      id_ = other.id_;

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



   public us.ihmc.euclid.geometry.Pose3D getPose()
   {
      return pose_;
   }


   public void setId(int id)
   {
      id_ = id;
   }
   public int getId()
   {
      return id_;
   }


   public static Supplier<DetectedObjectPacketPubSubType> getPubSubType()
   {
      return DetectedObjectPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return DetectedObjectPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(DetectedObjectPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!this.pose_.epsilonEquals(other.pose_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.id_, other.id_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof DetectedObjectPacket)) return false;

      DetectedObjectPacket otherMyClass = (DetectedObjectPacket) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if (!this.pose_.equals(otherMyClass.pose_)) return false;

      if(this.id_ != otherMyClass.id_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("DetectedObjectPacket {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("pose=");
      builder.append(this.pose_);      builder.append(", ");

      builder.append("id=");
      builder.append(this.id_);
      builder.append("}");
      return builder.toString();
   }
}
