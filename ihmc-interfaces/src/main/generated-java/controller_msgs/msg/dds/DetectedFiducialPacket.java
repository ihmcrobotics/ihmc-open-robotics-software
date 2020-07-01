package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC humanoid behavior module.
       */
public class DetectedFiducialPacket extends Packet<DetectedFiducialPacket> implements Settable<DetectedFiducialPacket>, EpsilonComparable<DetectedFiducialPacket>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public long fiducial_id_;

   public us.ihmc.euclid.geometry.Pose3D fiducial_transform_to_world_;

   public DetectedFiducialPacket()
   {



      fiducial_transform_to_world_ = new us.ihmc.euclid.geometry.Pose3D();

   }

   public DetectedFiducialPacket(DetectedFiducialPacket other)
   {
      this();
      set(other);
   }

   public void set(DetectedFiducialPacket other)
   {

      sequence_id_ = other.sequence_id_;


      fiducial_id_ = other.fiducial_id_;


      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.fiducial_transform_to_world_, fiducial_transform_to_world_);
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


   public void setFiducialId(long fiducial_id)
   {
      fiducial_id_ = fiducial_id;
   }
   public long getFiducialId()
   {
      return fiducial_id_;
   }



   public us.ihmc.euclid.geometry.Pose3D getFiducialTransformToWorld()
   {
      return fiducial_transform_to_world_;
   }


   public static Supplier<DetectedFiducialPacketPubSubType> getPubSubType()
   {
      return DetectedFiducialPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return DetectedFiducialPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(DetectedFiducialPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.fiducial_id_, other.fiducial_id_, epsilon)) return false;


      if (!this.fiducial_transform_to_world_.epsilonEquals(other.fiducial_transform_to_world_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof DetectedFiducialPacket)) return false;

      DetectedFiducialPacket otherMyClass = (DetectedFiducialPacket) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.fiducial_id_ != otherMyClass.fiducial_id_) return false;


      if (!this.fiducial_transform_to_world_.equals(otherMyClass.fiducial_transform_to_world_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("DetectedFiducialPacket {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("fiducial_id=");
      builder.append(this.fiducial_id_);      builder.append(", ");

      builder.append("fiducial_transform_to_world=");
      builder.append(this.fiducial_transform_to_world_);
      builder.append("}");
      return builder.toString();
   }
}
