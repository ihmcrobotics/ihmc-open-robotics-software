package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC quadruped controller API.
       * This message contains the controller's estimate of the ground plane.
       * It is defined by an origin and a normal
       */
public class QuadrupedGroundPlaneMessage extends Packet<QuadrupedGroundPlaneMessage> implements Settable<QuadrupedGroundPlaneMessage>, EpsilonComparable<QuadrupedGroundPlaneMessage>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public us.ihmc.euclid.tuple3D.Point3D region_origin_;

   public us.ihmc.euclid.tuple3D.Vector3D region_normal_;

   public QuadrupedGroundPlaneMessage()
   {


      region_origin_ = new us.ihmc.euclid.tuple3D.Point3D();

      region_normal_ = new us.ihmc.euclid.tuple3D.Vector3D();

   }

   public QuadrupedGroundPlaneMessage(QuadrupedGroundPlaneMessage other)
   {
      this();
      set(other);
   }

   public void set(QuadrupedGroundPlaneMessage other)
   {

      sequence_id_ = other.sequence_id_;


      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.region_origin_, region_origin_);

      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.region_normal_, region_normal_);
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



   public us.ihmc.euclid.tuple3D.Point3D getRegionOrigin()
   {
      return region_origin_;
   }



   public us.ihmc.euclid.tuple3D.Vector3D getRegionNormal()
   {
      return region_normal_;
   }


   public static Supplier<QuadrupedGroundPlaneMessagePubSubType> getPubSubType()
   {
      return QuadrupedGroundPlaneMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return QuadrupedGroundPlaneMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(QuadrupedGroundPlaneMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!this.region_origin_.epsilonEquals(other.region_origin_, epsilon)) return false;

      if (!this.region_normal_.epsilonEquals(other.region_normal_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof QuadrupedGroundPlaneMessage)) return false;

      QuadrupedGroundPlaneMessage otherMyClass = (QuadrupedGroundPlaneMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if (!this.region_origin_.equals(otherMyClass.region_origin_)) return false;

      if (!this.region_normal_.equals(otherMyClass.region_normal_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("QuadrupedGroundPlaneMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("region_origin=");
      builder.append(this.region_origin_);      builder.append(", ");

      builder.append("region_normal=");
      builder.append(this.region_normal_);
      builder.append("}");
      return builder.toString();
   }
}
