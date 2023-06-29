package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC robot environment awareness module.
       * This message contains a list of constraint regions.
       * A constraint region is a finite area that lies on a 3D plane and that is delimited by a concave hull.
       * As concave hulls are complex to manipulate, a set of convex polygons are also provided, altogether they approximate the area of the constraint region.
       */
public class StepConstraintsListMessage extends Packet<StepConstraintsListMessage> implements Settable<StepConstraintsListMessage>, EpsilonComparable<StepConstraintsListMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * The coordinates in world of the origin of each constraint region.
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  region_origin_;
   /**
            * The orientation with respect to world of each constraint region.
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple4D.Quaternion>  region_orientation_;
   /**
            * The normal expressed in world of each constraint region.
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Vector3D>  region_normal_;
   /**
            * Indicates for each constraint region the number of vertices for the concave hull.
            */
   public us.ihmc.idl.IDLSequence.Integer  concave_hulls_size_;
   /**
            * Indicates for each constraint region the number of holes contained in that region.
            */
   public us.ihmc.idl.IDLSequence.Integer  number_of_holes_in_region_;
   /**
            * Indicates successively the number of vertices for each hole of each constraint region.
            */
   public us.ihmc.idl.IDLSequence.Integer  hole_polygons_size_;
   /**
            * Single buffer used to store for each constraint the vertices of the concave hull and the convex polygons.
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  vertex_buffer_;

   public StepConstraintsListMessage()
   {
      region_origin_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D> (20, new geometry_msgs.msg.dds.PointPubSubType());
      region_orientation_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple4D.Quaternion> (20, new geometry_msgs.msg.dds.QuaternionPubSubType());
      region_normal_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Vector3D> (20, new geometry_msgs.msg.dds.Vector3PubSubType());
      concave_hulls_size_ = new us.ihmc.idl.IDLSequence.Integer (100, "type_2");

      number_of_holes_in_region_ = new us.ihmc.idl.IDLSequence.Integer (100, "type_2");

      hole_polygons_size_ = new us.ihmc.idl.IDLSequence.Integer (100, "type_2");

      vertex_buffer_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D> (50000, new geometry_msgs.msg.dds.PointPubSubType());

   }

   public StepConstraintsListMessage(StepConstraintsListMessage other)
   {
      this();
      set(other);
   }

   public void set(StepConstraintsListMessage other)
   {
      sequence_id_ = other.sequence_id_;

      region_origin_.set(other.region_origin_);
      region_orientation_.set(other.region_orientation_);
      region_normal_.set(other.region_normal_);
      concave_hulls_size_.set(other.concave_hulls_size_);
      number_of_holes_in_region_.set(other.number_of_holes_in_region_);
      hole_polygons_size_.set(other.hole_polygons_size_);
      vertex_buffer_.set(other.vertex_buffer_);
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
            * The coordinates in world of the origin of each constraint region.
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  getRegionOrigin()
   {
      return region_origin_;
   }


   /**
            * The orientation with respect to world of each constraint region.
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple4D.Quaternion>  getRegionOrientation()
   {
      return region_orientation_;
   }


   /**
            * The normal expressed in world of each constraint region.
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Vector3D>  getRegionNormal()
   {
      return region_normal_;
   }


   /**
            * Indicates for each constraint region the number of vertices for the concave hull.
            */
   public us.ihmc.idl.IDLSequence.Integer  getConcaveHullsSize()
   {
      return concave_hulls_size_;
   }


   /**
            * Indicates for each constraint region the number of holes contained in that region.
            */
   public us.ihmc.idl.IDLSequence.Integer  getNumberOfHolesInRegion()
   {
      return number_of_holes_in_region_;
   }


   /**
            * Indicates successively the number of vertices for each hole of each constraint region.
            */
   public us.ihmc.idl.IDLSequence.Integer  getHolePolygonsSize()
   {
      return hole_polygons_size_;
   }


   /**
            * Single buffer used to store for each constraint the vertices of the concave hull and the convex polygons.
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  getVertexBuffer()
   {
      return vertex_buffer_;
   }


   public static Supplier<StepConstraintsListMessagePubSubType> getPubSubType()
   {
      return StepConstraintsListMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return StepConstraintsListMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(StepConstraintsListMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (this.region_origin_.size() != other.region_origin_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.region_origin_.size(); i++)
         {  if (!this.region_origin_.get(i).epsilonEquals(other.region_origin_.get(i), epsilon)) return false; }
      }

      if (this.region_orientation_.size() != other.region_orientation_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.region_orientation_.size(); i++)
         {  if (!this.region_orientation_.get(i).epsilonEquals(other.region_orientation_.get(i), epsilon)) return false; }
      }

      if (this.region_normal_.size() != other.region_normal_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.region_normal_.size(); i++)
         {  if (!this.region_normal_.get(i).epsilonEquals(other.region_normal_.get(i), epsilon)) return false; }
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsIntegerSequence(this.concave_hulls_size_, other.concave_hulls_size_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsIntegerSequence(this.number_of_holes_in_region_, other.number_of_holes_in_region_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsIntegerSequence(this.hole_polygons_size_, other.hole_polygons_size_, epsilon)) return false;

      if (this.vertex_buffer_.size() != other.vertex_buffer_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.vertex_buffer_.size(); i++)
         {  if (!this.vertex_buffer_.get(i).epsilonEquals(other.vertex_buffer_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof StepConstraintsListMessage)) return false;

      StepConstraintsListMessage otherMyClass = (StepConstraintsListMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if (!this.region_origin_.equals(otherMyClass.region_origin_)) return false;
      if (!this.region_orientation_.equals(otherMyClass.region_orientation_)) return false;
      if (!this.region_normal_.equals(otherMyClass.region_normal_)) return false;
      if (!this.concave_hulls_size_.equals(otherMyClass.concave_hulls_size_)) return false;
      if (!this.number_of_holes_in_region_.equals(otherMyClass.number_of_holes_in_region_)) return false;
      if (!this.hole_polygons_size_.equals(otherMyClass.hole_polygons_size_)) return false;
      if (!this.vertex_buffer_.equals(otherMyClass.vertex_buffer_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("StepConstraintsListMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("region_origin=");
      builder.append(this.region_origin_);      builder.append(", ");
      builder.append("region_orientation=");
      builder.append(this.region_orientation_);      builder.append(", ");
      builder.append("region_normal=");
      builder.append(this.region_normal_);      builder.append(", ");
      builder.append("concave_hulls_size=");
      builder.append(this.concave_hulls_size_);      builder.append(", ");
      builder.append("number_of_holes_in_region=");
      builder.append(this.number_of_holes_in_region_);      builder.append(", ");
      builder.append("hole_polygons_size=");
      builder.append(this.hole_polygons_size_);      builder.append(", ");
      builder.append("vertex_buffer=");
      builder.append(this.vertex_buffer_);
      builder.append("}");
      return builder.toString();
   }
}
