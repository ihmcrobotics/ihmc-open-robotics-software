package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class StepConstraintMessage extends Packet<StepConstraintMessage> implements Settable<StepConstraintMessage>, EpsilonComparable<StepConstraintMessage>
{

   public us.ihmc.euclid.tuple3D.Point3D region_origin_;

   public us.ihmc.euclid.tuple4D.Quaternion region_orientation_;

   public us.ihmc.euclid.tuple3D.Vector3D region_normal_;

   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  vertex_buffer_;

   public int concave_hull_size_;

   public int number_of_holes_in_region_;

   public us.ihmc.idl.IDLSequence.Integer  hole_polygons_size_;

   public StepConstraintMessage()
   {

      region_origin_ = new us.ihmc.euclid.tuple3D.Point3D();

      region_orientation_ = new us.ihmc.euclid.tuple4D.Quaternion();

      region_normal_ = new us.ihmc.euclid.tuple3D.Vector3D();

      vertex_buffer_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D> (1000, new geometry_msgs.msg.dds.PointPubSubType());



      hole_polygons_size_ = new us.ihmc.idl.IDLSequence.Integer (20, "type_2");


   }

   public StepConstraintMessage(StepConstraintMessage other)
   {
      this();
      set(other);
   }

   public void set(StepConstraintMessage other)
   {

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.region_origin_, region_origin_);

      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.region_orientation_, region_orientation_);

      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.region_normal_, region_normal_);

      vertex_buffer_.set(other.vertex_buffer_);

      concave_hull_size_ = other.concave_hull_size_;


      number_of_holes_in_region_ = other.number_of_holes_in_region_;


      hole_polygons_size_.set(other.hole_polygons_size_);
   }



   public us.ihmc.euclid.tuple3D.Point3D getRegionOrigin()
   {
      return region_origin_;
   }



   public us.ihmc.euclid.tuple4D.Quaternion getRegionOrientation()
   {
      return region_orientation_;
   }



   public us.ihmc.euclid.tuple3D.Vector3D getRegionNormal()
   {
      return region_normal_;
   }



   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  getVertexBuffer()
   {
      return vertex_buffer_;
   }


   public void setConcaveHullSize(int concave_hull_size)
   {
      concave_hull_size_ = concave_hull_size;
   }
   public int getConcaveHullSize()
   {
      return concave_hull_size_;
   }


   public void setNumberOfHolesInRegion(int number_of_holes_in_region)
   {
      number_of_holes_in_region_ = number_of_holes_in_region;
   }
   public int getNumberOfHolesInRegion()
   {
      return number_of_holes_in_region_;
   }



   public us.ihmc.idl.IDLSequence.Integer  getHolePolygonsSize()
   {
      return hole_polygons_size_;
   }


   public static Supplier<StepConstraintMessagePubSubType> getPubSubType()
   {
      return StepConstraintMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return StepConstraintMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(StepConstraintMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!this.region_origin_.epsilonEquals(other.region_origin_, epsilon)) return false;

      if (!this.region_orientation_.epsilonEquals(other.region_orientation_, epsilon)) return false;

      if (!this.region_normal_.epsilonEquals(other.region_normal_, epsilon)) return false;

      if (this.vertex_buffer_.size() != other.vertex_buffer_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.vertex_buffer_.size(); i++)
         {  if (!this.vertex_buffer_.get(i).epsilonEquals(other.vertex_buffer_.get(i), epsilon)) return false; }
      }


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.concave_hull_size_, other.concave_hull_size_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.number_of_holes_in_region_, other.number_of_holes_in_region_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsIntegerSequence(this.hole_polygons_size_, other.hole_polygons_size_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof StepConstraintMessage)) return false;

      StepConstraintMessage otherMyClass = (StepConstraintMessage) other;


      if (!this.region_origin_.equals(otherMyClass.region_origin_)) return false;

      if (!this.region_orientation_.equals(otherMyClass.region_orientation_)) return false;

      if (!this.region_normal_.equals(otherMyClass.region_normal_)) return false;

      if (!this.vertex_buffer_.equals(otherMyClass.vertex_buffer_)) return false;

      if(this.concave_hull_size_ != otherMyClass.concave_hull_size_) return false;


      if(this.number_of_holes_in_region_ != otherMyClass.number_of_holes_in_region_) return false;


      if (!this.hole_polygons_size_.equals(otherMyClass.hole_polygons_size_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("StepConstraintMessage {");

      builder.append("region_origin=");
      builder.append(this.region_origin_);      builder.append(", ");

      builder.append("region_orientation=");
      builder.append(this.region_orientation_);      builder.append(", ");

      builder.append("region_normal=");
      builder.append(this.region_normal_);      builder.append(", ");

      builder.append("vertex_buffer=");
      builder.append(this.vertex_buffer_);      builder.append(", ");

      builder.append("concave_hull_size=");
      builder.append(this.concave_hull_size_);      builder.append(", ");

      builder.append("number_of_holes_in_region=");
      builder.append(this.number_of_holes_in_region_);      builder.append(", ");

      builder.append("hole_polygons_size=");
      builder.append(this.hole_polygons_size_);
      builder.append("}");
      return builder.toString();
   }
}
