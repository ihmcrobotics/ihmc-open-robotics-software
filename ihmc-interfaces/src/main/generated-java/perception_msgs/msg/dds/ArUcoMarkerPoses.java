package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class ArUcoMarkerPoses extends Packet<ArUcoMarkerPoses> implements Settable<ArUcoMarkerPoses>, EpsilonComparable<ArUcoMarkerPoses>
{
   /**
            * ID of arUco marker
            */
   public us.ihmc.idl.IDLSequence.Long  marker_id_;
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  position_;
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple4D.Quaternion>  orientation_;

   public ArUcoMarkerPoses()
   {
      marker_id_ = new us.ihmc.idl.IDLSequence.Long (100, "type_4");

      position_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D> (100, new geometry_msgs.msg.dds.PointPubSubType());
      orientation_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple4D.Quaternion> (100, new geometry_msgs.msg.dds.QuaternionPubSubType());

   }

   public ArUcoMarkerPoses(ArUcoMarkerPoses other)
   {
      this();
      set(other);
   }

   public void set(ArUcoMarkerPoses other)
   {
      marker_id_.set(other.marker_id_);
      position_.set(other.position_);
      orientation_.set(other.orientation_);
   }


   /**
            * ID of arUco marker
            */
   public us.ihmc.idl.IDLSequence.Long  getMarkerId()
   {
      return marker_id_;
   }


   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  getPosition()
   {
      return position_;
   }


   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple4D.Quaternion>  getOrientation()
   {
      return orientation_;
   }


   public static Supplier<ArUcoMarkerPosesPubSubType> getPubSubType()
   {
      return ArUcoMarkerPosesPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ArUcoMarkerPosesPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ArUcoMarkerPoses other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsLongSequence(this.marker_id_, other.marker_id_, epsilon)) return false;

      if (this.position_.size() != other.position_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.position_.size(); i++)
         {  if (!this.position_.get(i).epsilonEquals(other.position_.get(i), epsilon)) return false; }
      }

      if (this.orientation_.size() != other.orientation_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.orientation_.size(); i++)
         {  if (!this.orientation_.get(i).epsilonEquals(other.orientation_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ArUcoMarkerPoses)) return false;

      ArUcoMarkerPoses otherMyClass = (ArUcoMarkerPoses) other;

      if (!this.marker_id_.equals(otherMyClass.marker_id_)) return false;
      if (!this.position_.equals(otherMyClass.position_)) return false;
      if (!this.orientation_.equals(otherMyClass.orientation_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ArUcoMarkerPoses {");
      builder.append("marker_id=");
      builder.append(this.marker_id_);      builder.append(", ");
      builder.append("position=");
      builder.append(this.position_);      builder.append(", ");
      builder.append("orientation=");
      builder.append(this.orientation_);
      builder.append("}");
      return builder.toString();
   }
}
