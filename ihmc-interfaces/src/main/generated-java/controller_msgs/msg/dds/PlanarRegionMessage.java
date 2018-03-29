package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * This message is part of the IHMC robot environment awareness module.
 * This message contains a single planar region.
 * It is defined by an origin, a normal, and a concave hull.
 * For easier processing, a set of convex polygons approximating the concave hull are also provided.
 */
public class PlanarRegionMessage extends Packet<PlanarRegionMessage> implements Settable<PlanarRegionMessage>, EpsilonComparable<PlanarRegionMessage>
{
   public static final int NO_REGION_ID = -1;
   public int region_id_ = -1;
   public us.ihmc.euclid.tuple3D.Point3D region_origin_;
   public us.ihmc.euclid.tuple3D.Vector3D region_normal_;
   public controller_msgs.msg.dds.Polygon2DMessage concave_hull_;
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.Polygon2DMessage> convex_polygons_;

   public PlanarRegionMessage()
   {

      region_origin_ = new us.ihmc.euclid.tuple3D.Point3D();
      region_normal_ = new us.ihmc.euclid.tuple3D.Vector3D();
      concave_hull_ = new controller_msgs.msg.dds.Polygon2DMessage();
      convex_polygons_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.Polygon2DMessage>(100, controller_msgs.msg.dds.Polygon2DMessage.class,
                                                                                                      new controller_msgs.msg.dds.Polygon2DMessagePubSubType());
   }

   public PlanarRegionMessage(PlanarRegionMessage other)
   {
      set(other);
   }

   public void set(PlanarRegionMessage other)
   {
      region_id_ = other.region_id_;

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.region_origin_, region_origin_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.region_normal_, region_normal_);
      controller_msgs.msg.dds.Polygon2DMessagePubSubType.staticCopy(other.concave_hull_, concave_hull_);
      convex_polygons_.set(other.convex_polygons_);
   }

   public int getRegionId()
   {
      return region_id_;
   }

   public void setRegionId(int region_id)
   {
      region_id_ = region_id;
   }

   public us.ihmc.euclid.tuple3D.Point3D getRegionOrigin()
   {
      return region_origin_;
   }

   public us.ihmc.euclid.tuple3D.Vector3D getRegionNormal()
   {
      return region_normal_;
   }

   public controller_msgs.msg.dds.Polygon2DMessage getConcaveHull()
   {
      return concave_hull_;
   }

   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.Polygon2DMessage> getConvexPolygons()
   {
      return convex_polygons_;
   }

   @Override
   public boolean epsilonEquals(PlanarRegionMessage other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.region_id_, other.region_id_, epsilon))
         return false;

      if (!this.region_origin_.epsilonEquals(other.region_origin_, epsilon))
         return false;

      if (!this.region_normal_.epsilonEquals(other.region_normal_, epsilon))
         return false;

      if (!this.concave_hull_.epsilonEquals(other.concave_hull_, epsilon))
         return false;

      if (this.convex_polygons_.size() == other.convex_polygons_.size())
      {
         return false;
      }
      else
      {
         for (int i = 0; i < this.convex_polygons_.size(); i++)
         {
            if (!this.convex_polygons_.get(i).epsilonEquals(other.convex_polygons_.get(i), epsilon))
               return false;
         }
      }

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;
      if (!(other instanceof PlanarRegionMessage))
         return false;

      PlanarRegionMessage otherMyClass = (PlanarRegionMessage) other;

      if (this.region_id_ != otherMyClass.region_id_)
         return false;

      if (!this.region_origin_.equals(otherMyClass.region_origin_))
         return false;

      if (!this.region_normal_.equals(otherMyClass.region_normal_))
         return false;

      if (!this.concave_hull_.equals(otherMyClass.concave_hull_))
         return false;

      if (!this.convex_polygons_.equals(otherMyClass.convex_polygons_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("PlanarRegionMessage {");
      builder.append("region_id=");
      builder.append(this.region_id_);

      builder.append(", ");
      builder.append("region_origin=");
      builder.append(this.region_origin_);

      builder.append(", ");
      builder.append("region_normal=");
      builder.append(this.region_normal_);

      builder.append(", ");
      builder.append("concave_hull=");
      builder.append(this.concave_hull_);

      builder.append(", ");
      builder.append("convex_polygons=");
      builder.append(this.convex_polygons_);

      builder.append("}");
      return builder.toString();
   }
}
