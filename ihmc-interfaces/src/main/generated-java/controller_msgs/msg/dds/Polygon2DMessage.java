package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * General purpose message that contains vertices to represent a 2D polygon (z ignored).
 */
public class Polygon2DMessage extends Packet<Polygon2DMessage> implements Settable<Polygon2DMessage>, EpsilonComparable<Polygon2DMessage>
{
   public us.ihmc.euclid.tuple3D.Point3D vertices_;

   public Polygon2DMessage()
   {
      vertices_ = new us.ihmc.euclid.tuple3D.Point3D();
   }

   public Polygon2DMessage(Polygon2DMessage other)
   {
      set(other);
   }

   public void set(Polygon2DMessage other)
   {
      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.vertices_, vertices_);
   }

   public us.ihmc.euclid.tuple3D.Point3D getVertices()
   {
      return vertices_;
   }

   @Override
   public boolean epsilonEquals(Polygon2DMessage other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.vertices_.epsilonEquals(other.vertices_, epsilon))
         return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;
      if (!(other instanceof Polygon2DMessage))
         return false;

      Polygon2DMessage otherMyClass = (Polygon2DMessage) other;

      if (!this.vertices_.equals(otherMyClass.vertices_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("Polygon2DMessage {");
      builder.append("vertices=");
      builder.append(this.vertices_);

      builder.append("}");
      return builder.toString();
   }
}