package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

/**
 * General purpose message that contains vertices to represent a 2D polygon (z ignored).
 */
public class Polygon2DMessage extends Packet<Polygon2DMessage> implements Settable<Polygon2DMessage>, EpsilonComparable<Polygon2DMessage>
{
   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header header_;
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D> vertices_;

   public Polygon2DMessage()
   {
      header_ = new std_msgs.msg.dds.Header();
      vertices_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>(100, us.ihmc.euclid.tuple3D.Point3D.class,
                                                                                     new geometry_msgs.msg.dds.PointPubSubType());

   }

   public Polygon2DMessage(Polygon2DMessage other)
   {
      this();
      set(other);
   }

   public void set(Polygon2DMessage other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      vertices_.set(other.vertices_);
   }

   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }

   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D> getVertices()
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

      if (!this.header_.epsilonEquals(other.header_, epsilon))
         return false;
      if (this.vertices_.size() == other.vertices_.size())
      {
         return false;
      }
      else
      {
         for (int i = 0; i < this.vertices_.size(); i++)
         {
            if (!this.vertices_.get(i).epsilonEquals(other.vertices_.get(i), epsilon))
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
      if (!(other instanceof Polygon2DMessage))
         return false;

      Polygon2DMessage otherMyClass = (Polygon2DMessage) other;

      if (!this.header_.equals(otherMyClass.header_))
         return false;
      if (!this.vertices_.equals(otherMyClass.vertices_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("Polygon2DMessage {");
      builder.append("header=");
      builder.append(this.header_);
      builder.append(", ");
      builder.append("vertices=");
      builder.append(this.vertices_);
      builder.append("}");
      return builder.toString();
   }
}
