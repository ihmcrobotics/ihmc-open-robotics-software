package ihmc_common_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This is part of the IHMC Common message package.
       */
public class ConvexPolytope3DMessage extends Packet<ConvexPolytope3DMessage> implements Settable<ConvexPolytope3DMessage>, EpsilonComparable<ConvexPolytope3DMessage>
{
   /**
            * Vertices of the polytope, see ConvexPolytope3D.
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  vertices_;

   public ConvexPolytope3DMessage()
   {
      vertices_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D> (50, new geometry_msgs.msg.dds.PointPubSubType());

   }

   public ConvexPolytope3DMessage(ConvexPolytope3DMessage other)
   {
      this();
      set(other);
   }

   public void set(ConvexPolytope3DMessage other)
   {
      vertices_.set(other.vertices_);
   }


   /**
            * Vertices of the polytope, see ConvexPolytope3D.
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  getVertices()
   {
      return vertices_;
   }


   public static Supplier<ConvexPolytope3DMessagePubSubType> getPubSubType()
   {
      return ConvexPolytope3DMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ConvexPolytope3DMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ConvexPolytope3DMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (this.vertices_.size() != other.vertices_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.vertices_.size(); i++)
         {  if (!this.vertices_.get(i).epsilonEquals(other.vertices_.get(i), epsilon)) return false; }
      }

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ConvexPolytope3DMessage)) return false;

      ConvexPolytope3DMessage otherMyClass = (ConvexPolytope3DMessage) other;

      if (!this.vertices_.equals(otherMyClass.vertices_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ConvexPolytope3DMessage {");
      builder.append("vertices=");
      builder.append(this.vertices_);
      builder.append("}");
      return builder.toString();
   }
}
