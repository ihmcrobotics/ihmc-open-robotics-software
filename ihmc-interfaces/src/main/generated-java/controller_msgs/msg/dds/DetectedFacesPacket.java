package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

public class DetectedFacesPacket extends Packet<DetectedFacesPacket> implements Settable<DetectedFacesPacket>, EpsilonComparable<DetectedFacesPacket>
{
   public us.ihmc.idl.IDLSequence.StringBuilderHolder ids_;
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D> positions_;

   public DetectedFacesPacket()
   {
      ids_ = new us.ihmc.idl.IDLSequence.StringBuilderHolder(100, "type_d");
      positions_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>(100, us.ihmc.euclid.tuple3D.Point3D.class,
                                                                                      new geometry_msgs.msg.dds.PointPubSubType());
   }

   public DetectedFacesPacket(DetectedFacesPacket other)
   {
      set(other);
   }

   public void set(DetectedFacesPacket other)
   {
      ids_.set(other.ids_);
      positions_.set(other.positions_);
   }

   public us.ihmc.idl.IDLSequence.StringBuilderHolder getIds()
   {
      return ids_;
   }

   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D> getPositions()
   {
      return positions_;
   }

   @Override
   public boolean epsilonEquals(DetectedFacesPacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilderSequence(this.ids_, other.ids_, epsilon))
         return false;

      if (this.positions_.size() == other.positions_.size())
      {
         return false;
      }
      else
      {
         for (int i = 0; i < this.positions_.size(); i++)
         {
            if (!this.positions_.get(i).epsilonEquals(other.positions_.get(i), epsilon))
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
      if (!(other instanceof DetectedFacesPacket))
         return false;

      DetectedFacesPacket otherMyClass = (DetectedFacesPacket) other;

      if (!this.ids_.equals(otherMyClass.ids_))
         return false;

      if (!this.positions_.equals(otherMyClass.positions_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("DetectedFacesPacket {");
      builder.append("ids=");
      builder.append(this.ids_);

      builder.append(", ");
      builder.append("positions=");
      builder.append(this.positions_);

      builder.append("}");
      return builder.toString();
   }
}