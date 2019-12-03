package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC footstep planning module.
       */
public class VisibilityClusterPointsMessage extends Packet<VisibilityClusterPointsMessage> implements Settable<VisibilityClusterPointsMessage>, EpsilonComparable<VisibilityClusterPointsMessage>
{
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  points_;

   public VisibilityClusterPointsMessage()
   {
      points_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D> (25, new geometry_msgs.msg.dds.PointPubSubType());

   }

   public VisibilityClusterPointsMessage(VisibilityClusterPointsMessage other)
   {
      this();
      set(other);
   }

   public void set(VisibilityClusterPointsMessage other)
   {
      points_.set(other.points_);
   }


   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  getPoints()
   {
      return points_;
   }


   public static Supplier<VisibilityClusterPointsMessagePubSubType> getPubSubType()
   {
      return VisibilityClusterPointsMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return VisibilityClusterPointsMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(VisibilityClusterPointsMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (this.points_.size() != other.points_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.points_.size(); i++)
         {  if (!this.points_.get(i).epsilonEquals(other.points_.get(i), epsilon)) return false; }
      }

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof VisibilityClusterPointsMessage)) return false;

      VisibilityClusterPointsMessage otherMyClass = (VisibilityClusterPointsMessage) other;

      if (!this.points_.equals(otherMyClass.points_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("VisibilityClusterPointsMessage {");
      builder.append("points=");
      builder.append(this.points_);
      builder.append("}");
      return builder.toString();
   }
}
