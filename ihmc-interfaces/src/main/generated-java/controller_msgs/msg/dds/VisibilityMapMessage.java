package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC footstep planning module.
       */
public class VisibilityMapMessage extends Packet<VisibilityMapMessage> implements Settable<VisibilityMapMessage>, EpsilonComparable<VisibilityMapMessage>
{
   /**
            * unique id of the map
            */
   public long map_id_;
   /**
            * start point for the connection
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  source_points_;
   /**
            * target point for the connection
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  target_points_;

   public VisibilityMapMessage()
   {
      source_points_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D> (100, new geometry_msgs.msg.dds.PointPubSubType());
      target_points_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D> (100, new geometry_msgs.msg.dds.PointPubSubType());

   }

   public VisibilityMapMessage(VisibilityMapMessage other)
   {
      this();
      set(other);
   }

   public void set(VisibilityMapMessage other)
   {
      map_id_ = other.map_id_;

      source_points_.set(other.source_points_);
      target_points_.set(other.target_points_);
   }

   /**
            * unique id of the map
            */
   public void setMapId(long map_id)
   {
      map_id_ = map_id;
   }
   /**
            * unique id of the map
            */
   public long getMapId()
   {
      return map_id_;
   }


   /**
            * start point for the connection
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  getSourcePoints()
   {
      return source_points_;
   }


   /**
            * target point for the connection
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  getTargetPoints()
   {
      return target_points_;
   }


   public static Supplier<VisibilityMapMessagePubSubType> getPubSubType()
   {
      return VisibilityMapMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return VisibilityMapMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(VisibilityMapMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.map_id_, other.map_id_, epsilon)) return false;

      if (this.source_points_.size() != other.source_points_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.source_points_.size(); i++)
         {  if (!this.source_points_.get(i).epsilonEquals(other.source_points_.get(i), epsilon)) return false; }
      }

      if (this.target_points_.size() != other.target_points_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.target_points_.size(); i++)
         {  if (!this.target_points_.get(i).epsilonEquals(other.target_points_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof VisibilityMapMessage)) return false;

      VisibilityMapMessage otherMyClass = (VisibilityMapMessage) other;

      if(this.map_id_ != otherMyClass.map_id_) return false;

      if (!this.source_points_.equals(otherMyClass.source_points_)) return false;
      if (!this.target_points_.equals(otherMyClass.target_points_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("VisibilityMapMessage {");
      builder.append("map_id=");
      builder.append(this.map_id_);      builder.append(", ");
      builder.append("source_points=");
      builder.append(this.source_points_);      builder.append(", ");
      builder.append("target_points=");
      builder.append(this.target_points_);
      builder.append("}");
      return builder.toString();
   }
}
