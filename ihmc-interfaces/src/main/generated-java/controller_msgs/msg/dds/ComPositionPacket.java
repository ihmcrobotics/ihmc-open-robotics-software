package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC quadruped controller API.
       */
public class ComPositionPacket extends Packet<ComPositionPacket> implements Settable<ComPositionPacket>, EpsilonComparable<ComPositionPacket>
{

   public us.ihmc.euclid.tuple3D.Point3D position_;

   public ComPositionPacket()
   {

      position_ = new us.ihmc.euclid.tuple3D.Point3D();

   }

   public ComPositionPacket(ComPositionPacket other)
   {
      this();
      set(other);
   }

   public void set(ComPositionPacket other)
   {

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.position_, position_);
   }



   public us.ihmc.euclid.tuple3D.Point3D getPosition()
   {
      return position_;
   }


   public static Supplier<ComPositionPacketPubSubType> getPubSubType()
   {
      return ComPositionPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ComPositionPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ComPositionPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!this.position_.epsilonEquals(other.position_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ComPositionPacket)) return false;

      ComPositionPacket otherMyClass = (ComPositionPacket) other;


      if (!this.position_.equals(otherMyClass.position_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ComPositionPacket {");

      builder.append("position=");
      builder.append(this.position_);
      builder.append("}");
      return builder.toString();
   }
}
