package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC quadruped controller API.
       */
public class ComVelocityPacket extends Packet<ComVelocityPacket> implements Settable<ComVelocityPacket>, EpsilonComparable<ComVelocityPacket>
{

   public us.ihmc.euclid.tuple3D.Vector3D velocity_;

   public ComVelocityPacket()
   {

      velocity_ = new us.ihmc.euclid.tuple3D.Vector3D();

   }

   public ComVelocityPacket(ComVelocityPacket other)
   {
      this();
      set(other);
   }

   public void set(ComVelocityPacket other)
   {

      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.velocity_, velocity_);
   }



   public us.ihmc.euclid.tuple3D.Vector3D getVelocity()
   {
      return velocity_;
   }


   public static Supplier<ComVelocityPacketPubSubType> getPubSubType()
   {
      return ComVelocityPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ComVelocityPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ComVelocityPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!this.velocity_.epsilonEquals(other.velocity_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ComVelocityPacket)) return false;

      ComVelocityPacket otherMyClass = (ComVelocityPacket) other;


      if (!this.velocity_.equals(otherMyClass.velocity_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ComVelocityPacket {");

      builder.append("velocity=");
      builder.append(this.velocity_);
      builder.append("}");
      return builder.toString();
   }
}
