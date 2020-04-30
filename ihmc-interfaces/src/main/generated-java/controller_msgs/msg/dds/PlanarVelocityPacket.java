package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC quadruped controller API.
       */
public class PlanarVelocityPacket extends Packet<PlanarVelocityPacket> implements Settable<PlanarVelocityPacket>, EpsilonComparable<PlanarVelocityPacket>
{

   public us.ihmc.euclid.tuple3D.Vector3D velocity_;

   public PlanarVelocityPacket()
   {

      velocity_ = new us.ihmc.euclid.tuple3D.Vector3D();

   }

   public PlanarVelocityPacket(PlanarVelocityPacket other)
   {
      this();
      set(other);
   }

   public void set(PlanarVelocityPacket other)
   {

      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.velocity_, velocity_);
   }



   public us.ihmc.euclid.tuple3D.Vector3D getVelocity()
   {
      return velocity_;
   }


   public static Supplier<PlanarVelocityPacketPubSubType> getPubSubType()
   {
      return PlanarVelocityPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return PlanarVelocityPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(PlanarVelocityPacket other, double epsilon)
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
      if(!(other instanceof PlanarVelocityPacket)) return false;

      PlanarVelocityPacket otherMyClass = (PlanarVelocityPacket) other;


      if (!this.velocity_.equals(otherMyClass.velocity_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("PlanarVelocityPacket {");

      builder.append("velocity=");
      builder.append(this.velocity_);
      builder.append("}");
      return builder.toString();
   }
}
