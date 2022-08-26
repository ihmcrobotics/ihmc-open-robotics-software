package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC fast walking controller API.
       * This message sets the desired walking and turning speed.
       * This message may initiate or stop walking.
       */
public class DesiredWalkingSpeedPacket extends Packet<DesiredWalkingSpeedPacket> implements Settable<DesiredWalkingSpeedPacket>, EpsilonComparable<DesiredWalkingSpeedPacket>
{
   /**
            * Define the desired 3D linear velocity to be reached.
            */
   public us.ihmc.euclid.tuple3D.Vector3D desired_speed_;
   /**
            * Define the desired turning velocity
            */
   public double turning_speed_;

   public DesiredWalkingSpeedPacket()
   {
      desired_speed_ = new us.ihmc.euclid.tuple3D.Vector3D();
   }

   public DesiredWalkingSpeedPacket(DesiredWalkingSpeedPacket other)
   {
      this();
      set(other);
   }

   public void set(DesiredWalkingSpeedPacket other)
   {
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.desired_speed_, desired_speed_);
      turning_speed_ = other.turning_speed_;

   }


   /**
            * Define the desired 3D linear velocity to be reached.
            */
   public us.ihmc.euclid.tuple3D.Vector3D getDesiredSpeed()
   {
      return desired_speed_;
   }

   /**
            * Define the desired turning velocity
            */
   public void setTurningSpeed(double turning_speed)
   {
      turning_speed_ = turning_speed;
   }
   /**
            * Define the desired turning velocity
            */
   public double getTurningSpeed()
   {
      return turning_speed_;
   }


   public static Supplier<DesiredWalkingSpeedPacketPubSubType> getPubSubType()
   {
      return DesiredWalkingSpeedPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return DesiredWalkingSpeedPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(DesiredWalkingSpeedPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.desired_speed_.epsilonEquals(other.desired_speed_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.turning_speed_, other.turning_speed_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof DesiredWalkingSpeedPacket)) return false;

      DesiredWalkingSpeedPacket otherMyClass = (DesiredWalkingSpeedPacket) other;

      if (!this.desired_speed_.equals(otherMyClass.desired_speed_)) return false;
      if(this.turning_speed_ != otherMyClass.turning_speed_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("DesiredWalkingSpeedPacket {");
      builder.append("desired_speed=");
      builder.append(this.desired_speed_);      builder.append(", ");
      builder.append("turning_speed=");
      builder.append(this.turning_speed_);
      builder.append("}");
      return builder.toString();
   }
}
