package ihmc_common_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This is part of the IHMC Common message package.
       */
public class Capsule3DMessage extends Packet<Capsule3DMessage> implements Settable<Capsule3DMessage>, EpsilonComparable<Capsule3DMessage>
{
   /**
            * Position of the capsule, see Capsule3D.
            */
   public us.ihmc.euclid.tuple3D.Point3D position_;
   /**
            * Axis of the capsule, see Capsule3D.
            */
   public us.ihmc.euclid.tuple3D.Vector3D axis_;
   /**
            * Radius of the capsule, see Capsule3D.
            */
   public double radius_;
   /**
            * Length of the capsule, see Capsule3D.
            */
   public double length_;

   public Capsule3DMessage()
   {
      position_ = new us.ihmc.euclid.tuple3D.Point3D();
      axis_ = new us.ihmc.euclid.tuple3D.Vector3D();
   }

   public Capsule3DMessage(Capsule3DMessage other)
   {
      this();
      set(other);
   }

   public void set(Capsule3DMessage other)
   {
      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.position_, position_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.axis_, axis_);
      radius_ = other.radius_;

      length_ = other.length_;

   }


   /**
            * Position of the capsule, see Capsule3D.
            */
   public us.ihmc.euclid.tuple3D.Point3D getPosition()
   {
      return position_;
   }


   /**
            * Axis of the capsule, see Capsule3D.
            */
   public us.ihmc.euclid.tuple3D.Vector3D getAxis()
   {
      return axis_;
   }

   /**
            * Radius of the capsule, see Capsule3D.
            */
   public void setRadius(double radius)
   {
      radius_ = radius;
   }
   /**
            * Radius of the capsule, see Capsule3D.
            */
   public double getRadius()
   {
      return radius_;
   }

   /**
            * Length of the capsule, see Capsule3D.
            */
   public void setLength(double length)
   {
      length_ = length;
   }
   /**
            * Length of the capsule, see Capsule3D.
            */
   public double getLength()
   {
      return length_;
   }


   public static Supplier<Capsule3DMessagePubSubType> getPubSubType()
   {
      return Capsule3DMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return Capsule3DMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(Capsule3DMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.position_.epsilonEquals(other.position_, epsilon)) return false;
      if (!this.axis_.epsilonEquals(other.axis_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.radius_, other.radius_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.length_, other.length_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof Capsule3DMessage)) return false;

      Capsule3DMessage otherMyClass = (Capsule3DMessage) other;

      if (!this.position_.equals(otherMyClass.position_)) return false;
      if (!this.axis_.equals(otherMyClass.axis_)) return false;
      if(this.radius_ != otherMyClass.radius_) return false;

      if(this.length_ != otherMyClass.length_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("Capsule3DMessage {");
      builder.append("position=");
      builder.append(this.position_);      builder.append(", ");
      builder.append("axis=");
      builder.append(this.axis_);      builder.append(", ");
      builder.append("radius=");
      builder.append(this.radius_);      builder.append(", ");
      builder.append("length=");
      builder.append(this.length_);
      builder.append("}");
      return builder.toString();
   }
}
