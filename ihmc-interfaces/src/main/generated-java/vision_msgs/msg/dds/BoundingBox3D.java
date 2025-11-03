package vision_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * A 3D bounding box that can be positioned and rotated about its center (6 DOF)
       * Dimensions of this box are in meters, and as such, it may be migrated to
       * another package, such as geometry_msgs, in the future.
       */
public class BoundingBox3D extends Packet<BoundingBox3D> implements Settable<BoundingBox3D>, EpsilonComparable<BoundingBox3D>
{
   /**
            * The 3D position and orientation of the bounding box center
            */
   public us.ihmc.euclid.geometry.Pose3D center_;
   /**
            * The total size of the bounding box, in meters, surrounding the object's center
            * pose.
            */
   public us.ihmc.euclid.tuple3D.Vector3D size_;

   public BoundingBox3D()
   {
      center_ = new us.ihmc.euclid.geometry.Pose3D();
      size_ = new us.ihmc.euclid.tuple3D.Vector3D();
   }

   public BoundingBox3D(BoundingBox3D other)
   {
      this();
      set(other);
   }

   public void set(BoundingBox3D other)
   {
      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.center_, center_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.size_, size_);
   }


   /**
            * The 3D position and orientation of the bounding box center
            */
   public us.ihmc.euclid.geometry.Pose3D getCenter()
   {
      return center_;
   }


   /**
            * The total size of the bounding box, in meters, surrounding the object's center
            * pose.
            */
   public us.ihmc.euclid.tuple3D.Vector3D getSize()
   {
      return size_;
   }


   public static Supplier<BoundingBox3DPubSubType> getPubSubType()
   {
      return BoundingBox3DPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return BoundingBox3DPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(BoundingBox3D other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.center_.epsilonEquals(other.center_, epsilon)) return false;
      if (!this.size_.epsilonEquals(other.size_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof BoundingBox3D)) return false;

      BoundingBox3D otherMyClass = (BoundingBox3D) other;

      if (!this.center_.equals(otherMyClass.center_)) return false;
      if (!this.size_.equals(otherMyClass.size_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BoundingBox3D {");
      builder.append("center=");
      builder.append(this.center_);      builder.append(", ");
      builder.append("size=");
      builder.append(this.size_);
      builder.append("}");
      return builder.toString();
   }
}
