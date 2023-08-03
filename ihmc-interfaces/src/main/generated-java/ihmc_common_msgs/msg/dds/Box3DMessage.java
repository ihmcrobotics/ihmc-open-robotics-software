package ihmc_common_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This is part of the IHMC Common message package.
       */
public class Box3DMessage extends Packet<Box3DMessage> implements Settable<Box3DMessage>, EpsilonComparable<Box3DMessage>
{
   /**
            * Size of the box, see Box3D.
            */
   public us.ihmc.euclid.tuple3D.Vector3D size_;
   /**
            * Pose of the box
            */
   public us.ihmc.euclid.geometry.Pose3D pose_;

   public Box3DMessage()
   {
      size_ = new us.ihmc.euclid.tuple3D.Vector3D();
      pose_ = new us.ihmc.euclid.geometry.Pose3D();
   }

   public Box3DMessage(Box3DMessage other)
   {
      this();
      set(other);
   }

   public void set(Box3DMessage other)
   {
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.size_, size_);
      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.pose_, pose_);
   }


   /**
            * Size of the box, see Box3D.
            */
   public us.ihmc.euclid.tuple3D.Vector3D getSize()
   {
      return size_;
   }


   /**
            * Pose of the box
            */
   public us.ihmc.euclid.geometry.Pose3D getPose()
   {
      return pose_;
   }


   public static Supplier<Box3DMessagePubSubType> getPubSubType()
   {
      return Box3DMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return Box3DMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(Box3DMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.size_.epsilonEquals(other.size_, epsilon)) return false;
      if (!this.pose_.epsilonEquals(other.pose_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof Box3DMessage)) return false;

      Box3DMessage otherMyClass = (Box3DMessage) other;

      if (!this.size_.equals(otherMyClass.size_)) return false;
      if (!this.pose_.equals(otherMyClass.pose_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("Box3DMessage {");
      builder.append("size=");
      builder.append(this.size_);      builder.append(", ");
      builder.append("pose=");
      builder.append(this.pose_);
      builder.append("}");
      return builder.toString();
   }
}
