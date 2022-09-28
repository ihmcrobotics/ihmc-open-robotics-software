package ihmc_common_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This is part of the IHMC Common message package and intended to be platform-independent, such as primitives and generic data.
       */
public class ConvexPolytope3DMessage extends Packet<ConvexPolytope3DMessage> implements Settable<ConvexPolytope3DMessage>, EpsilonComparable<ConvexPolytope3DMessage>
{
   /**
            * Dimensions of the ramp, see Ramp3D.
            */
   public us.ihmc.euclid.tuple3D.Vector3D size_;
   /**
            * Pose of the ramp
            */
   public us.ihmc.euclid.geometry.Pose3D pose_;

   public ConvexPolytope3DMessage()
   {
      size_ = new us.ihmc.euclid.tuple3D.Vector3D();
      pose_ = new us.ihmc.euclid.geometry.Pose3D();
   }

   public ConvexPolytope3DMessage(ConvexPolytope3DMessage other)
   {
      this();
      set(other);
   }

   public void set(ConvexPolytope3DMessage other)
   {
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.size_, size_);
      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.pose_, pose_);
   }


   /**
            * Dimensions of the ramp, see Ramp3D.
            */
   public us.ihmc.euclid.tuple3D.Vector3D getSize()
   {
      return size_;
   }


   /**
            * Pose of the ramp
            */
   public us.ihmc.euclid.geometry.Pose3D getPose()
   {
      return pose_;
   }


   public static Supplier<ConvexPolytope3DMessagePubSubType> getPubSubType()
   {
      return ConvexPolytope3DMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ConvexPolytope3DMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ConvexPolytope3DMessage other, double epsilon)
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
      if(!(other instanceof ConvexPolytope3DMessage)) return false;

      ConvexPolytope3DMessage otherMyClass = (ConvexPolytope3DMessage) other;

      if (!this.size_.equals(otherMyClass.size_)) return false;
      if (!this.pose_.equals(otherMyClass.pose_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ConvexPolytope3DMessage {");
      builder.append("size=");
      builder.append(this.size_);      builder.append(", ");
      builder.append("pose=");
      builder.append(this.pose_);
      builder.append("}");
      return builder.toString();
   }
}
