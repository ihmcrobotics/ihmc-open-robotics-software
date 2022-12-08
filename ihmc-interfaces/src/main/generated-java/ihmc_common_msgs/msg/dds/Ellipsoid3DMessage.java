package ihmc_common_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This is part of the IHMC Common message package.
       */
public class Ellipsoid3DMessage extends Packet<Ellipsoid3DMessage> implements Settable<Ellipsoid3DMessage>, EpsilonComparable<Ellipsoid3DMessage>
{
   /**
            * Pose of the ellipsoid, see Ellipsoid3D
            */
   public us.ihmc.euclid.geometry.Pose3D pose_;
   /**
            * Radii of the ellipsoid, see Ellipsoid3D
            */
   public us.ihmc.euclid.tuple3D.Vector3D radii_;

   public Ellipsoid3DMessage()
   {
      pose_ = new us.ihmc.euclid.geometry.Pose3D();
      radii_ = new us.ihmc.euclid.tuple3D.Vector3D();
   }

   public Ellipsoid3DMessage(Ellipsoid3DMessage other)
   {
      this();
      set(other);
   }

   public void set(Ellipsoid3DMessage other)
   {
      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.pose_, pose_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.radii_, radii_);
   }


   /**
            * Pose of the ellipsoid, see Ellipsoid3D
            */
   public us.ihmc.euclid.geometry.Pose3D getPose()
   {
      return pose_;
   }


   /**
            * Radii of the ellipsoid, see Ellipsoid3D
            */
   public us.ihmc.euclid.tuple3D.Vector3D getRadii()
   {
      return radii_;
   }


   public static Supplier<Ellipsoid3DMessagePubSubType> getPubSubType()
   {
      return Ellipsoid3DMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return Ellipsoid3DMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(Ellipsoid3DMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.pose_.epsilonEquals(other.pose_, epsilon)) return false;
      if (!this.radii_.epsilonEquals(other.radii_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof Ellipsoid3DMessage)) return false;

      Ellipsoid3DMessage otherMyClass = (Ellipsoid3DMessage) other;

      if (!this.pose_.equals(otherMyClass.pose_)) return false;
      if (!this.radii_.equals(otherMyClass.radii_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("Ellipsoid3DMessage {");
      builder.append("pose=");
      builder.append(this.pose_);      builder.append(", ");
      builder.append("radii=");
      builder.append(this.radii_);
      builder.append("}");
      return builder.toString();
   }
}
