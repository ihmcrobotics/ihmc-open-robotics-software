package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

/**
 * Represents a 6D vector composed of a 3D anuglar vector and a 3D linear vector.
 */
public class SpatialVectorMessage extends Packet<SpatialVectorMessage> implements Settable<SpatialVectorMessage>, EpsilonComparable<SpatialVectorMessage>
{
   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header header_;
   public us.ihmc.euclid.tuple3D.Vector3D angular_part_;
   public us.ihmc.euclid.tuple3D.Vector3D linear_part_;

   public SpatialVectorMessage()
   {
      header_ = new std_msgs.msg.dds.Header();
      angular_part_ = new us.ihmc.euclid.tuple3D.Vector3D();
      linear_part_ = new us.ihmc.euclid.tuple3D.Vector3D();
   }

   public SpatialVectorMessage(SpatialVectorMessage other)
   {
      this();
      set(other);
   }

   public void set(SpatialVectorMessage other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.angular_part_, angular_part_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.linear_part_, linear_part_);
   }

   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }

   public us.ihmc.euclid.tuple3D.Vector3D getAngularPart()
   {
      return angular_part_;
   }

   public us.ihmc.euclid.tuple3D.Vector3D getLinearPart()
   {
      return linear_part_;
   }

   @Override
   public boolean epsilonEquals(SpatialVectorMessage other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon))
         return false;
      if (!this.angular_part_.epsilonEquals(other.angular_part_, epsilon))
         return false;
      if (!this.linear_part_.epsilonEquals(other.linear_part_, epsilon))
         return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;
      if (!(other instanceof SpatialVectorMessage))
         return false;

      SpatialVectorMessage otherMyClass = (SpatialVectorMessage) other;

      if (!this.header_.equals(otherMyClass.header_))
         return false;
      if (!this.angular_part_.equals(otherMyClass.angular_part_))
         return false;
      if (!this.linear_part_.equals(otherMyClass.linear_part_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("SpatialVectorMessage {");
      builder.append("header=");
      builder.append(this.header_);
      builder.append(", ");
      builder.append("angular_part=");
      builder.append(this.angular_part_);
      builder.append(", ");
      builder.append("linear_part=");
      builder.append(this.linear_part_);
      builder.append("}");
      return builder.toString();
   }
}
