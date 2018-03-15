package controller_msgs.msg.dds;

import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * Represents a 6D vector composed of a 3D anuglar vector and a 3D linear vector.
 */
public class SpatialVectorMessage implements Settable<SpatialVectorMessage>, EpsilonComparable<SpatialVectorMessage>
{
   private us.ihmc.euclid.tuple3D.Vector3D angular_part_;
   private us.ihmc.euclid.tuple3D.Vector3D linear_part_;

   public SpatialVectorMessage()
   {
      angular_part_ = new us.ihmc.euclid.tuple3D.Vector3D();
      linear_part_ = new us.ihmc.euclid.tuple3D.Vector3D();
   }

   public SpatialVectorMessage(SpatialVectorMessage other)
   {
      set(other);
   }

   public void set(SpatialVectorMessage other)
   {
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.angular_part_, angular_part_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.linear_part_, linear_part_);
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
      builder.append("angular_part=");
      builder.append(this.angular_part_);

      builder.append(", ");
      builder.append("linear_part=");
      builder.append(this.linear_part_);

      builder.append("}");
      return builder.toString();
   }
}