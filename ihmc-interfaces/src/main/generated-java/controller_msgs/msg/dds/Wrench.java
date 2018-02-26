package controller_msgs.msg.dds;

/**
 * Definition of the class "Wrench" defined in Wrench_.idl.
 *
 * This file was automatically generated from Wrench_.idl by us.ihmc.idl.generator.IDLGenerator.
 * Do not update this file directly, edit Wrench_.idl instead.
 */
public class Wrench
{
   private us.ihmc.euclid.tuple3D.Vector3D linear_part_;
   private us.ihmc.euclid.tuple3D.Vector3D angular_part_;

   public Wrench()
   {
      linear_part_ = new us.ihmc.euclid.tuple3D.Vector3D();
      angular_part_ = new us.ihmc.euclid.tuple3D.Vector3D();
   }

   public void set(Wrench other)
   {
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.linear_part_, linear_part_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.angular_part_, angular_part_);
   }

   public us.ihmc.euclid.tuple3D.Vector3D getLinear_part()
   {
      return linear_part_;
   }

   public us.ihmc.euclid.tuple3D.Vector3D getAngular_part()
   {
      return angular_part_;
   }

   @Override
   public boolean equals(java.lang.Object other)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;
      if (!(other instanceof Wrench))
         return false;
      Wrench otherMyClass = (Wrench) other;
      boolean returnedValue = true;

      returnedValue &= this.linear_part_.equals(otherMyClass.linear_part_);

      returnedValue &= this.angular_part_.equals(otherMyClass.angular_part_);

      return returnedValue;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("Wrench {");
      builder.append("linear_part=");
      builder.append(this.linear_part_);

      builder.append(", ");
      builder.append("angular_part=");
      builder.append(this.angular_part_);

      builder.append("}");
      return builder.toString();
   }
}