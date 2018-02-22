package controller_msgs.msg.dds;

/**
 * Definition of the class "WeightedJointTrajectoryPoint" defined in WeightedJointTrajectoryPoint_.idl.
 *
 * This file was automatically generated from WeightedJointTrajectoryPoint_.idl by us.ihmc.idl.generator.IDLGenerator.
 * Do not update this file directly, edit WeightedJointTrajectoryPoint_.idl instead.
 */
public class WeightedJointTrajectoryPoint
{
   private trajectory_msgs.msg.dds.JointTrajectoryPoint point_;
   private double weight_;

   public WeightedJointTrajectoryPoint()
   {
      point_ = new trajectory_msgs.msg.dds.JointTrajectoryPoint();
   }

   public void set(WeightedJointTrajectoryPoint other)
   {
      trajectory_msgs.msg.dds.JointTrajectoryPointPubSubType.staticCopy(other.point_, point_);
      weight_ = other.weight_;
   }

   public trajectory_msgs.msg.dds.JointTrajectoryPoint getPoint()
   {
      return point_;
   }

   public double getWeight()
   {
      return weight_;
   }

   public void setWeight(double weight)
   {
      weight_ = weight;
   }

   @Override
   public boolean equals(java.lang.Object other)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;
      if (!(other instanceof WeightedJointTrajectoryPoint))
         return false;
      WeightedJointTrajectoryPoint otherMyClass = (WeightedJointTrajectoryPoint) other;
      boolean returnedValue = true;

      returnedValue &= this.point_.equals(otherMyClass.point_);

      returnedValue &= this.weight_ == otherMyClass.weight_;

      return returnedValue;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("WeightedJointTrajectoryPoint {");
      builder.append("point=");
      builder.append(this.point_);

      builder.append(", ");
      builder.append("weight=");
      builder.append(this.weight_);

      builder.append("}");
      return builder.toString();
   }
}