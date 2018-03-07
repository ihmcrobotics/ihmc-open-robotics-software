package controller_msgs.msg.dds;

import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

public class WeightedJointTrajectoryPoint implements Settable<WeightedJointTrajectoryPoint>, EpsilonComparable<WeightedJointTrajectoryPoint>
{
   private trajectory_msgs.msg.dds.JointTrajectoryPoint point_;
   private double weight_;

   public WeightedJointTrajectoryPoint()
   {
      point_ = new trajectory_msgs.msg.dds.JointTrajectoryPoint();
   }

   public WeightedJointTrajectoryPoint(WeightedJointTrajectoryPoint other)
   {
      set(other);
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
   public boolean epsilonEquals(WeightedJointTrajectoryPoint other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.point_.epsilonEquals(other.point_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.weight_, other.weight_, epsilon))
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
      if (!(other instanceof WeightedJointTrajectoryPoint))
         return false;

      WeightedJointTrajectoryPoint otherMyClass = (WeightedJointTrajectoryPoint) other;

      if (!this.point_.equals(otherMyClass.point_))
         return false;

      if (this.weight_ != otherMyClass.weight_)
         return false;

      return true;
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