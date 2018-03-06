package controller_msgs.msg.dds;

import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

public class TaskspaceTrajectoryStamped implements Settable<TaskspaceTrajectoryStamped>, EpsilonComparable<TaskspaceTrajectoryStamped>
{
   private std_msgs.msg.dds.Header header_;
   private us.ihmc.idl.IDLSequence.Object<geometry_msgs.msg.dds.PoseStamped> trajectory_points_stamped_;
   private builtin_interfaces.msg.dds.Duration time_from_start_;

   public TaskspaceTrajectoryStamped()
   {
      header_ = new std_msgs.msg.dds.Header();
      trajectory_points_stamped_ = new us.ihmc.idl.IDLSequence.Object<geometry_msgs.msg.dds.PoseStamped>(100, geometry_msgs.msg.dds.PoseStamped.class,
                                                                                                         new geometry_msgs.msg.dds.PoseStampedPubSubType());

      time_from_start_ = new builtin_interfaces.msg.dds.Duration();
   }

   public TaskspaceTrajectoryStamped(TaskspaceTrajectoryStamped other)
   {
      set(other);
   }

   public void set(TaskspaceTrajectoryStamped other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      trajectory_points_stamped_.set(other.trajectory_points_stamped_);
      builtin_interfaces.msg.dds.DurationPubSubType.staticCopy(other.time_from_start_, time_from_start_);
   }

   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }

   public us.ihmc.idl.IDLSequence.Object<geometry_msgs.msg.dds.PoseStamped> getTrajectoryPointsStamped()
   {
      return trajectory_points_stamped_;
   }

   public builtin_interfaces.msg.dds.Duration getTimeFromStart()
   {
      return time_from_start_;
   }

   @Override
   public boolean epsilonEquals(TaskspaceTrajectoryStamped other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon))
         return false;

      if (this.trajectory_points_stamped_.isEnum())
      {
         if (!this.trajectory_points_stamped_.equals(other.trajectory_points_stamped_))
            return false;
      }
      else if (this.trajectory_points_stamped_.size() == other.trajectory_points_stamped_.size())
      {
         return false;
      }
      else
      {
         for (int i = 0; i < this.trajectory_points_stamped_.size(); i++)
         {
            if (!this.trajectory_points_stamped_.get(i).epsilonEquals(other.trajectory_points_stamped_.get(i), epsilon))
               return false;
         }
      }

      if (!this.time_from_start_.epsilonEquals(other.time_from_start_, epsilon))
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
      if (!(other instanceof TaskspaceTrajectoryStamped))
         return false;

      TaskspaceTrajectoryStamped otherMyClass = (TaskspaceTrajectoryStamped) other;

      if (!this.header_.equals(otherMyClass.header_))
         return false;

      if (!this.trajectory_points_stamped_.equals(otherMyClass.trajectory_points_stamped_))
         return false;

      if (!this.time_from_start_.equals(otherMyClass.time_from_start_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("TaskspaceTrajectoryStamped {");
      builder.append("header=");
      builder.append(this.header_);

      builder.append(", ");
      builder.append("trajectory_points_stamped=");
      builder.append(this.trajectory_points_stamped_);

      builder.append(", ");
      builder.append("time_from_start=");
      builder.append(this.time_from_start_);

      builder.append("}");
      return builder.toString();
   }
}