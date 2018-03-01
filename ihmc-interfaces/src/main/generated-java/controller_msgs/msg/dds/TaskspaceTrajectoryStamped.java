package controller_msgs.msg.dds;

/**
 * Definition of the class "TaskspaceTrajectoryStamped" defined in TaskspaceTrajectoryStamped_.idl.
 *
 * This file was automatically generated from TaskspaceTrajectoryStamped_.idl by us.ihmc.idl.generator.IDLGenerator.
 * Do not update this file directly, edit TaskspaceTrajectoryStamped_.idl instead.
 */
public class TaskspaceTrajectoryStamped
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

   public us.ihmc.idl.IDLSequence.Object<geometry_msgs.msg.dds.PoseStamped> getTrajectory_points_stamped()
   {
      return trajectory_points_stamped_;
   }

   public builtin_interfaces.msg.dds.Duration getTime_from_start()
   {
      return time_from_start_;
   }

   @Override
   public boolean equals(java.lang.Object other)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;
      if (!(other instanceof TaskspaceTrajectoryStamped))
         return false;
      TaskspaceTrajectoryStamped otherMyClass = (TaskspaceTrajectoryStamped) other;
      boolean returnedValue = true;

      returnedValue &= this.header_.equals(otherMyClass.header_);

      returnedValue &= this.trajectory_points_stamped_.equals(otherMyClass.trajectory_points_stamped_);

      returnedValue &= this.time_from_start_.equals(otherMyClass.time_from_start_);

      return returnedValue;
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