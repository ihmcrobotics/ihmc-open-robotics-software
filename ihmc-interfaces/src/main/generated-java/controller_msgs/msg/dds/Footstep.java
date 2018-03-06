package controller_msgs.msg.dds;

import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

public class Footstep implements Settable<Footstep>, EpsilonComparable<Footstep>
{
   private long unique_id_;
   private byte robot_side_;
   private us.ihmc.euclid.tuple3D.Point3D location_;
   private us.ihmc.euclid.tuple4D.Quaternion orientation_;
   private us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D> predicted_contact_points_2d_;
   private byte trajectory_type_;
   private double swing_height_;
   private us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D> position_waypoints_;
   private us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.TaskspaceTrajectoryStamped> swing_trajectory_;
   private double swing_trajectory_blend_duration_;
   private double swing_duration_;
   private double transfer_duration_;

   public Footstep()
   {

      location_ = new us.ihmc.euclid.tuple3D.Point3D();
      orientation_ = new us.ihmc.euclid.tuple4D.Quaternion();
      predicted_contact_points_2d_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>(100, us.ihmc.euclid.tuple3D.Point3D.class,
                                                                                                        new geometry_msgs.msg.dds.PointPubSubType());

      position_waypoints_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>(100, us.ihmc.euclid.tuple3D.Point3D.class,
                                                                                               new geometry_msgs.msg.dds.PointPubSubType());

      swing_trajectory_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.TaskspaceTrajectoryStamped>(100,
                                                                                                                 controller_msgs.msg.dds.TaskspaceTrajectoryStamped.class,
                                                                                                                 new controller_msgs.msg.dds.TaskspaceTrajectoryStampedPubSubType());
   }

   public Footstep(Footstep other)
   {
      set(other);
   }

   public void set(Footstep other)
   {
      unique_id_ = other.unique_id_;

      robot_side_ = other.robot_side_;

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.location_, location_);
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.orientation_, orientation_);
      predicted_contact_points_2d_.set(other.predicted_contact_points_2d_);
      trajectory_type_ = other.trajectory_type_;

      swing_height_ = other.swing_height_;

      position_waypoints_.set(other.position_waypoints_);
      swing_trajectory_.set(other.swing_trajectory_);
      swing_trajectory_blend_duration_ = other.swing_trajectory_blend_duration_;

      swing_duration_ = other.swing_duration_;

      transfer_duration_ = other.transfer_duration_;
   }

   public long getUniqueId()
   {
      return unique_id_;
   }

   public void setUniqueId(long unique_id)
   {
      unique_id_ = unique_id;
   }

   public byte getRobotSide()
   {
      return robot_side_;
   }

   public void setRobotSide(byte robot_side)
   {
      robot_side_ = robot_side;
   }

   public us.ihmc.euclid.tuple3D.Point3D getLocation()
   {
      return location_;
   }

   public us.ihmc.euclid.tuple4D.Quaternion getOrientation()
   {
      return orientation_;
   }

   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D> getPredictedContactPoints2d()
   {
      return predicted_contact_points_2d_;
   }

   public byte getTrajectoryType()
   {
      return trajectory_type_;
   }

   public void setTrajectoryType(byte trajectory_type)
   {
      trajectory_type_ = trajectory_type;
   }

   public double getSwingHeight()
   {
      return swing_height_;
   }

   public void setSwingHeight(double swing_height)
   {
      swing_height_ = swing_height;
   }

   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D> getPositionWaypoints()
   {
      return position_waypoints_;
   }

   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.TaskspaceTrajectoryStamped> getSwingTrajectory()
   {
      return swing_trajectory_;
   }

   public double getSwingTrajectoryBlendDuration()
   {
      return swing_trajectory_blend_duration_;
   }

   public void setSwingTrajectoryBlendDuration(double swing_trajectory_blend_duration)
   {
      swing_trajectory_blend_duration_ = swing_trajectory_blend_duration;
   }

   public double getSwingDuration()
   {
      return swing_duration_;
   }

   public void setSwingDuration(double swing_duration)
   {
      swing_duration_ = swing_duration;
   }

   public double getTransferDuration()
   {
      return transfer_duration_;
   }

   public void setTransferDuration(double transfer_duration)
   {
      transfer_duration_ = transfer_duration;
   }

   @Override
   public boolean epsilonEquals(Footstep other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.unique_id_, other.unique_id_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon))
         return false;

      if (!this.location_.epsilonEquals(other.location_, epsilon))
         return false;

      if (!this.orientation_.epsilonEquals(other.orientation_, epsilon))
         return false;

      if (this.predicted_contact_points_2d_.isEnum())
      {
         if (!this.predicted_contact_points_2d_.equals(other.predicted_contact_points_2d_))
            return false;
      }
      else if (this.predicted_contact_points_2d_.size() == other.predicted_contact_points_2d_.size())
      {
         return false;
      }
      else
      {
         for (int i = 0; i < this.predicted_contact_points_2d_.size(); i++)
         {
            if (!this.predicted_contact_points_2d_.get(i).epsilonEquals(other.predicted_contact_points_2d_.get(i), epsilon))
               return false;
         }
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.trajectory_type_, other.trajectory_type_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.swing_height_, other.swing_height_, epsilon))
         return false;

      if (this.position_waypoints_.isEnum())
      {
         if (!this.position_waypoints_.equals(other.position_waypoints_))
            return false;
      }
      else if (this.position_waypoints_.size() == other.position_waypoints_.size())
      {
         return false;
      }
      else
      {
         for (int i = 0; i < this.position_waypoints_.size(); i++)
         {
            if (!this.position_waypoints_.get(i).epsilonEquals(other.position_waypoints_.get(i), epsilon))
               return false;
         }
      }

      if (this.swing_trajectory_.isEnum())
      {
         if (!this.swing_trajectory_.equals(other.swing_trajectory_))
            return false;
      }
      else if (this.swing_trajectory_.size() == other.swing_trajectory_.size())
      {
         return false;
      }
      else
      {
         for (int i = 0; i < this.swing_trajectory_.size(); i++)
         {
            if (!this.swing_trajectory_.get(i).epsilonEquals(other.swing_trajectory_.get(i), epsilon))
               return false;
         }
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.swing_trajectory_blend_duration_, other.swing_trajectory_blend_duration_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.swing_duration_, other.swing_duration_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.transfer_duration_, other.transfer_duration_, epsilon))
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
      if (!(other instanceof Footstep))
         return false;

      Footstep otherMyClass = (Footstep) other;

      if (this.unique_id_ != otherMyClass.unique_id_)
         return false;

      if (this.robot_side_ != otherMyClass.robot_side_)
         return false;

      if (!this.location_.equals(otherMyClass.location_))
         return false;

      if (!this.orientation_.equals(otherMyClass.orientation_))
         return false;

      if (!this.predicted_contact_points_2d_.equals(otherMyClass.predicted_contact_points_2d_))
         return false;

      if (this.trajectory_type_ != otherMyClass.trajectory_type_)
         return false;

      if (this.swing_height_ != otherMyClass.swing_height_)
         return false;

      if (!this.position_waypoints_.equals(otherMyClass.position_waypoints_))
         return false;

      if (!this.swing_trajectory_.equals(otherMyClass.swing_trajectory_))
         return false;

      if (this.swing_trajectory_blend_duration_ != otherMyClass.swing_trajectory_blend_duration_)
         return false;

      if (this.swing_duration_ != otherMyClass.swing_duration_)
         return false;

      if (this.transfer_duration_ != otherMyClass.transfer_duration_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("Footstep {");
      builder.append("unique_id=");
      builder.append(this.unique_id_);

      builder.append(", ");
      builder.append("robot_side=");
      builder.append(this.robot_side_);

      builder.append(", ");
      builder.append("location=");
      builder.append(this.location_);

      builder.append(", ");
      builder.append("orientation=");
      builder.append(this.orientation_);

      builder.append(", ");
      builder.append("predicted_contact_points_2d=");
      builder.append(this.predicted_contact_points_2d_);

      builder.append(", ");
      builder.append("trajectory_type=");
      builder.append(this.trajectory_type_);

      builder.append(", ");
      builder.append("swing_height=");
      builder.append(this.swing_height_);

      builder.append(", ");
      builder.append("position_waypoints=");
      builder.append(this.position_waypoints_);

      builder.append(", ");
      builder.append("swing_trajectory=");
      builder.append(this.swing_trajectory_);

      builder.append(", ");
      builder.append("swing_trajectory_blend_duration=");
      builder.append(this.swing_trajectory_blend_duration_);

      builder.append(", ");
      builder.append("swing_duration=");
      builder.append(this.swing_duration_);

      builder.append(", ");
      builder.append("transfer_duration=");
      builder.append(this.transfer_duration_);

      builder.append("}");
      return builder.toString();
   }
}