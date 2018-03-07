package controller_msgs.msg.dds;

import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * This message is part of the IHMC whole-body controller API.
 * Published by the IHMC controller, this message carries minimal information relative
 * to the current balance status of the robot.
 * All the information here is expressed in the world frame.
 */
public class CapturabilityBasedStatus implements Settable<CapturabilityBasedStatus>, EpsilonComparable<CapturabilityBasedStatus>
{
   private us.ihmc.euclid.tuple3D.Point3D capture_point_2d_;
   private us.ihmc.euclid.tuple3D.Point3D desired_capture_point_2d_;
   private us.ihmc.euclid.tuple3D.Point3D center_of_mass_3d_;
   private us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D> left_foot_support_polygon_2d_;
   private us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D> right_foot_support_polygon_2d_;

   public CapturabilityBasedStatus()
   {
      capture_point_2d_ = new us.ihmc.euclid.tuple3D.Point3D();
      desired_capture_point_2d_ = new us.ihmc.euclid.tuple3D.Point3D();
      center_of_mass_3d_ = new us.ihmc.euclid.tuple3D.Point3D();
      left_foot_support_polygon_2d_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>(8, us.ihmc.euclid.tuple3D.Point3D.class,
                                                                                                         new geometry_msgs.msg.dds.PointPubSubType());

      right_foot_support_polygon_2d_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>(8, us.ihmc.euclid.tuple3D.Point3D.class,
                                                                                                          new geometry_msgs.msg.dds.PointPubSubType());
   }

   public CapturabilityBasedStatus(CapturabilityBasedStatus other)
   {
      set(other);
   }

   public void set(CapturabilityBasedStatus other)
   {
      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.capture_point_2d_, capture_point_2d_);
      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.desired_capture_point_2d_, desired_capture_point_2d_);
      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.center_of_mass_3d_, center_of_mass_3d_);
      left_foot_support_polygon_2d_.set(other.left_foot_support_polygon_2d_);
      right_foot_support_polygon_2d_.set(other.right_foot_support_polygon_2d_);
   }

   public us.ihmc.euclid.tuple3D.Point3D getCapturePoint2d()
   {
      return capture_point_2d_;
   }

   public us.ihmc.euclid.tuple3D.Point3D getDesiredCapturePoint2d()
   {
      return desired_capture_point_2d_;
   }

   public us.ihmc.euclid.tuple3D.Point3D getCenterOfMass3d()
   {
      return center_of_mass_3d_;
   }

   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D> getLeftFootSupportPolygon2d()
   {
      return left_foot_support_polygon_2d_;
   }

   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D> getRightFootSupportPolygon2d()
   {
      return right_foot_support_polygon_2d_;
   }

   @Override
   public boolean epsilonEquals(CapturabilityBasedStatus other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.capture_point_2d_.epsilonEquals(other.capture_point_2d_, epsilon))
         return false;

      if (!this.desired_capture_point_2d_.epsilonEquals(other.desired_capture_point_2d_, epsilon))
         return false;

      if (!this.center_of_mass_3d_.epsilonEquals(other.center_of_mass_3d_, epsilon))
         return false;

      if (this.left_foot_support_polygon_2d_.size() == other.left_foot_support_polygon_2d_.size())
      {
         return false;
      }
      else
      {
         for (int i = 0; i < this.left_foot_support_polygon_2d_.size(); i++)
         {
            if (!this.left_foot_support_polygon_2d_.get(i).epsilonEquals(other.left_foot_support_polygon_2d_.get(i), epsilon))
               return false;
         }
      }

      if (this.right_foot_support_polygon_2d_.size() == other.right_foot_support_polygon_2d_.size())
      {
         return false;
      }
      else
      {
         for (int i = 0; i < this.right_foot_support_polygon_2d_.size(); i++)
         {
            if (!this.right_foot_support_polygon_2d_.get(i).epsilonEquals(other.right_foot_support_polygon_2d_.get(i), epsilon))
               return false;
         }
      }

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;
      if (!(other instanceof CapturabilityBasedStatus))
         return false;

      CapturabilityBasedStatus otherMyClass = (CapturabilityBasedStatus) other;

      if (!this.capture_point_2d_.equals(otherMyClass.capture_point_2d_))
         return false;

      if (!this.desired_capture_point_2d_.equals(otherMyClass.desired_capture_point_2d_))
         return false;

      if (!this.center_of_mass_3d_.equals(otherMyClass.center_of_mass_3d_))
         return false;

      if (!this.left_foot_support_polygon_2d_.equals(otherMyClass.left_foot_support_polygon_2d_))
         return false;

      if (!this.right_foot_support_polygon_2d_.equals(otherMyClass.right_foot_support_polygon_2d_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("CapturabilityBasedStatus {");
      builder.append("capture_point_2d=");
      builder.append(this.capture_point_2d_);

      builder.append(", ");
      builder.append("desired_capture_point_2d=");
      builder.append(this.desired_capture_point_2d_);

      builder.append(", ");
      builder.append("center_of_mass_3d=");
      builder.append(this.center_of_mass_3d_);

      builder.append(", ");
      builder.append("left_foot_support_polygon_2d=");
      builder.append(this.left_foot_support_polygon_2d_);

      builder.append(", ");
      builder.append("right_foot_support_polygon_2d=");
      builder.append(this.right_foot_support_polygon_2d_);

      builder.append("}");
      return builder.toString();
   }
}