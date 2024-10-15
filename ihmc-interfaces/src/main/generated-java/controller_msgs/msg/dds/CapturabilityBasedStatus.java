package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * Published by the IHMC controller, this message carries minimal information relative
       * to the current balance status of the robot.
       * All the information here is expressed in the world frame.
       */
public class CapturabilityBasedStatus extends Packet<CapturabilityBasedStatus> implements Settable<CapturabilityBasedStatus>, EpsilonComparable<CapturabilityBasedStatus>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * This is the current value of omega, which is the time constant of the inverted pendulum.
            */
   public double omega_;
   /**
            * This is the measured position in world of the robot's capture point. Only x and y coordinates are relevant.
            */
   public us.ihmc.euclid.tuple3D.Point3D capture_point_2d_;
   /**
            * This is the desired position in world for the robot's capture point. Only x and y coordinates are relevant.
            */
   public us.ihmc.euclid.tuple3D.Point3D desired_capture_point_2d_;
   /**
            * This is the measured position in world of the robot's center of mass.
            */
   public us.ihmc.euclid.tuple3D.Point3D center_of_mass_3d_;
   /**
            * List of the active contact points used for the left foot. The coordinates are in world frame.
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  left_foot_support_polygon_3d_;
   /**
            * List of the active contact points used for the right foot. The coordinates are in world frame.
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  right_foot_support_polygon_3d_;
   /**
            * List of the active contact points for the left hand, empty if there is no hand contact point. The coordinates are in the hand body-fixed frame.
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  left_hand_contact_points_;
   /**
            * List of the active contact points for the right hand, empty is there is no hand contact point. The coordinates are in the hand body-fixed frame.
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  right_hand_contact_points_;
   /**
            * Surface normal of the left hand, if in contact. Contact is assumed to be at the hand control frame. Set to zero (the default value) if not in contact.
            */
   public us.ihmc.euclid.tuple3D.Vector3D left_hand_contact_normal_;
   /**
            * Surface normal of the right hand, if in contact. Contact is assumed to be at the hand control frame. Set to zero (the default value) if not in contact.
            */
   public us.ihmc.euclid.tuple3D.Vector3D right_hand_contact_normal_;
   /**
            * The estimated hand wrench of the right hand
            */
   public controller_msgs.msg.dds.SpatialVectorMessage right_hand_wrench_;
   /**
            * The estimated hand wrench of the left hand
            */
   public controller_msgs.msg.dds.SpatialVectorMessage left_hand_wrench_;

   public CapturabilityBasedStatus()
   {
      capture_point_2d_ = new us.ihmc.euclid.tuple3D.Point3D();
      desired_capture_point_2d_ = new us.ihmc.euclid.tuple3D.Point3D();
      center_of_mass_3d_ = new us.ihmc.euclid.tuple3D.Point3D();
      left_foot_support_polygon_3d_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D> (8, new geometry_msgs.msg.dds.PointPubSubType());
      right_foot_support_polygon_3d_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D> (8, new geometry_msgs.msg.dds.PointPubSubType());
      left_hand_contact_points_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D> (1, new geometry_msgs.msg.dds.PointPubSubType());
      right_hand_contact_points_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D> (1, new geometry_msgs.msg.dds.PointPubSubType());
      left_hand_contact_normal_ = new us.ihmc.euclid.tuple3D.Vector3D();
      right_hand_contact_normal_ = new us.ihmc.euclid.tuple3D.Vector3D();
      right_hand_wrench_ = new controller_msgs.msg.dds.SpatialVectorMessage();
      left_hand_wrench_ = new controller_msgs.msg.dds.SpatialVectorMessage();

   }

   public CapturabilityBasedStatus(CapturabilityBasedStatus other)
   {
      this();
      set(other);
   }

   public void set(CapturabilityBasedStatus other)
   {
      sequence_id_ = other.sequence_id_;

      omega_ = other.omega_;

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.capture_point_2d_, capture_point_2d_);
      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.desired_capture_point_2d_, desired_capture_point_2d_);
      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.center_of_mass_3d_, center_of_mass_3d_);
      left_foot_support_polygon_3d_.set(other.left_foot_support_polygon_3d_);
      right_foot_support_polygon_3d_.set(other.right_foot_support_polygon_3d_);
      left_hand_contact_points_.set(other.left_hand_contact_points_);
      right_hand_contact_points_.set(other.right_hand_contact_points_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.left_hand_contact_normal_, left_hand_contact_normal_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.right_hand_contact_normal_, right_hand_contact_normal_);
      controller_msgs.msg.dds.SpatialVectorMessagePubSubType.staticCopy(other.right_hand_wrench_, right_hand_wrench_);
      controller_msgs.msg.dds.SpatialVectorMessagePubSubType.staticCopy(other.left_hand_wrench_, left_hand_wrench_);
   }

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public void setSequenceId(long sequence_id)
   {
      sequence_id_ = sequence_id;
   }
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long getSequenceId()
   {
      return sequence_id_;
   }

   /**
            * This is the current value of omega, which is the time constant of the inverted pendulum.
            */
   public void setOmega(double omega)
   {
      omega_ = omega;
   }
   /**
            * This is the current value of omega, which is the time constant of the inverted pendulum.
            */
   public double getOmega()
   {
      return omega_;
   }


   /**
            * This is the measured position in world of the robot's capture point. Only x and y coordinates are relevant.
            */
   public us.ihmc.euclid.tuple3D.Point3D getCapturePoint2d()
   {
      return capture_point_2d_;
   }


   /**
            * This is the desired position in world for the robot's capture point. Only x and y coordinates are relevant.
            */
   public us.ihmc.euclid.tuple3D.Point3D getDesiredCapturePoint2d()
   {
      return desired_capture_point_2d_;
   }


   /**
            * This is the measured position in world of the robot's center of mass.
            */
   public us.ihmc.euclid.tuple3D.Point3D getCenterOfMass3d()
   {
      return center_of_mass_3d_;
   }


   /**
            * List of the active contact points used for the left foot. The coordinates are in world frame.
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  getLeftFootSupportPolygon3d()
   {
      return left_foot_support_polygon_3d_;
   }


   /**
            * List of the active contact points used for the right foot. The coordinates are in world frame.
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  getRightFootSupportPolygon3d()
   {
      return right_foot_support_polygon_3d_;
   }


   /**
            * List of the active contact points for the left hand, empty if there is no hand contact point. The coordinates are in the hand body-fixed frame.
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  getLeftHandContactPoints()
   {
      return left_hand_contact_points_;
   }


   /**
            * List of the active contact points for the right hand, empty is there is no hand contact point. The coordinates are in the hand body-fixed frame.
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  getRightHandContactPoints()
   {
      return right_hand_contact_points_;
   }


   /**
            * Surface normal of the left hand, if in contact. Contact is assumed to be at the hand control frame. Set to zero (the default value) if not in contact.
            */
   public us.ihmc.euclid.tuple3D.Vector3D getLeftHandContactNormal()
   {
      return left_hand_contact_normal_;
   }


   /**
            * Surface normal of the right hand, if in contact. Contact is assumed to be at the hand control frame. Set to zero (the default value) if not in contact.
            */
   public us.ihmc.euclid.tuple3D.Vector3D getRightHandContactNormal()
   {
      return right_hand_contact_normal_;
   }


   /**
            * The estimated hand wrench of the right hand
            */
   public controller_msgs.msg.dds.SpatialVectorMessage getRightHandWrench()
   {
      return right_hand_wrench_;
   }


   /**
            * The estimated hand wrench of the left hand
            */
   public controller_msgs.msg.dds.SpatialVectorMessage getLeftHandWrench()
   {
      return left_hand_wrench_;
   }


   public static Supplier<CapturabilityBasedStatusPubSubType> getPubSubType()
   {
      return CapturabilityBasedStatusPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return CapturabilityBasedStatusPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(CapturabilityBasedStatus other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.omega_, other.omega_, epsilon)) return false;

      if (!this.capture_point_2d_.epsilonEquals(other.capture_point_2d_, epsilon)) return false;
      if (!this.desired_capture_point_2d_.epsilonEquals(other.desired_capture_point_2d_, epsilon)) return false;
      if (!this.center_of_mass_3d_.epsilonEquals(other.center_of_mass_3d_, epsilon)) return false;
      if (this.left_foot_support_polygon_3d_.size() != other.left_foot_support_polygon_3d_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.left_foot_support_polygon_3d_.size(); i++)
         {  if (!this.left_foot_support_polygon_3d_.get(i).epsilonEquals(other.left_foot_support_polygon_3d_.get(i), epsilon)) return false; }
      }

      if (this.right_foot_support_polygon_3d_.size() != other.right_foot_support_polygon_3d_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.right_foot_support_polygon_3d_.size(); i++)
         {  if (!this.right_foot_support_polygon_3d_.get(i).epsilonEquals(other.right_foot_support_polygon_3d_.get(i), epsilon)) return false; }
      }

      if (this.left_hand_contact_points_.size() != other.left_hand_contact_points_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.left_hand_contact_points_.size(); i++)
         {  if (!this.left_hand_contact_points_.get(i).epsilonEquals(other.left_hand_contact_points_.get(i), epsilon)) return false; }
      }

      if (this.right_hand_contact_points_.size() != other.right_hand_contact_points_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.right_hand_contact_points_.size(); i++)
         {  if (!this.right_hand_contact_points_.get(i).epsilonEquals(other.right_hand_contact_points_.get(i), epsilon)) return false; }
      }

      if (!this.left_hand_contact_normal_.epsilonEquals(other.left_hand_contact_normal_, epsilon)) return false;
      if (!this.right_hand_contact_normal_.epsilonEquals(other.right_hand_contact_normal_, epsilon)) return false;
      if (!this.right_hand_wrench_.epsilonEquals(other.right_hand_wrench_, epsilon)) return false;
      if (!this.left_hand_wrench_.epsilonEquals(other.left_hand_wrench_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof CapturabilityBasedStatus)) return false;

      CapturabilityBasedStatus otherMyClass = (CapturabilityBasedStatus) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.omega_ != otherMyClass.omega_) return false;

      if (!this.capture_point_2d_.equals(otherMyClass.capture_point_2d_)) return false;
      if (!this.desired_capture_point_2d_.equals(otherMyClass.desired_capture_point_2d_)) return false;
      if (!this.center_of_mass_3d_.equals(otherMyClass.center_of_mass_3d_)) return false;
      if (!this.left_foot_support_polygon_3d_.equals(otherMyClass.left_foot_support_polygon_3d_)) return false;
      if (!this.right_foot_support_polygon_3d_.equals(otherMyClass.right_foot_support_polygon_3d_)) return false;
      if (!this.left_hand_contact_points_.equals(otherMyClass.left_hand_contact_points_)) return false;
      if (!this.right_hand_contact_points_.equals(otherMyClass.right_hand_contact_points_)) return false;
      if (!this.left_hand_contact_normal_.equals(otherMyClass.left_hand_contact_normal_)) return false;
      if (!this.right_hand_contact_normal_.equals(otherMyClass.right_hand_contact_normal_)) return false;
      if (!this.right_hand_wrench_.equals(otherMyClass.right_hand_wrench_)) return false;
      if (!this.left_hand_wrench_.equals(otherMyClass.left_hand_wrench_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("CapturabilityBasedStatus {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("omega=");
      builder.append(this.omega_);      builder.append(", ");
      builder.append("capture_point_2d=");
      builder.append(this.capture_point_2d_);      builder.append(", ");
      builder.append("desired_capture_point_2d=");
      builder.append(this.desired_capture_point_2d_);      builder.append(", ");
      builder.append("center_of_mass_3d=");
      builder.append(this.center_of_mass_3d_);      builder.append(", ");
      builder.append("left_foot_support_polygon_3d=");
      builder.append(this.left_foot_support_polygon_3d_);      builder.append(", ");
      builder.append("right_foot_support_polygon_3d=");
      builder.append(this.right_foot_support_polygon_3d_);      builder.append(", ");
      builder.append("left_hand_contact_points=");
      builder.append(this.left_hand_contact_points_);      builder.append(", ");
      builder.append("right_hand_contact_points=");
      builder.append(this.right_hand_contact_points_);      builder.append(", ");
      builder.append("left_hand_contact_normal=");
      builder.append(this.left_hand_contact_normal_);      builder.append(", ");
      builder.append("right_hand_contact_normal=");
      builder.append(this.right_hand_contact_normal_);      builder.append(", ");
      builder.append("right_hand_wrench=");
      builder.append(this.right_hand_wrench_);      builder.append(", ");
      builder.append("left_hand_wrench=");
      builder.append(this.left_hand_wrench_);
      builder.append("}");
      return builder.toString();
   }
}
