package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC multi-contact controller API.
       * Published by the IHMC controller, this message carries minimal information relative
       * to the balance status of the robot.
       * All the information here is expressed in the world frame.
       */
public class MultiContactBalanceStatus extends Packet<MultiContactBalanceStatus> implements Settable<MultiContactBalanceStatus>, EpsilonComparable<MultiContactBalanceStatus>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * This is the measured position in world of the robot's capture point. Only x and y coordinates are relevant.
            */
   public us.ihmc.euclid.tuple3D.Point3D capture_point_2d_;
   /**
            * This is the measured position in world of the robot's center of mass.
            */
   public us.ihmc.euclid.tuple3D.Point3D center_of_mass_3d_;
   /**
            * List of contact points in world-frame.
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  contact_points_in_world_;
   /**
            * (Optional) List of the contact surface normals in world frame. If provided these are used to solve generalized support region.
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Vector3D>  surface_normals_in_world_;
   /**
            * List of the rigid-bodies in contact. This list's size and ordering corresponds to contact_point_in_world and surface_normals_in_world.
            */
   public us.ihmc.idl.IDLSequence.Integer  support_rigid_body_ids_;

   public MultiContactBalanceStatus()
   {
      capture_point_2d_ = new us.ihmc.euclid.tuple3D.Point3D();
      center_of_mass_3d_ = new us.ihmc.euclid.tuple3D.Point3D();
      contact_points_in_world_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D> (16, new geometry_msgs.msg.dds.PointPubSubType());
      surface_normals_in_world_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Vector3D> (16, new geometry_msgs.msg.dds.Vector3PubSubType());
      support_rigid_body_ids_ = new us.ihmc.idl.IDLSequence.Integer (16, "type_2");


   }

   public MultiContactBalanceStatus(MultiContactBalanceStatus other)
   {
      this();
      set(other);
   }

   public void set(MultiContactBalanceStatus other)
   {
      sequence_id_ = other.sequence_id_;

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.capture_point_2d_, capture_point_2d_);
      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.center_of_mass_3d_, center_of_mass_3d_);
      contact_points_in_world_.set(other.contact_points_in_world_);
      surface_normals_in_world_.set(other.surface_normals_in_world_);
      support_rigid_body_ids_.set(other.support_rigid_body_ids_);
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
            * This is the measured position in world of the robot's capture point. Only x and y coordinates are relevant.
            */
   public us.ihmc.euclid.tuple3D.Point3D getCapturePoint2d()
   {
      return capture_point_2d_;
   }


   /**
            * This is the measured position in world of the robot's center of mass.
            */
   public us.ihmc.euclid.tuple3D.Point3D getCenterOfMass3d()
   {
      return center_of_mass_3d_;
   }


   /**
            * List of contact points in world-frame.
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  getContactPointsInWorld()
   {
      return contact_points_in_world_;
   }


   /**
            * (Optional) List of the contact surface normals in world frame. If provided these are used to solve generalized support region.
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Vector3D>  getSurfaceNormalsInWorld()
   {
      return surface_normals_in_world_;
   }


   /**
            * List of the rigid-bodies in contact. This list's size and ordering corresponds to contact_point_in_world and surface_normals_in_world.
            */
   public us.ihmc.idl.IDLSequence.Integer  getSupportRigidBodyIds()
   {
      return support_rigid_body_ids_;
   }


   public static Supplier<MultiContactBalanceStatusPubSubType> getPubSubType()
   {
      return MultiContactBalanceStatusPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return MultiContactBalanceStatusPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(MultiContactBalanceStatus other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!this.capture_point_2d_.epsilonEquals(other.capture_point_2d_, epsilon)) return false;
      if (!this.center_of_mass_3d_.epsilonEquals(other.center_of_mass_3d_, epsilon)) return false;
      if (this.contact_points_in_world_.size() != other.contact_points_in_world_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.contact_points_in_world_.size(); i++)
         {  if (!this.contact_points_in_world_.get(i).epsilonEquals(other.contact_points_in_world_.get(i), epsilon)) return false; }
      }

      if (this.surface_normals_in_world_.size() != other.surface_normals_in_world_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.surface_normals_in_world_.size(); i++)
         {  if (!this.surface_normals_in_world_.get(i).epsilonEquals(other.surface_normals_in_world_.get(i), epsilon)) return false; }
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsIntegerSequence(this.support_rigid_body_ids_, other.support_rigid_body_ids_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof MultiContactBalanceStatus)) return false;

      MultiContactBalanceStatus otherMyClass = (MultiContactBalanceStatus) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if (!this.capture_point_2d_.equals(otherMyClass.capture_point_2d_)) return false;
      if (!this.center_of_mass_3d_.equals(otherMyClass.center_of_mass_3d_)) return false;
      if (!this.contact_points_in_world_.equals(otherMyClass.contact_points_in_world_)) return false;
      if (!this.surface_normals_in_world_.equals(otherMyClass.surface_normals_in_world_)) return false;
      if (!this.support_rigid_body_ids_.equals(otherMyClass.support_rigid_body_ids_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("MultiContactBalanceStatus {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("capture_point_2d=");
      builder.append(this.capture_point_2d_);      builder.append(", ");
      builder.append("center_of_mass_3d=");
      builder.append(this.center_of_mass_3d_);      builder.append(", ");
      builder.append("contact_points_in_world=");
      builder.append(this.contact_points_in_world_);      builder.append(", ");
      builder.append("surface_normals_in_world=");
      builder.append(this.surface_normals_in_world_);      builder.append(", ");
      builder.append("support_rigid_body_ids=");
      builder.append(this.support_rigid_body_ids_);
      builder.append("}");
      return builder.toString();
   }
}
