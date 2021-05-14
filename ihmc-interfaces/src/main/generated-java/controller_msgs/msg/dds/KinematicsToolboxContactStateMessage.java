package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body inverse kinematics module.
       * It holds all the information needed for specifying the contact state of the robot.
       * The inverse kinematics solver provides the option to constrain the projection onto the horizontal plane of the center-of-mass
       * to remain inside the active support polygon also projected onto the horizontal plane.
       * This allows to constrain the solution to be statically stable.
       * While the support polygon is usually determined based on the active controller, i.e. via CapturabilityBasedStatus or MultiContactBalanceStatus,
       * this message can be used to directly specify it.
       */
public class KinematicsToolboxContactStateMessage extends Packet<KinematicsToolboxContactStateMessage> implements Settable<KinematicsToolboxContactStateMessage>, EpsilonComparable<KinematicsToolboxContactStateMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Specifies the minimum distance to keep the center of mass away from the support polygon's edges.
            */
   public double center_of_mass_margin_ = -1.0;
   /**
            * The list of active contact points to use evaluate the support polygon.
            * Each contact point is expected to be expressed in body frame.
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  contact_points_in_body_frame_;
   /**
            * The id used to retrieve the contacting body for each contact point.
            */
   public us.ihmc.idl.IDLSequence.Integer  contacting_body_ids_;

   public KinematicsToolboxContactStateMessage()
   {
      contact_points_in_body_frame_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D> (20, new geometry_msgs.msg.dds.PointPubSubType());
      contacting_body_ids_ = new us.ihmc.idl.IDLSequence.Integer (20, "type_2");


   }

   public KinematicsToolboxContactStateMessage(KinematicsToolboxContactStateMessage other)
   {
      this();
      set(other);
   }

   public void set(KinematicsToolboxContactStateMessage other)
   {
      sequence_id_ = other.sequence_id_;

      center_of_mass_margin_ = other.center_of_mass_margin_;

      contact_points_in_body_frame_.set(other.contact_points_in_body_frame_);
      contacting_body_ids_.set(other.contacting_body_ids_);
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
            * Specifies the minimum distance to keep the center of mass away from the support polygon's edges.
            */
   public void setCenterOfMassMargin(double center_of_mass_margin)
   {
      center_of_mass_margin_ = center_of_mass_margin;
   }
   /**
            * Specifies the minimum distance to keep the center of mass away from the support polygon's edges.
            */
   public double getCenterOfMassMargin()
   {
      return center_of_mass_margin_;
   }


   /**
            * The list of active contact points to use evaluate the support polygon.
            * Each contact point is expected to be expressed in body frame.
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  getContactPointsInBodyFrame()
   {
      return contact_points_in_body_frame_;
   }


   /**
            * The id used to retrieve the contacting body for each contact point.
            */
   public us.ihmc.idl.IDLSequence.Integer  getContactingBodyIds()
   {
      return contacting_body_ids_;
   }


   public static Supplier<KinematicsToolboxContactStateMessagePubSubType> getPubSubType()
   {
      return KinematicsToolboxContactStateMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return KinematicsToolboxContactStateMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(KinematicsToolboxContactStateMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.center_of_mass_margin_, other.center_of_mass_margin_, epsilon)) return false;

      if (this.contact_points_in_body_frame_.size() != other.contact_points_in_body_frame_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.contact_points_in_body_frame_.size(); i++)
         {  if (!this.contact_points_in_body_frame_.get(i).epsilonEquals(other.contact_points_in_body_frame_.get(i), epsilon)) return false; }
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsIntegerSequence(this.contacting_body_ids_, other.contacting_body_ids_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof KinematicsToolboxContactStateMessage)) return false;

      KinematicsToolboxContactStateMessage otherMyClass = (KinematicsToolboxContactStateMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.center_of_mass_margin_ != otherMyClass.center_of_mass_margin_) return false;

      if (!this.contact_points_in_body_frame_.equals(otherMyClass.contact_points_in_body_frame_)) return false;
      if (!this.contacting_body_ids_.equals(otherMyClass.contacting_body_ids_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("KinematicsToolboxContactStateMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("center_of_mass_margin=");
      builder.append(this.center_of_mass_margin_);      builder.append(", ");
      builder.append("contact_points_in_body_frame=");
      builder.append(this.contact_points_in_body_frame_);      builder.append(", ");
      builder.append("contacting_body_ids=");
      builder.append(this.contacting_body_ids_);
      builder.append("}");
      return builder.toString();
   }
}
