package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC multi-contact controller API.
       */
public class WholeBodyContactStateMessage extends Packet<WholeBodyContactStateMessage> implements Settable<WholeBodyContactStateMessage>, EpsilonComparable<WholeBodyContactStateMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Hash codes of contacting bodies
            */
   public us.ihmc.idl.IDLSequence.Integer  contacting_bodies_hash_codes_;
   /**
            * List of contact positions for each rigid body, expressed in RigidBody.getParentJoint().getFrameAfterJoint()
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  contact_point_positions_;

   public WholeBodyContactStateMessage()
   {
      contacting_bodies_hash_codes_ = new us.ihmc.idl.IDLSequence.Integer (100, "type_2");

      contact_point_positions_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D> (100, new geometry_msgs.msg.dds.PointPubSubType());

   }

   public WholeBodyContactStateMessage(WholeBodyContactStateMessage other)
   {
      this();
      set(other);
   }

   public void set(WholeBodyContactStateMessage other)
   {
      sequence_id_ = other.sequence_id_;

      contacting_bodies_hash_codes_.set(other.contacting_bodies_hash_codes_);
      contact_point_positions_.set(other.contact_point_positions_);
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
            * Hash codes of contacting bodies
            */
   public us.ihmc.idl.IDLSequence.Integer  getContactingBodiesHashCodes()
   {
      return contacting_bodies_hash_codes_;
   }


   /**
            * List of contact positions for each rigid body, expressed in RigidBody.getParentJoint().getFrameAfterJoint()
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  getContactPointPositions()
   {
      return contact_point_positions_;
   }


   public static Supplier<WholeBodyContactStateMessagePubSubType> getPubSubType()
   {
      return WholeBodyContactStateMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return WholeBodyContactStateMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(WholeBodyContactStateMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsIntegerSequence(this.contacting_bodies_hash_codes_, other.contacting_bodies_hash_codes_, epsilon)) return false;

      if (this.contact_point_positions_.size() != other.contact_point_positions_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.contact_point_positions_.size(); i++)
         {  if (!this.contact_point_positions_.get(i).epsilonEquals(other.contact_point_positions_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof WholeBodyContactStateMessage)) return false;

      WholeBodyContactStateMessage otherMyClass = (WholeBodyContactStateMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if (!this.contacting_bodies_hash_codes_.equals(otherMyClass.contacting_bodies_hash_codes_)) return false;
      if (!this.contact_point_positions_.equals(otherMyClass.contact_point_positions_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("WholeBodyContactStateMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("contacting_bodies_hash_codes=");
      builder.append(this.contacting_bodies_hash_codes_);      builder.append(", ");
      builder.append("contact_point_positions=");
      builder.append(this.contact_point_positions_);
      builder.append("}");
      return builder.toString();
   }
}
