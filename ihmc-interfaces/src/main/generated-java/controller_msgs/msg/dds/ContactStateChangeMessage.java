package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC multi-contact controller API.
       * Provides a way for a user or module external to the controller to signal a change in contact state
       */
public class ContactStateChangeMessage extends Packet<ContactStateChangeMessage> implements Settable<ContactStateChangeMessage>, EpsilonComparable<ContactStateChangeMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Hash code of the rigid body that will have a contact change
            */
   public int rigid_body_hash_code_;
   /**
            * Flag to indicate whether this rigid body is making or breaking contact.
            * If true, the controller will try to add a contact point at the given time. If the rigid body is already in contact, this request is ignored.
            * Otherwise, the controller will add a contact point at the rigid body's lowest point in world frame as defined by its collision mesh.
            * If false, the controller will remove all contact points associated with this rigid body.
            */
   public boolean add_contact_point_;

   public ContactStateChangeMessage()
   {
   }

   public ContactStateChangeMessage(ContactStateChangeMessage other)
   {
      this();
      set(other);
   }

   public void set(ContactStateChangeMessage other)
   {
      sequence_id_ = other.sequence_id_;

      rigid_body_hash_code_ = other.rigid_body_hash_code_;

      add_contact_point_ = other.add_contact_point_;

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
            * Hash code of the rigid body that will have a contact change
            */
   public void setRigidBodyHashCode(int rigid_body_hash_code)
   {
      rigid_body_hash_code_ = rigid_body_hash_code;
   }
   /**
            * Hash code of the rigid body that will have a contact change
            */
   public int getRigidBodyHashCode()
   {
      return rigid_body_hash_code_;
   }

   /**
            * Flag to indicate whether this rigid body is making or breaking contact.
            * If true, the controller will try to add a contact point at the given time. If the rigid body is already in contact, this request is ignored.
            * Otherwise, the controller will add a contact point at the rigid body's lowest point in world frame as defined by its collision mesh.
            * If false, the controller will remove all contact points associated with this rigid body.
            */
   public void setAddContactPoint(boolean add_contact_point)
   {
      add_contact_point_ = add_contact_point;
   }
   /**
            * Flag to indicate whether this rigid body is making or breaking contact.
            * If true, the controller will try to add a contact point at the given time. If the rigid body is already in contact, this request is ignored.
            * Otherwise, the controller will add a contact point at the rigid body's lowest point in world frame as defined by its collision mesh.
            * If false, the controller will remove all contact points associated with this rigid body.
            */
   public boolean getAddContactPoint()
   {
      return add_contact_point_;
   }


   public static Supplier<ContactStateChangeMessagePubSubType> getPubSubType()
   {
      return ContactStateChangeMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ContactStateChangeMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ContactStateChangeMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.rigid_body_hash_code_, other.rigid_body_hash_code_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.add_contact_point_, other.add_contact_point_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ContactStateChangeMessage)) return false;

      ContactStateChangeMessage otherMyClass = (ContactStateChangeMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.rigid_body_hash_code_ != otherMyClass.rigid_body_hash_code_) return false;

      if(this.add_contact_point_ != otherMyClass.add_contact_point_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ContactStateChangeMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("rigid_body_hash_code=");
      builder.append(this.rigid_body_hash_code_);      builder.append(", ");
      builder.append("add_contact_point=");
      builder.append(this.add_contact_point_);
      builder.append("}");
      return builder.toString();
   }
}
