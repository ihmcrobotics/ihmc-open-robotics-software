package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * Message carrying the information needed to enable load bearing for a end-effector.
       * A contact point will be used to enable this feature. This point is attached to the end-effector.
       */
public class LoadBearingMessage extends Packet<LoadBearingMessage> implements Settable<LoadBearingMessage>, EpsilonComparable<LoadBearingMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * If set to true this will load the contact point. Otherwise the rigid body will stop bearing load.
            */
   public boolean load_;
   /**
            * Sets the coefficient of friction that the controller will use for the contact point.
            */
   public double coefficient_of_friction_;
   /**
            * Pose of the contact frame, expressed in the frame of the end effector body.
            * The contact frame origin is the contact point and the z-axis of contact frame is the contact normal (points into the environment)
            */
   public us.ihmc.euclid.geometry.Pose3D contact_pose_in_body_frame_;

   public LoadBearingMessage()
   {
      contact_pose_in_body_frame_ = new us.ihmc.euclid.geometry.Pose3D();
   }

   public LoadBearingMessage(LoadBearingMessage other)
   {
      this();
      set(other);
   }

   public void set(LoadBearingMessage other)
   {
      sequence_id_ = other.sequence_id_;

      load_ = other.load_;

      coefficient_of_friction_ = other.coefficient_of_friction_;

      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.contact_pose_in_body_frame_, contact_pose_in_body_frame_);
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
            * If set to true this will load the contact point. Otherwise the rigid body will stop bearing load.
            */
   public void setLoad(boolean load)
   {
      load_ = load;
   }
   /**
            * If set to true this will load the contact point. Otherwise the rigid body will stop bearing load.
            */
   public boolean getLoad()
   {
      return load_;
   }

   /**
            * Sets the coefficient of friction that the controller will use for the contact point.
            */
   public void setCoefficientOfFriction(double coefficient_of_friction)
   {
      coefficient_of_friction_ = coefficient_of_friction;
   }
   /**
            * Sets the coefficient of friction that the controller will use for the contact point.
            */
   public double getCoefficientOfFriction()
   {
      return coefficient_of_friction_;
   }


   /**
            * Pose of the contact frame, expressed in the frame of the end effector body.
            * The contact frame origin is the contact point and the z-axis of contact frame is the contact normal (points into the environment)
            */
   public us.ihmc.euclid.geometry.Pose3D getContactPoseInBodyFrame()
   {
      return contact_pose_in_body_frame_;
   }


   public static Supplier<LoadBearingMessagePubSubType> getPubSubType()
   {
      return LoadBearingMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return LoadBearingMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(LoadBearingMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.load_, other.load_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.coefficient_of_friction_, other.coefficient_of_friction_, epsilon)) return false;

      if (!this.contact_pose_in_body_frame_.epsilonEquals(other.contact_pose_in_body_frame_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof LoadBearingMessage)) return false;

      LoadBearingMessage otherMyClass = (LoadBearingMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.load_ != otherMyClass.load_) return false;

      if(this.coefficient_of_friction_ != otherMyClass.coefficient_of_friction_) return false;

      if (!this.contact_pose_in_body_frame_.equals(otherMyClass.contact_pose_in_body_frame_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("LoadBearingMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("load=");
      builder.append(this.load_);      builder.append(", ");
      builder.append("coefficient_of_friction=");
      builder.append(this.coefficient_of_friction_);      builder.append(", ");
      builder.append("contact_pose_in_body_frame=");
      builder.append(this.contact_pose_in_body_frame_);
      builder.append("}");
      return builder.toString();
   }
}
