package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * Request the controller to use a hand to help supporting the robot weight.
       */
public class HandLoadBearingMessage extends Packet<HandLoadBearingMessage> implements Settable<HandLoadBearingMessage>, EpsilonComparable<HandLoadBearingMessage>
{
   public static final byte ROBOT_SIDE_LEFT = (byte) 0;
   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * The robot side of the hand that will be load bearing.
            */
   public byte robot_side_ = (byte) 255;
   /**
            * If true it will load the contact point, otherwise the hand will stop bearing load.
            */
   public boolean load_;
   /**
            * Sets the coefficient of friction that the controller will use for the contact point (only used if load=true).
            */
   public double coefficient_of_friction_;
   /**
            * Hand contact point expressed in the hand's body-fixed frame (only used if load=true).
            */
   public us.ihmc.euclid.tuple3D.Point3D contact_point_in_body_frame_;
   /**
            * Contact normal in world frame, pointing away from the environment (only used if load=true).
            */
   public us.ihmc.euclid.tuple3D.Vector3D contact_normal_in_world_;

   public HandLoadBearingMessage()
   {
      contact_point_in_body_frame_ = new us.ihmc.euclid.tuple3D.Point3D();
      contact_normal_in_world_ = new us.ihmc.euclid.tuple3D.Vector3D();
   }

   public HandLoadBearingMessage(HandLoadBearingMessage other)
   {
      this();
      set(other);
   }

   public void set(HandLoadBearingMessage other)
   {
      sequence_id_ = other.sequence_id_;

      robot_side_ = other.robot_side_;

      load_ = other.load_;

      coefficient_of_friction_ = other.coefficient_of_friction_;

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.contact_point_in_body_frame_, contact_point_in_body_frame_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.contact_normal_in_world_, contact_normal_in_world_);
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
            * The robot side of the hand that will be load bearing.
            */
   public void setRobotSide(byte robot_side)
   {
      robot_side_ = robot_side;
   }
   /**
            * The robot side of the hand that will be load bearing.
            */
   public byte getRobotSide()
   {
      return robot_side_;
   }

   /**
            * If true it will load the contact point, otherwise the hand will stop bearing load.
            */
   public void setLoad(boolean load)
   {
      load_ = load;
   }
   /**
            * If true it will load the contact point, otherwise the hand will stop bearing load.
            */
   public boolean getLoad()
   {
      return load_;
   }

   /**
            * Sets the coefficient of friction that the controller will use for the contact point (only used if load=true).
            */
   public void setCoefficientOfFriction(double coefficient_of_friction)
   {
      coefficient_of_friction_ = coefficient_of_friction;
   }
   /**
            * Sets the coefficient of friction that the controller will use for the contact point (only used if load=true).
            */
   public double getCoefficientOfFriction()
   {
      return coefficient_of_friction_;
   }


   /**
            * Hand contact point expressed in the hand's body-fixed frame (only used if load=true).
            */
   public us.ihmc.euclid.tuple3D.Point3D getContactPointInBodyFrame()
   {
      return contact_point_in_body_frame_;
   }


   /**
            * Contact normal in world frame, pointing away from the environment (only used if load=true).
            */
   public us.ihmc.euclid.tuple3D.Vector3D getContactNormalInWorld()
   {
      return contact_normal_in_world_;
   }


   public static Supplier<HandLoadBearingMessagePubSubType> getPubSubType()
   {
      return HandLoadBearingMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return HandLoadBearingMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(HandLoadBearingMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.load_, other.load_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.coefficient_of_friction_, other.coefficient_of_friction_, epsilon)) return false;

      if (!this.contact_point_in_body_frame_.epsilonEquals(other.contact_point_in_body_frame_, epsilon)) return false;
      if (!this.contact_normal_in_world_.epsilonEquals(other.contact_normal_in_world_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof HandLoadBearingMessage)) return false;

      HandLoadBearingMessage otherMyClass = (HandLoadBearingMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.robot_side_ != otherMyClass.robot_side_) return false;

      if(this.load_ != otherMyClass.load_) return false;

      if(this.coefficient_of_friction_ != otherMyClass.coefficient_of_friction_) return false;

      if (!this.contact_point_in_body_frame_.equals(otherMyClass.contact_point_in_body_frame_)) return false;
      if (!this.contact_normal_in_world_.equals(otherMyClass.contact_normal_in_world_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("HandLoadBearingMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");
      builder.append("load=");
      builder.append(this.load_);      builder.append(", ");
      builder.append("coefficient_of_friction=");
      builder.append(this.coefficient_of_friction_);      builder.append(", ");
      builder.append("contact_point_in_body_frame=");
      builder.append(this.contact_point_in_body_frame_);      builder.append(", ");
      builder.append("contact_normal_in_world=");
      builder.append(this.contact_normal_in_world_);
      builder.append("}");
      return builder.toString();
   }
}
