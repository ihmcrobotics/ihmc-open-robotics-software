package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Message responsible for sending desired velocity setpoints (forward, lateral, and turning) to walking
       * controllers that take this form of input
       */
public class VelocityBasedWalkingInputMessage extends Packet<VelocityBasedWalkingInputMessage> implements Settable<VelocityBasedWalkingInputMessage>, EpsilonComparable<VelocityBasedWalkingInputMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * If true, start stepping. If false, stand in place
            */
   public boolean walk_;
   /**
            * Desired velocity setpoints
            */
   public double forward_velocity_;
   public double lateral_velocity_;
   public double turn_velocity_;
   /**
            * If true, desired velocity setpoints are normalized from a range of [-1.0, 1.0]
            * If false, desired velocity setpoints are sent in m/s
            */
   public boolean are_velocities_normalized_;

   public VelocityBasedWalkingInputMessage()
   {
   }

   public VelocityBasedWalkingInputMessage(VelocityBasedWalkingInputMessage other)
   {
      this();
      set(other);
   }

   public void set(VelocityBasedWalkingInputMessage other)
   {
      sequence_id_ = other.sequence_id_;

      walk_ = other.walk_;

      forward_velocity_ = other.forward_velocity_;

      lateral_velocity_ = other.lateral_velocity_;

      turn_velocity_ = other.turn_velocity_;

      are_velocities_normalized_ = other.are_velocities_normalized_;

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
            * If true, start stepping. If false, stand in place
            */
   public void setWalk(boolean walk)
   {
      walk_ = walk;
   }
   /**
            * If true, start stepping. If false, stand in place
            */
   public boolean getWalk()
   {
      return walk_;
   }

   /**
            * Desired velocity setpoints
            */
   public void setForwardVelocity(double forward_velocity)
   {
      forward_velocity_ = forward_velocity;
   }
   /**
            * Desired velocity setpoints
            */
   public double getForwardVelocity()
   {
      return forward_velocity_;
   }

   public void setLateralVelocity(double lateral_velocity)
   {
      lateral_velocity_ = lateral_velocity;
   }
   public double getLateralVelocity()
   {
      return lateral_velocity_;
   }

   public void setTurnVelocity(double turn_velocity)
   {
      turn_velocity_ = turn_velocity;
   }
   public double getTurnVelocity()
   {
      return turn_velocity_;
   }

   /**
            * If true, desired velocity setpoints are normalized from a range of [-1.0, 1.0]
            * If false, desired velocity setpoints are sent in m/s
            */
   public void setAreVelocitiesNormalized(boolean are_velocities_normalized)
   {
      are_velocities_normalized_ = are_velocities_normalized;
   }
   /**
            * If true, desired velocity setpoints are normalized from a range of [-1.0, 1.0]
            * If false, desired velocity setpoints are sent in m/s
            */
   public boolean getAreVelocitiesNormalized()
   {
      return are_velocities_normalized_;
   }


   public static Supplier<VelocityBasedWalkingInputMessagePubSubType> getPubSubType()
   {
      return VelocityBasedWalkingInputMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return VelocityBasedWalkingInputMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(VelocityBasedWalkingInputMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.walk_, other.walk_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.forward_velocity_, other.forward_velocity_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.lateral_velocity_, other.lateral_velocity_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.turn_velocity_, other.turn_velocity_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.are_velocities_normalized_, other.are_velocities_normalized_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof VelocityBasedWalkingInputMessage)) return false;

      VelocityBasedWalkingInputMessage otherMyClass = (VelocityBasedWalkingInputMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.walk_ != otherMyClass.walk_) return false;

      if(this.forward_velocity_ != otherMyClass.forward_velocity_) return false;

      if(this.lateral_velocity_ != otherMyClass.lateral_velocity_) return false;

      if(this.turn_velocity_ != otherMyClass.turn_velocity_) return false;

      if(this.are_velocities_normalized_ != otherMyClass.are_velocities_normalized_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("VelocityBasedWalkingInputMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("walk=");
      builder.append(this.walk_);      builder.append(", ");
      builder.append("forward_velocity=");
      builder.append(this.forward_velocity_);      builder.append(", ");
      builder.append("lateral_velocity=");
      builder.append(this.lateral_velocity_);      builder.append(", ");
      builder.append("turn_velocity=");
      builder.append(this.turn_velocity_);      builder.append(", ");
      builder.append("are_velocities_normalized=");
      builder.append(this.are_velocities_normalized_);
      builder.append("}");
      return builder.toString();
   }
}
