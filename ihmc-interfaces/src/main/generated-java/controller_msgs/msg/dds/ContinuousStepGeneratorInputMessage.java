package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Input message for the ContinuousStepGenerator.
       */
public class ContinuousStepGeneratorInputMessage extends Packet<ContinuousStepGeneratorInputMessage> implements Settable<ContinuousStepGeneratorInputMessage>, EpsilonComparable<ContinuousStepGeneratorInputMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   public boolean walk_;
   public double forward_velocity_;
   public double lateral_velocity_;
   public double turn_velocity_;
   public boolean unit_velocities_;

   public ContinuousStepGeneratorInputMessage()
   {
   }

   public ContinuousStepGeneratorInputMessage(ContinuousStepGeneratorInputMessage other)
   {
      this();
      set(other);
   }

   public void set(ContinuousStepGeneratorInputMessage other)
   {
      sequence_id_ = other.sequence_id_;

      walk_ = other.walk_;

      forward_velocity_ = other.forward_velocity_;

      lateral_velocity_ = other.lateral_velocity_;

      turn_velocity_ = other.turn_velocity_;

      unit_velocities_ = other.unit_velocities_;

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

   public void setWalk(boolean walk)
   {
      walk_ = walk;
   }
   public boolean getWalk()
   {
      return walk_;
   }

   public void setForwardVelocity(double forward_velocity)
   {
      forward_velocity_ = forward_velocity;
   }
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

   public void setUnitVelocities(boolean unit_velocities)
   {
      unit_velocities_ = unit_velocities;
   }
   public boolean getUnitVelocities()
   {
      return unit_velocities_;
   }


   public static Supplier<ContinuousStepGeneratorInputMessagePubSubType> getPubSubType()
   {
      return ContinuousStepGeneratorInputMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ContinuousStepGeneratorInputMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ContinuousStepGeneratorInputMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.walk_, other.walk_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.forward_velocity_, other.forward_velocity_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.lateral_velocity_, other.lateral_velocity_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.turn_velocity_, other.turn_velocity_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.unit_velocities_, other.unit_velocities_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ContinuousStepGeneratorInputMessage)) return false;

      ContinuousStepGeneratorInputMessage otherMyClass = (ContinuousStepGeneratorInputMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.walk_ != otherMyClass.walk_) return false;

      if(this.forward_velocity_ != otherMyClass.forward_velocity_) return false;

      if(this.lateral_velocity_ != otherMyClass.lateral_velocity_) return false;

      if(this.turn_velocity_ != otherMyClass.turn_velocity_) return false;

      if(this.unit_velocities_ != otherMyClass.unit_velocities_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ContinuousStepGeneratorInputMessage {");
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
      builder.append("unit_velocities=");
      builder.append(this.unit_velocities_);
      builder.append("}");
      return builder.toString();
   }
}
