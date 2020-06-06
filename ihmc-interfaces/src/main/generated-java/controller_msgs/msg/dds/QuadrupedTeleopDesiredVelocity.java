package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC quadruped controller API.
       * This message sends a desired stepping velocity to the step teleop module.
       */
public class QuadrupedTeleopDesiredVelocity extends Packet<QuadrupedTeleopDesiredVelocity> implements Settable<QuadrupedTeleopDesiredVelocity>, EpsilonComparable<QuadrupedTeleopDesiredVelocity>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public double desired_x_velocity_;

   public double desired_y_velocity_;

   public double desired_yaw_velocity_;

   public QuadrupedTeleopDesiredVelocity()
   {





   }

   public QuadrupedTeleopDesiredVelocity(QuadrupedTeleopDesiredVelocity other)
   {
      this();
      set(other);
   }

   public void set(QuadrupedTeleopDesiredVelocity other)
   {

      sequence_id_ = other.sequence_id_;


      desired_x_velocity_ = other.desired_x_velocity_;


      desired_y_velocity_ = other.desired_y_velocity_;


      desired_yaw_velocity_ = other.desired_yaw_velocity_;

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


   public void setDesiredXVelocity(double desired_x_velocity)
   {
      desired_x_velocity_ = desired_x_velocity;
   }
   public double getDesiredXVelocity()
   {
      return desired_x_velocity_;
   }


   public void setDesiredYVelocity(double desired_y_velocity)
   {
      desired_y_velocity_ = desired_y_velocity;
   }
   public double getDesiredYVelocity()
   {
      return desired_y_velocity_;
   }


   public void setDesiredYawVelocity(double desired_yaw_velocity)
   {
      desired_yaw_velocity_ = desired_yaw_velocity;
   }
   public double getDesiredYawVelocity()
   {
      return desired_yaw_velocity_;
   }


   public static Supplier<QuadrupedTeleopDesiredVelocityPubSubType> getPubSubType()
   {
      return QuadrupedTeleopDesiredVelocityPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return QuadrupedTeleopDesiredVelocityPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(QuadrupedTeleopDesiredVelocity other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.desired_x_velocity_, other.desired_x_velocity_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.desired_y_velocity_, other.desired_y_velocity_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.desired_yaw_velocity_, other.desired_yaw_velocity_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof QuadrupedTeleopDesiredVelocity)) return false;

      QuadrupedTeleopDesiredVelocity otherMyClass = (QuadrupedTeleopDesiredVelocity) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.desired_x_velocity_ != otherMyClass.desired_x_velocity_) return false;


      if(this.desired_y_velocity_ != otherMyClass.desired_y_velocity_) return false;


      if(this.desired_yaw_velocity_ != otherMyClass.desired_yaw_velocity_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("QuadrupedTeleopDesiredVelocity {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("desired_x_velocity=");
      builder.append(this.desired_x_velocity_);      builder.append(", ");

      builder.append("desired_y_velocity=");
      builder.append(this.desired_y_velocity_);      builder.append(", ");

      builder.append("desired_yaw_velocity=");
      builder.append(this.desired_yaw_velocity_);
      builder.append("}");
      return builder.toString();
   }
}
