package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * This message enables a box manipulation state.
       */
public class BimanualManipulationMessage extends Packet<BimanualManipulationMessage> implements Settable<BimanualManipulationMessage>, EpsilonComparable<BimanualManipulationMessage>
{
   /**
            * If true, disables the bimanual manipulation manager and the fields below are ignored
            */
   public boolean disable_;
   /**
            * Mass of the object being manipulated
            */
   public double object_mass_ = -1.0;
   /**
            * Lateral squeeze force while manipulating the object. Should approximately be squeeze_force = 0.5 * g * object_mass / friction_coefficient
            */
   public double squeeze_force_ = -1.0;
   /**
            * The squeeze and mass compensation forces ramp up over this duration (if negative a default value is used)
            */
   public double initialize_duration_ = -1.0;
   /**
            * Tracking error - if the distance between the hands varies from their initial distance by more than this value, the manipulation manager stops
            */
   public double acceptable_tracking_error_ = -1.0;

   public BimanualManipulationMessage()
   {
   }

   public BimanualManipulationMessage(BimanualManipulationMessage other)
   {
      this();
      set(other);
   }

   public void set(BimanualManipulationMessage other)
   {
      disable_ = other.disable_;

      object_mass_ = other.object_mass_;

      squeeze_force_ = other.squeeze_force_;

      initialize_duration_ = other.initialize_duration_;

      acceptable_tracking_error_ = other.acceptable_tracking_error_;

   }

   /**
            * If true, disables the bimanual manipulation manager and the fields below are ignored
            */
   public void setDisable(boolean disable)
   {
      disable_ = disable;
   }
   /**
            * If true, disables the bimanual manipulation manager and the fields below are ignored
            */
   public boolean getDisable()
   {
      return disable_;
   }

   /**
            * Mass of the object being manipulated
            */
   public void setObjectMass(double object_mass)
   {
      object_mass_ = object_mass;
   }
   /**
            * Mass of the object being manipulated
            */
   public double getObjectMass()
   {
      return object_mass_;
   }

   /**
            * Lateral squeeze force while manipulating the object. Should approximately be squeeze_force = 0.5 * g * object_mass / friction_coefficient
            */
   public void setSqueezeForce(double squeeze_force)
   {
      squeeze_force_ = squeeze_force;
   }
   /**
            * Lateral squeeze force while manipulating the object. Should approximately be squeeze_force = 0.5 * g * object_mass / friction_coefficient
            */
   public double getSqueezeForce()
   {
      return squeeze_force_;
   }

   /**
            * The squeeze and mass compensation forces ramp up over this duration (if negative a default value is used)
            */
   public void setInitializeDuration(double initialize_duration)
   {
      initialize_duration_ = initialize_duration;
   }
   /**
            * The squeeze and mass compensation forces ramp up over this duration (if negative a default value is used)
            */
   public double getInitializeDuration()
   {
      return initialize_duration_;
   }

   /**
            * Tracking error - if the distance between the hands varies from their initial distance by more than this value, the manipulation manager stops
            */
   public void setAcceptableTrackingError(double acceptable_tracking_error)
   {
      acceptable_tracking_error_ = acceptable_tracking_error;
   }
   /**
            * Tracking error - if the distance between the hands varies from their initial distance by more than this value, the manipulation manager stops
            */
   public double getAcceptableTrackingError()
   {
      return acceptable_tracking_error_;
   }


   public static Supplier<BimanualManipulationMessagePubSubType> getPubSubType()
   {
      return BimanualManipulationMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return BimanualManipulationMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(BimanualManipulationMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.disable_, other.disable_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.object_mass_, other.object_mass_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.squeeze_force_, other.squeeze_force_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.initialize_duration_, other.initialize_duration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.acceptable_tracking_error_, other.acceptable_tracking_error_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof BimanualManipulationMessage)) return false;

      BimanualManipulationMessage otherMyClass = (BimanualManipulationMessage) other;

      if(this.disable_ != otherMyClass.disable_) return false;

      if(this.object_mass_ != otherMyClass.object_mass_) return false;

      if(this.squeeze_force_ != otherMyClass.squeeze_force_) return false;

      if(this.initialize_duration_ != otherMyClass.initialize_duration_) return false;

      if(this.acceptable_tracking_error_ != otherMyClass.acceptable_tracking_error_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BimanualManipulationMessage {");
      builder.append("disable=");
      builder.append(this.disable_);      builder.append(", ");
      builder.append("object_mass=");
      builder.append(this.object_mass_);      builder.append(", ");
      builder.append("squeeze_force=");
      builder.append(this.squeeze_force_);      builder.append(", ");
      builder.append("initialize_duration=");
      builder.append(this.initialize_duration_);      builder.append(", ");
      builder.append("acceptable_tracking_error=");
      builder.append(this.acceptable_tracking_error_);
      builder.append("}");
      return builder.toString();
   }
}
