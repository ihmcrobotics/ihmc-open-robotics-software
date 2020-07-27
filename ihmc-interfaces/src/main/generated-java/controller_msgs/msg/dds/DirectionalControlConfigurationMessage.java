package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message contains the control behavior for Directional Control.
       */
public class DirectionalControlConfigurationMessage extends Packet<DirectionalControlConfigurationMessage> implements Settable<DirectionalControlConfigurationMessage>, EpsilonComparable<DirectionalControlConfigurationMessage>
{
   public long sequence_id_;
   /**
            * Set to true to cause the robot to walk. Setting this to false will cause the robot to stop walking.
            * However, the robot will still finish the current step.
            */
   public boolean enable_walking_;
   /**
            * Specify the profile name to use. Profiles contain definitions of walking and stepping
            * parameters that modify how aggressive or conservative steps will be.
            * Assuming it exists, directional control will be initialized with the "default" profile.
            * If this field is left empty, no alteration will be made to the profile.
            * By default, profiles are stored in the directory ~/.ihmc/joystick_step_app and are
            * in .ini format.
            * For more details, see UserProfileManager and JoystickStepParameters.
            */
   public java.lang.StringBuilder profile_name_;

   public DirectionalControlConfigurationMessage()
   {
      profile_name_ = new java.lang.StringBuilder(255);
   }

   public DirectionalControlConfigurationMessage(DirectionalControlConfigurationMessage other)
   {
      this();
      set(other);
   }

   public void set(DirectionalControlConfigurationMessage other)
   {
      sequence_id_ = other.sequence_id_;

      enable_walking_ = other.enable_walking_;

      profile_name_.setLength(0);
      profile_name_.append(other.profile_name_);

   }

   public void setSequenceId(long sequence_id)
   {
      sequence_id_ = sequence_id;
   }
   public long getSequenceId()
   {
      return sequence_id_;
   }

   /**
            * Set to true to cause the robot to walk. Setting this to false will cause the robot to stop walking.
            * However, the robot will still finish the current step.
            */
   public void setEnableWalking(boolean enable_walking)
   {
      enable_walking_ = enable_walking;
   }
   /**
            * Set to true to cause the robot to walk. Setting this to false will cause the robot to stop walking.
            * However, the robot will still finish the current step.
            */
   public boolean getEnableWalking()
   {
      return enable_walking_;
   }

   /**
            * Specify the profile name to use. Profiles contain definitions of walking and stepping
            * parameters that modify how aggressive or conservative steps will be.
            * Assuming it exists, directional control will be initialized with the "default" profile.
            * If this field is left empty, no alteration will be made to the profile.
            * By default, profiles are stored in the directory ~/.ihmc/joystick_step_app and are
            * in .ini format.
            * For more details, see UserProfileManager and JoystickStepParameters.
            */
   public void setProfileName(java.lang.String profile_name)
   {
      profile_name_.setLength(0);
      profile_name_.append(profile_name);
   }

   /**
            * Specify the profile name to use. Profiles contain definitions of walking and stepping
            * parameters that modify how aggressive or conservative steps will be.
            * Assuming it exists, directional control will be initialized with the "default" profile.
            * If this field is left empty, no alteration will be made to the profile.
            * By default, profiles are stored in the directory ~/.ihmc/joystick_step_app and are
            * in .ini format.
            * For more details, see UserProfileManager and JoystickStepParameters.
            */
   public java.lang.String getProfileNameAsString()
   {
      return getProfileName().toString();
   }
   /**
            * Specify the profile name to use. Profiles contain definitions of walking and stepping
            * parameters that modify how aggressive or conservative steps will be.
            * Assuming it exists, directional control will be initialized with the "default" profile.
            * If this field is left empty, no alteration will be made to the profile.
            * By default, profiles are stored in the directory ~/.ihmc/joystick_step_app and are
            * in .ini format.
            * For more details, see UserProfileManager and JoystickStepParameters.
            */
   public java.lang.StringBuilder getProfileName()
   {
      return profile_name_;
   }


   public static Supplier<DirectionalControlConfigurationMessagePubSubType> getPubSubType()
   {
      return DirectionalControlConfigurationMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return DirectionalControlConfigurationMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(DirectionalControlConfigurationMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.enable_walking_, other.enable_walking_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.profile_name_, other.profile_name_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof DirectionalControlConfigurationMessage)) return false;

      DirectionalControlConfigurationMessage otherMyClass = (DirectionalControlConfigurationMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.enable_walking_ != otherMyClass.enable_walking_) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.profile_name_, otherMyClass.profile_name_)) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("DirectionalControlConfigurationMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("enable_walking=");
      builder.append(this.enable_walking_);      builder.append(", ");
      builder.append("profile_name=");
      builder.append(this.profile_name_);
      builder.append("}");
      return builder.toString();
   }
}
