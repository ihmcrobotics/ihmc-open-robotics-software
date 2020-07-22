package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message contains the control behavior for Joystick Remote Control.
       */
public class JoystickRemoteControlMessage extends Packet<JoystickRemoteControlMessage> implements Settable<JoystickRemoteControlMessage>, EpsilonComparable<JoystickRemoteControlMessage>
{
   /**
            * Set to true to cause the robot to walk. Setting this to false will cause the robot to stop walking.
            * However, the robot will still finish the current step.
            */
   public boolean enable_walking_;
   /**
            * Specify the profile name to use. Profiles contain definitions of walking and stepping
            * parameters that modify how aggressive or conservative joystick steps will be.
            * Assuming it exists, joystick remote control will be initialized with the "default" profile.
            * If this field is left empty, no alteration will be made to the profile.
            * By default, profiles are stored in the directory ~/.ihmc/joystick_step_app and are
            * in .ini format.
            * For more details, see UserProfileManager and JoystickStepParameters.
            */
   public java.lang.StringBuilder profile_name_;

   public JoystickRemoteControlMessage()
   {
      profile_name_ = new java.lang.StringBuilder(255);
   }

   public JoystickRemoteControlMessage(JoystickRemoteControlMessage other)
   {
      this();
      set(other);
   }

   public void set(JoystickRemoteControlMessage other)
   {
      enable_walking_ = other.enable_walking_;

      profile_name_.setLength(0);
      profile_name_.append(other.profile_name_);

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
            * parameters that modify how aggressive or conservative joystick steps will be.
            * Assuming it exists, joystick remote control will be initialized with the "default" profile.
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
            * parameters that modify how aggressive or conservative joystick steps will be.
            * Assuming it exists, joystick remote control will be initialized with the "default" profile.
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
            * parameters that modify how aggressive or conservative joystick steps will be.
            * Assuming it exists, joystick remote control will be initialized with the "default" profile.
            * If this field is left empty, no alteration will be made to the profile.
            * By default, profiles are stored in the directory ~/.ihmc/joystick_step_app and are
            * in .ini format.
            * For more details, see UserProfileManager and JoystickStepParameters.
            */
   public java.lang.StringBuilder getProfileName()
   {
      return profile_name_;
   }


   public static Supplier<JoystickRemoteControlMessagePubSubType> getPubSubType()
   {
      return JoystickRemoteControlMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return JoystickRemoteControlMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(JoystickRemoteControlMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.enable_walking_, other.enable_walking_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.profile_name_, other.profile_name_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof JoystickRemoteControlMessage)) return false;

      JoystickRemoteControlMessage otherMyClass = (JoystickRemoteControlMessage) other;

      if(this.enable_walking_ != otherMyClass.enable_walking_) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.profile_name_, otherMyClass.profile_name_)) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("JoystickRemoteControlMessage {");
      builder.append("enable_walking=");
      builder.append(this.enable_walking_);      builder.append(", ");
      builder.append("profile_name=");
      builder.append(this.profile_name_);
      builder.append("}");
      return builder.toString();
   }
}
