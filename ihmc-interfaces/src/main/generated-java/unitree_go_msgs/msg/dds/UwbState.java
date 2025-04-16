package unitree_go_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class UwbState extends Packet<UwbState> implements Settable<UwbState>, EpsilonComparable<UwbState>
{
   public byte[] version_;
   public byte channel_;
   public byte joy_mode_;
   public float orientation_est_;
   public float pitch_est_;
   public float distance_est_;
   public float yaw_est_;
   public float tag_roll_;
   public float tag_pitch_;
   public float tag_yaw_;
   public float base_roll_;
   public float base_pitch_;
   public float base_yaw_;
   public float[] joystick_;
   public byte error_state_;
   public byte buttons_;
   public byte enabled_from_app_;

   public UwbState()
   {
      version_ = new byte[2];

      joystick_ = new float[2];

   }

   public UwbState(UwbState other)
   {
      this();
      set(other);
   }

   public void set(UwbState other)
   {
      for(int i1 = 0; i1 < version_.length; ++i1)
      {
            version_[i1] = other.version_[i1];

      }

      channel_ = other.channel_;

      joy_mode_ = other.joy_mode_;

      orientation_est_ = other.orientation_est_;

      pitch_est_ = other.pitch_est_;

      distance_est_ = other.distance_est_;

      yaw_est_ = other.yaw_est_;

      tag_roll_ = other.tag_roll_;

      tag_pitch_ = other.tag_pitch_;

      tag_yaw_ = other.tag_yaw_;

      base_roll_ = other.base_roll_;

      base_pitch_ = other.base_pitch_;

      base_yaw_ = other.base_yaw_;

      for(int i3 = 0; i3 < joystick_.length; ++i3)
      {
            joystick_[i3] = other.joystick_[i3];

      }

      error_state_ = other.error_state_;

      buttons_ = other.buttons_;

      enabled_from_app_ = other.enabled_from_app_;

   }


   public byte[] getVersion()
   {
      return version_;
   }

   public void setChannel(byte channel)
   {
      channel_ = channel;
   }
   public byte getChannel()
   {
      return channel_;
   }

   public void setJoyMode(byte joy_mode)
   {
      joy_mode_ = joy_mode;
   }
   public byte getJoyMode()
   {
      return joy_mode_;
   }

   public void setOrientationEst(float orientation_est)
   {
      orientation_est_ = orientation_est;
   }
   public float getOrientationEst()
   {
      return orientation_est_;
   }

   public void setPitchEst(float pitch_est)
   {
      pitch_est_ = pitch_est;
   }
   public float getPitchEst()
   {
      return pitch_est_;
   }

   public void setDistanceEst(float distance_est)
   {
      distance_est_ = distance_est;
   }
   public float getDistanceEst()
   {
      return distance_est_;
   }

   public void setYawEst(float yaw_est)
   {
      yaw_est_ = yaw_est;
   }
   public float getYawEst()
   {
      return yaw_est_;
   }

   public void setTagRoll(float tag_roll)
   {
      tag_roll_ = tag_roll;
   }
   public float getTagRoll()
   {
      return tag_roll_;
   }

   public void setTagPitch(float tag_pitch)
   {
      tag_pitch_ = tag_pitch;
   }
   public float getTagPitch()
   {
      return tag_pitch_;
   }

   public void setTagYaw(float tag_yaw)
   {
      tag_yaw_ = tag_yaw;
   }
   public float getTagYaw()
   {
      return tag_yaw_;
   }

   public void setBaseRoll(float base_roll)
   {
      base_roll_ = base_roll;
   }
   public float getBaseRoll()
   {
      return base_roll_;
   }

   public void setBasePitch(float base_pitch)
   {
      base_pitch_ = base_pitch;
   }
   public float getBasePitch()
   {
      return base_pitch_;
   }

   public void setBaseYaw(float base_yaw)
   {
      base_yaw_ = base_yaw;
   }
   public float getBaseYaw()
   {
      return base_yaw_;
   }


   public float[] getJoystick()
   {
      return joystick_;
   }

   public void setErrorState(byte error_state)
   {
      error_state_ = error_state;
   }
   public byte getErrorState()
   {
      return error_state_;
   }

   public void setButtons(byte buttons)
   {
      buttons_ = buttons;
   }
   public byte getButtons()
   {
      return buttons_;
   }

   public void setEnabledFromApp(byte enabled_from_app)
   {
      enabled_from_app_ = enabled_from_app;
   }
   public byte getEnabledFromApp()
   {
      return enabled_from_app_;
   }


   public static Supplier<UwbStatePubSubType> getPubSubType()
   {
      return UwbStatePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return UwbStatePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(UwbState other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      for(int i5 = 0; i5 < version_.length; ++i5)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.version_[i5], other.version_[i5], epsilon)) return false;
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.channel_, other.channel_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.joy_mode_, other.joy_mode_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.orientation_est_, other.orientation_est_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.pitch_est_, other.pitch_est_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.distance_est_, other.distance_est_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.yaw_est_, other.yaw_est_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.tag_roll_, other.tag_roll_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.tag_pitch_, other.tag_pitch_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.tag_yaw_, other.tag_yaw_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.base_roll_, other.base_roll_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.base_pitch_, other.base_pitch_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.base_yaw_, other.base_yaw_, epsilon)) return false;

      for(int i7 = 0; i7 < joystick_.length; ++i7)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.joystick_[i7], other.joystick_[i7], epsilon)) return false;
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.error_state_, other.error_state_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.buttons_, other.buttons_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.enabled_from_app_, other.enabled_from_app_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof UwbState)) return false;

      UwbState otherMyClass = (UwbState) other;

      for(int i9 = 0; i9 < version_.length; ++i9)
      {
                if(this.version_[i9] != otherMyClass.version_[i9]) return false;

      }
      if(this.channel_ != otherMyClass.channel_) return false;

      if(this.joy_mode_ != otherMyClass.joy_mode_) return false;

      if(this.orientation_est_ != otherMyClass.orientation_est_) return false;

      if(this.pitch_est_ != otherMyClass.pitch_est_) return false;

      if(this.distance_est_ != otherMyClass.distance_est_) return false;

      if(this.yaw_est_ != otherMyClass.yaw_est_) return false;

      if(this.tag_roll_ != otherMyClass.tag_roll_) return false;

      if(this.tag_pitch_ != otherMyClass.tag_pitch_) return false;

      if(this.tag_yaw_ != otherMyClass.tag_yaw_) return false;

      if(this.base_roll_ != otherMyClass.base_roll_) return false;

      if(this.base_pitch_ != otherMyClass.base_pitch_) return false;

      if(this.base_yaw_ != otherMyClass.base_yaw_) return false;

      for(int i11 = 0; i11 < joystick_.length; ++i11)
      {
                if(this.joystick_[i11] != otherMyClass.joystick_[i11]) return false;

      }
      if(this.error_state_ != otherMyClass.error_state_) return false;

      if(this.buttons_ != otherMyClass.buttons_) return false;

      if(this.enabled_from_app_ != otherMyClass.enabled_from_app_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("UwbState {");
      builder.append("version=");
      builder.append(java.util.Arrays.toString(this.version_));      builder.append(", ");
      builder.append("channel=");
      builder.append(this.channel_);      builder.append(", ");
      builder.append("joy_mode=");
      builder.append(this.joy_mode_);      builder.append(", ");
      builder.append("orientation_est=");
      builder.append(this.orientation_est_);      builder.append(", ");
      builder.append("pitch_est=");
      builder.append(this.pitch_est_);      builder.append(", ");
      builder.append("distance_est=");
      builder.append(this.distance_est_);      builder.append(", ");
      builder.append("yaw_est=");
      builder.append(this.yaw_est_);      builder.append(", ");
      builder.append("tag_roll=");
      builder.append(this.tag_roll_);      builder.append(", ");
      builder.append("tag_pitch=");
      builder.append(this.tag_pitch_);      builder.append(", ");
      builder.append("tag_yaw=");
      builder.append(this.tag_yaw_);      builder.append(", ");
      builder.append("base_roll=");
      builder.append(this.base_roll_);      builder.append(", ");
      builder.append("base_pitch=");
      builder.append(this.base_pitch_);      builder.append(", ");
      builder.append("base_yaw=");
      builder.append(this.base_yaw_);      builder.append(", ");
      builder.append("joystick=");
      builder.append(java.util.Arrays.toString(this.joystick_));      builder.append(", ");
      builder.append("error_state=");
      builder.append(this.error_state_);      builder.append(", ");
      builder.append("buttons=");
      builder.append(this.buttons_);      builder.append(", ");
      builder.append("enabled_from_app=");
      builder.append(this.enabled_from_app_);
      builder.append("}");
      return builder.toString();
   }
}
