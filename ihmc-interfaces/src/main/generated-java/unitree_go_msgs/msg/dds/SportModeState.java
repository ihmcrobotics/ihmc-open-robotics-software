package unitree_go_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class SportModeState extends Packet<SportModeState> implements Settable<SportModeState>, EpsilonComparable<SportModeState>
{
   public unitree_go_msgs.msg.dds.TimeSpec stamp_;
   public long error_code_;
   public unitree_go_msgs.msg.dds.IMUState imu_state_;
   public byte mode_;
   public float progress_;
   public byte gait_type_;
   public float foot_raise_height_;
   public float[] position_;
   public float body_height_;
   public float[] velocity_;
   public float yaw_speed_;
   public float[] range_obstacle_;
   public short[] foot_force_;
   public float[] foot_position_body_;
   public float[] foot_speed_body_;

   public SportModeState()
   {
      stamp_ = new unitree_go_msgs.msg.dds.TimeSpec();
      imu_state_ = new unitree_go_msgs.msg.dds.IMUState();
      position_ = new float[3];

      velocity_ = new float[3];

      range_obstacle_ = new float[4];

      foot_force_ = new short[4];

      foot_position_body_ = new float[12];

      foot_speed_body_ = new float[12];

   }

   public SportModeState(SportModeState other)
   {
      this();
      set(other);
   }

   public void set(SportModeState other)
   {
      unitree_go_msgs.msg.dds.TimeSpecPubSubType.staticCopy(other.stamp_, stamp_);
      error_code_ = other.error_code_;

      unitree_go_msgs.msg.dds.IMUStatePubSubType.staticCopy(other.imu_state_, imu_state_);
      mode_ = other.mode_;

      progress_ = other.progress_;

      gait_type_ = other.gait_type_;

      foot_raise_height_ = other.foot_raise_height_;

      for(int i1 = 0; i1 < position_.length; ++i1)
      {
            position_[i1] = other.position_[i1];

      }

      body_height_ = other.body_height_;

      for(int i3 = 0; i3 < velocity_.length; ++i3)
      {
            velocity_[i3] = other.velocity_[i3];

      }

      yaw_speed_ = other.yaw_speed_;

      for(int i5 = 0; i5 < range_obstacle_.length; ++i5)
      {
            range_obstacle_[i5] = other.range_obstacle_[i5];

      }

      for(int i7 = 0; i7 < foot_force_.length; ++i7)
      {
            foot_force_[i7] = other.foot_force_[i7];

      }

      for(int i9 = 0; i9 < foot_position_body_.length; ++i9)
      {
            foot_position_body_[i9] = other.foot_position_body_[i9];

      }

      for(int i11 = 0; i11 < foot_speed_body_.length; ++i11)
      {
            foot_speed_body_[i11] = other.foot_speed_body_[i11];

      }

   }


   public unitree_go_msgs.msg.dds.TimeSpec getStamp()
   {
      return stamp_;
   }

   public void setErrorCode(long error_code)
   {
      error_code_ = error_code;
   }
   public long getErrorCode()
   {
      return error_code_;
   }


   public unitree_go_msgs.msg.dds.IMUState getImuState()
   {
      return imu_state_;
   }

   public void setMode(byte mode)
   {
      mode_ = mode;
   }
   public byte getMode()
   {
      return mode_;
   }

   public void setProgress(float progress)
   {
      progress_ = progress;
   }
   public float getProgress()
   {
      return progress_;
   }

   public void setGaitType(byte gait_type)
   {
      gait_type_ = gait_type;
   }
   public byte getGaitType()
   {
      return gait_type_;
   }

   public void setFootRaiseHeight(float foot_raise_height)
   {
      foot_raise_height_ = foot_raise_height;
   }
   public float getFootRaiseHeight()
   {
      return foot_raise_height_;
   }


   public float[] getPosition()
   {
      return position_;
   }

   public void setBodyHeight(float body_height)
   {
      body_height_ = body_height;
   }
   public float getBodyHeight()
   {
      return body_height_;
   }


   public float[] getVelocity()
   {
      return velocity_;
   }

   public void setYawSpeed(float yaw_speed)
   {
      yaw_speed_ = yaw_speed;
   }
   public float getYawSpeed()
   {
      return yaw_speed_;
   }


   public float[] getRangeObstacle()
   {
      return range_obstacle_;
   }


   public short[] getFootForce()
   {
      return foot_force_;
   }


   public float[] getFootPositionBody()
   {
      return foot_position_body_;
   }


   public float[] getFootSpeedBody()
   {
      return foot_speed_body_;
   }


   public static Supplier<SportModeStatePubSubType> getPubSubType()
   {
      return SportModeStatePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return SportModeStatePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(SportModeState other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.stamp_.epsilonEquals(other.stamp_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.error_code_, other.error_code_, epsilon)) return false;

      if (!this.imu_state_.epsilonEquals(other.imu_state_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.mode_, other.mode_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.progress_, other.progress_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.gait_type_, other.gait_type_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.foot_raise_height_, other.foot_raise_height_, epsilon)) return false;

      for(int i13 = 0; i13 < position_.length; ++i13)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.position_[i13], other.position_[i13], epsilon)) return false;
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.body_height_, other.body_height_, epsilon)) return false;

      for(int i15 = 0; i15 < velocity_.length; ++i15)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.velocity_[i15], other.velocity_[i15], epsilon)) return false;
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.yaw_speed_, other.yaw_speed_, epsilon)) return false;

      for(int i17 = 0; i17 < range_obstacle_.length; ++i17)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.range_obstacle_[i17], other.range_obstacle_[i17], epsilon)) return false;
      }

      for(int i19 = 0; i19 < foot_force_.length; ++i19)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.foot_force_[i19], other.foot_force_[i19], epsilon)) return false;
      }

      for(int i21 = 0; i21 < foot_position_body_.length; ++i21)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.foot_position_body_[i21], other.foot_position_body_[i21], epsilon)) return false;
      }

      for(int i23 = 0; i23 < foot_speed_body_.length; ++i23)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.foot_speed_body_[i23], other.foot_speed_body_[i23], epsilon)) return false;
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof SportModeState)) return false;

      SportModeState otherMyClass = (SportModeState) other;

      if (!this.stamp_.equals(otherMyClass.stamp_)) return false;
      if(this.error_code_ != otherMyClass.error_code_) return false;

      if (!this.imu_state_.equals(otherMyClass.imu_state_)) return false;
      if(this.mode_ != otherMyClass.mode_) return false;

      if(this.progress_ != otherMyClass.progress_) return false;

      if(this.gait_type_ != otherMyClass.gait_type_) return false;

      if(this.foot_raise_height_ != otherMyClass.foot_raise_height_) return false;

      for(int i25 = 0; i25 < position_.length; ++i25)
      {
                if(this.position_[i25] != otherMyClass.position_[i25]) return false;

      }
      if(this.body_height_ != otherMyClass.body_height_) return false;

      for(int i27 = 0; i27 < velocity_.length; ++i27)
      {
                if(this.velocity_[i27] != otherMyClass.velocity_[i27]) return false;

      }
      if(this.yaw_speed_ != otherMyClass.yaw_speed_) return false;

      for(int i29 = 0; i29 < range_obstacle_.length; ++i29)
      {
                if(this.range_obstacle_[i29] != otherMyClass.range_obstacle_[i29]) return false;

      }
      for(int i31 = 0; i31 < foot_force_.length; ++i31)
      {
                if(this.foot_force_[i31] != otherMyClass.foot_force_[i31]) return false;

      }
      for(int i33 = 0; i33 < foot_position_body_.length; ++i33)
      {
                if(this.foot_position_body_[i33] != otherMyClass.foot_position_body_[i33]) return false;

      }
      for(int i35 = 0; i35 < foot_speed_body_.length; ++i35)
      {
                if(this.foot_speed_body_[i35] != otherMyClass.foot_speed_body_[i35]) return false;

      }

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("SportModeState {");
      builder.append("stamp=");
      builder.append(this.stamp_);      builder.append(", ");
      builder.append("error_code=");
      builder.append(this.error_code_);      builder.append(", ");
      builder.append("imu_state=");
      builder.append(this.imu_state_);      builder.append(", ");
      builder.append("mode=");
      builder.append(this.mode_);      builder.append(", ");
      builder.append("progress=");
      builder.append(this.progress_);      builder.append(", ");
      builder.append("gait_type=");
      builder.append(this.gait_type_);      builder.append(", ");
      builder.append("foot_raise_height=");
      builder.append(this.foot_raise_height_);      builder.append(", ");
      builder.append("position=");
      builder.append(java.util.Arrays.toString(this.position_));      builder.append(", ");
      builder.append("body_height=");
      builder.append(this.body_height_);      builder.append(", ");
      builder.append("velocity=");
      builder.append(java.util.Arrays.toString(this.velocity_));      builder.append(", ");
      builder.append("yaw_speed=");
      builder.append(this.yaw_speed_);      builder.append(", ");
      builder.append("range_obstacle=");
      builder.append(java.util.Arrays.toString(this.range_obstacle_));      builder.append(", ");
      builder.append("foot_force=");
      builder.append(java.util.Arrays.toString(this.foot_force_));      builder.append(", ");
      builder.append("foot_position_body=");
      builder.append(java.util.Arrays.toString(this.foot_position_body_));      builder.append(", ");
      builder.append("foot_speed_body=");
      builder.append(java.util.Arrays.toString(this.foot_speed_body_));
      builder.append("}");
      return builder.toString();
   }
}
