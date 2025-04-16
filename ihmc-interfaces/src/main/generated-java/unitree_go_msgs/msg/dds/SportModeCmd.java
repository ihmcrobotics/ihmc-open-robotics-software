package unitree_go_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class SportModeCmd extends Packet<SportModeCmd> implements Settable<SportModeCmd>, EpsilonComparable<SportModeCmd>
{
   public byte mode_;
   public byte gait_type_;
   public byte speed_level_;
   public float foot_raise_height_;
   public float body_height_;
   public float[] position_;
   public float[] euler_;
   public float[] velocity_;
   public float yaw_speed_;
   public unitree_go_msgs.msg.dds.BmsCmd bms_cmd_;
   public unitree_go_msgs.msg.dds.PathPoint[] path_point_;

   public SportModeCmd()
   {
      position_ = new float[2];

      euler_ = new float[3];

      velocity_ = new float[2];

      bms_cmd_ = new unitree_go_msgs.msg.dds.BmsCmd();
      path_point_ = new unitree_go_msgs.msg.dds.PathPoint[30];

      for(int i1 = 0; i1 < path_point_.length; ++i1)
      {
          path_point_[i1] = new unitree_go_msgs.msg.dds.PathPoint();
      }
   }

   public SportModeCmd(SportModeCmd other)
   {
      this();
      set(other);
   }

   public void set(SportModeCmd other)
   {
      mode_ = other.mode_;

      gait_type_ = other.gait_type_;

      speed_level_ = other.speed_level_;

      foot_raise_height_ = other.foot_raise_height_;

      body_height_ = other.body_height_;

      for(int i3 = 0; i3 < position_.length; ++i3)
      {
            position_[i3] = other.position_[i3];

      }

      for(int i5 = 0; i5 < euler_.length; ++i5)
      {
            euler_[i5] = other.euler_[i5];

      }

      for(int i7 = 0; i7 < velocity_.length; ++i7)
      {
            velocity_[i7] = other.velocity_[i7];

      }

      yaw_speed_ = other.yaw_speed_;

      unitree_go_msgs.msg.dds.BmsCmdPubSubType.staticCopy(other.bms_cmd_, bms_cmd_);
      for(int i9 = 0; i9 < path_point_.length; ++i9)
      {
            unitree_go_msgs.msg.dds.PathPointPubSubType.staticCopy(other.path_point_[i9], path_point_[i9]);}

   }

   public void setMode(byte mode)
   {
      mode_ = mode;
   }
   public byte getMode()
   {
      return mode_;
   }

   public void setGaitType(byte gait_type)
   {
      gait_type_ = gait_type;
   }
   public byte getGaitType()
   {
      return gait_type_;
   }

   public void setSpeedLevel(byte speed_level)
   {
      speed_level_ = speed_level;
   }
   public byte getSpeedLevel()
   {
      return speed_level_;
   }

   public void setFootRaiseHeight(float foot_raise_height)
   {
      foot_raise_height_ = foot_raise_height;
   }
   public float getFootRaiseHeight()
   {
      return foot_raise_height_;
   }

   public void setBodyHeight(float body_height)
   {
      body_height_ = body_height;
   }
   public float getBodyHeight()
   {
      return body_height_;
   }


   public float[] getPosition()
   {
      return position_;
   }


   public float[] getEuler()
   {
      return euler_;
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


   public unitree_go_msgs.msg.dds.BmsCmd getBmsCmd()
   {
      return bms_cmd_;
   }


   public unitree_go_msgs.msg.dds.PathPoint[] getPathPoint()
   {
      return path_point_;
   }


   public static Supplier<SportModeCmdPubSubType> getPubSubType()
   {
      return SportModeCmdPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return SportModeCmdPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(SportModeCmd other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.mode_, other.mode_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.gait_type_, other.gait_type_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.speed_level_, other.speed_level_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.foot_raise_height_, other.foot_raise_height_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.body_height_, other.body_height_, epsilon)) return false;

      for(int i11 = 0; i11 < position_.length; ++i11)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.position_[i11], other.position_[i11], epsilon)) return false;
      }

      for(int i13 = 0; i13 < euler_.length; ++i13)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.euler_[i13], other.euler_[i13], epsilon)) return false;
      }

      for(int i15 = 0; i15 < velocity_.length; ++i15)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.velocity_[i15], other.velocity_[i15], epsilon)) return false;
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.yaw_speed_, other.yaw_speed_, epsilon)) return false;

      if (!this.bms_cmd_.epsilonEquals(other.bms_cmd_, epsilon)) return false;
      for(int i17 = 0; i17 < path_point_.length; ++i17)
      {
              if (!this.path_point_[i17].epsilonEquals(other.path_point_[i17], epsilon)) return false;
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof SportModeCmd)) return false;

      SportModeCmd otherMyClass = (SportModeCmd) other;

      if(this.mode_ != otherMyClass.mode_) return false;

      if(this.gait_type_ != otherMyClass.gait_type_) return false;

      if(this.speed_level_ != otherMyClass.speed_level_) return false;

      if(this.foot_raise_height_ != otherMyClass.foot_raise_height_) return false;

      if(this.body_height_ != otherMyClass.body_height_) return false;

      for(int i19 = 0; i19 < position_.length; ++i19)
      {
                if(this.position_[i19] != otherMyClass.position_[i19]) return false;

      }
      for(int i21 = 0; i21 < euler_.length; ++i21)
      {
                if(this.euler_[i21] != otherMyClass.euler_[i21]) return false;

      }
      for(int i23 = 0; i23 < velocity_.length; ++i23)
      {
                if(this.velocity_[i23] != otherMyClass.velocity_[i23]) return false;

      }
      if(this.yaw_speed_ != otherMyClass.yaw_speed_) return false;

      if (!this.bms_cmd_.equals(otherMyClass.bms_cmd_)) return false;
      for(int i25 = 0; i25 < path_point_.length; ++i25)
      {
                if (!this.path_point_[i25].equals(otherMyClass.path_point_[i25])) return false;
      }

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("SportModeCmd {");
      builder.append("mode=");
      builder.append(this.mode_);      builder.append(", ");
      builder.append("gait_type=");
      builder.append(this.gait_type_);      builder.append(", ");
      builder.append("speed_level=");
      builder.append(this.speed_level_);      builder.append(", ");
      builder.append("foot_raise_height=");
      builder.append(this.foot_raise_height_);      builder.append(", ");
      builder.append("body_height=");
      builder.append(this.body_height_);      builder.append(", ");
      builder.append("position=");
      builder.append(java.util.Arrays.toString(this.position_));      builder.append(", ");
      builder.append("euler=");
      builder.append(java.util.Arrays.toString(this.euler_));      builder.append(", ");
      builder.append("velocity=");
      builder.append(java.util.Arrays.toString(this.velocity_));      builder.append(", ");
      builder.append("yaw_speed=");
      builder.append(this.yaw_speed_);      builder.append(", ");
      builder.append("bms_cmd=");
      builder.append(this.bms_cmd_);      builder.append(", ");
      builder.append("path_point=");
      builder.append(java.util.Arrays.toString(this.path_point_));
      builder.append("}");
      return builder.toString();
   }
}
