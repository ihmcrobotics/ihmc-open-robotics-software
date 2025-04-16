package unitree_h_one_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class LowCmd extends Packet<LowCmd> implements Settable<LowCmd>, EpsilonComparable<LowCmd>
{
   public byte mode_pr_;
   public byte mode_machine_;
   public unitree_h_one_msgs.msg.dds.MotorCmd[] motor_cmd_;
   public long crc_;

   public LowCmd()
   {
      motor_cmd_ = new unitree_h_one_msgs.msg.dds.MotorCmd[35];

      for(int i1 = 0; i1 < motor_cmd_.length; ++i1)
      {
          motor_cmd_[i1] = new unitree_h_one_msgs.msg.dds.MotorCmd();
      }
   }

   public LowCmd(LowCmd other)
   {
      this();
      set(other);
   }

   public void set(LowCmd other)
   {
      mode_pr_ = other.mode_pr_;

      mode_machine_ = other.mode_machine_;

      for(int i3 = 0; i3 < motor_cmd_.length; ++i3)
      {
            unitree_h_one_msgs.msg.dds.MotorCmdPubSubType.staticCopy(other.motor_cmd_[i3], motor_cmd_[i3]);}

      crc_ = other.crc_;

   }

   public void setModePr(byte mode_pr)
   {
      mode_pr_ = mode_pr;
   }
   public byte getModePr()
   {
      return mode_pr_;
   }

   public void setModeMachine(byte mode_machine)
   {
      mode_machine_ = mode_machine;
   }
   public byte getModeMachine()
   {
      return mode_machine_;
   }


   public unitree_h_one_msgs.msg.dds.MotorCmd[] getMotorCmd()
   {
      return motor_cmd_;
   }

   public void setCrc(long crc)
   {
      crc_ = crc;
   }
   public long getCrc()
   {
      return crc_;
   }


   public static Supplier<LowCmdPubSubType> getPubSubType()
   {
      return LowCmdPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return LowCmdPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(LowCmd other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.mode_pr_, other.mode_pr_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.mode_machine_, other.mode_machine_, epsilon)) return false;

      for(int i5 = 0; i5 < motor_cmd_.length; ++i5)
      {
              if (!this.motor_cmd_[i5].epsilonEquals(other.motor_cmd_[i5], epsilon)) return false;
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.crc_, other.crc_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof LowCmd)) return false;

      LowCmd otherMyClass = (LowCmd) other;

      if(this.mode_pr_ != otherMyClass.mode_pr_) return false;

      if(this.mode_machine_ != otherMyClass.mode_machine_) return false;

      for(int i7 = 0; i7 < motor_cmd_.length; ++i7)
      {
                if (!this.motor_cmd_[i7].equals(otherMyClass.motor_cmd_[i7])) return false;
      }
      if(this.crc_ != otherMyClass.crc_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("LowCmd {");
      builder.append("mode_pr=");
      builder.append(this.mode_pr_);      builder.append(", ");
      builder.append("mode_machine=");
      builder.append(this.mode_machine_);      builder.append(", ");
      builder.append("motor_cmd=");
      builder.append(java.util.Arrays.toString(this.motor_cmd_));      builder.append(", ");
      builder.append("crc=");
      builder.append(this.crc_);
      builder.append("}");
      return builder.toString();
   }
}
