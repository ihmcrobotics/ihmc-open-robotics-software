package unitree_h_one_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class HandCmd extends Packet<HandCmd> implements Settable<HandCmd>, EpsilonComparable<HandCmd>
{
   public us.ihmc.idl.IDLSequence.Object<unitree_h_one_msgs.msg.dds.MotorCmd>  motor_cmd_;

   public HandCmd()
   {
      motor_cmd_ = new us.ihmc.idl.IDLSequence.Object<unitree_h_one_msgs.msg.dds.MotorCmd> (100, new unitree_h_one_msgs.msg.dds.MotorCmdPubSubType());

   }

   public HandCmd(HandCmd other)
   {
      this();
      set(other);
   }

   public void set(HandCmd other)
   {
      motor_cmd_.set(other.motor_cmd_);
   }


   public us.ihmc.idl.IDLSequence.Object<unitree_h_one_msgs.msg.dds.MotorCmd>  getMotorCmd()
   {
      return motor_cmd_;
   }


   public static Supplier<HandCmdPubSubType> getPubSubType()
   {
      return HandCmdPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return HandCmdPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(HandCmd other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (this.motor_cmd_.size() != other.motor_cmd_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.motor_cmd_.size(); i++)
         {  if (!this.motor_cmd_.get(i).epsilonEquals(other.motor_cmd_.get(i), epsilon)) return false; }
      }

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof HandCmd)) return false;

      HandCmd otherMyClass = (HandCmd) other;

      if (!this.motor_cmd_.equals(otherMyClass.motor_cmd_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("HandCmd {");
      builder.append("motor_cmd=");
      builder.append(this.motor_cmd_);
      builder.append("}");
      return builder.toString();
   }
}
