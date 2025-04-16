package unitree_go_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class MotorCmds extends Packet<MotorCmds> implements Settable<MotorCmds>, EpsilonComparable<MotorCmds>
{
   public us.ihmc.idl.IDLSequence.Object<unitree_go_msgs.msg.dds.MotorCmd>  cmds_;

   public MotorCmds()
   {
      cmds_ = new us.ihmc.idl.IDLSequence.Object<unitree_go_msgs.msg.dds.MotorCmd> (100, new unitree_go_msgs.msg.dds.MotorCmdPubSubType());

   }

   public MotorCmds(MotorCmds other)
   {
      this();
      set(other);
   }

   public void set(MotorCmds other)
   {
      cmds_.set(other.cmds_);
   }


   public us.ihmc.idl.IDLSequence.Object<unitree_go_msgs.msg.dds.MotorCmd>  getCmds()
   {
      return cmds_;
   }


   public static Supplier<MotorCmdsPubSubType> getPubSubType()
   {
      return MotorCmdsPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return MotorCmdsPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(MotorCmds other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (this.cmds_.size() != other.cmds_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.cmds_.size(); i++)
         {  if (!this.cmds_.get(i).epsilonEquals(other.cmds_.get(i), epsilon)) return false; }
      }

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof MotorCmds)) return false;

      MotorCmds otherMyClass = (MotorCmds) other;

      if (!this.cmds_.equals(otherMyClass.cmds_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("MotorCmds {");
      builder.append("cmds=");
      builder.append(this.cmds_);
      builder.append("}");
      return builder.toString();
   }
}
