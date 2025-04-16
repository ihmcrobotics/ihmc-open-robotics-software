package unitree_go_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class MotorStates extends Packet<MotorStates> implements Settable<MotorStates>, EpsilonComparable<MotorStates>
{
   public us.ihmc.idl.IDLSequence.Object<unitree_go_msgs.msg.dds.MotorState>  states_;

   public MotorStates()
   {
      states_ = new us.ihmc.idl.IDLSequence.Object<unitree_go_msgs.msg.dds.MotorState> (100, new unitree_go_msgs.msg.dds.MotorStatePubSubType());

   }

   public MotorStates(MotorStates other)
   {
      this();
      set(other);
   }

   public void set(MotorStates other)
   {
      states_.set(other.states_);
   }


   public us.ihmc.idl.IDLSequence.Object<unitree_go_msgs.msg.dds.MotorState>  getStates()
   {
      return states_;
   }


   public static Supplier<MotorStatesPubSubType> getPubSubType()
   {
      return MotorStatesPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return MotorStatesPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(MotorStates other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (this.states_.size() != other.states_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.states_.size(); i++)
         {  if (!this.states_.get(i).epsilonEquals(other.states_.get(i), epsilon)) return false; }
      }

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof MotorStates)) return false;

      MotorStates otherMyClass = (MotorStates) other;

      if (!this.states_.equals(otherMyClass.states_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("MotorStates {");
      builder.append("states=");
      builder.append(this.states_);
      builder.append("}");
      return builder.toString();
   }
}
