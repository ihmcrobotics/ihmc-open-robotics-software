package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class MinimalFootstepListMessage extends Packet<MinimalFootstepListMessage> implements Settable<MinimalFootstepListMessage>, EpsilonComparable<MinimalFootstepListMessage>
{
   /**
            * List of minimal footsteps
            */
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.MinimalFootstepMessage>  minimal_footsteps_;

   public MinimalFootstepListMessage()
   {
      minimal_footsteps_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.MinimalFootstepMessage> (200, new behavior_msgs.msg.dds.MinimalFootstepMessagePubSubType());

   }

   public MinimalFootstepListMessage(MinimalFootstepListMessage other)
   {
      this();
      set(other);
   }

   public void set(MinimalFootstepListMessage other)
   {
      minimal_footsteps_.set(other.minimal_footsteps_);
   }


   /**
            * List of minimal footsteps
            */
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.MinimalFootstepMessage>  getMinimalFootsteps()
   {
      return minimal_footsteps_;
   }


   public static Supplier<MinimalFootstepListMessagePubSubType> getPubSubType()
   {
      return MinimalFootstepListMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return MinimalFootstepListMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(MinimalFootstepListMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (this.minimal_footsteps_.size() != other.minimal_footsteps_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.minimal_footsteps_.size(); i++)
         {  if (!this.minimal_footsteps_.get(i).epsilonEquals(other.minimal_footsteps_.get(i), epsilon)) return false; }
      }

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof MinimalFootstepListMessage)) return false;

      MinimalFootstepListMessage otherMyClass = (MinimalFootstepListMessage) other;

      if (!this.minimal_footsteps_.equals(otherMyClass.minimal_footsteps_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("MinimalFootstepListMessage {");
      builder.append("minimal_footsteps=");
      builder.append(this.minimal_footsteps_);
      builder.append("}");
      return builder.toString();
   }
}
