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
   public behavior_msgs.msg.dds.MinimalFootstepMessage[] minimal_footsteps_;

   public MinimalFootstepListMessage()
   {
      minimal_footsteps_ = new behavior_msgs.msg.dds.MinimalFootstepMessage[200];

      for(int i1 = 0; i1 < minimal_footsteps_.length; ++i1)
      {
          minimal_footsteps_[i1] = new behavior_msgs.msg.dds.MinimalFootstepMessage();
      }
   }

   public MinimalFootstepListMessage(MinimalFootstepListMessage other)
   {
      this();
      set(other);
   }

   public void set(MinimalFootstepListMessage other)
   {
      for(int i3 = 0; i3 < minimal_footsteps_.length; ++i3)
      {
            behavior_msgs.msg.dds.MinimalFootstepMessagePubSubType.staticCopy(other.minimal_footsteps_[i3], minimal_footsteps_[i3]);}
   }


   /**
            * List of minimal footsteps
            */
   public behavior_msgs.msg.dds.MinimalFootstepMessage[] getMinimalFootsteps()
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

      for(int i5 = 0; i5 < minimal_footsteps_.length; ++i5)
      {
              if (!this.minimal_footsteps_[i5].epsilonEquals(other.minimal_footsteps_[i5], epsilon)) return false;
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

      for(int i7 = 0; i7 < minimal_footsteps_.length; ++i7)
      {
                if (!this.minimal_footsteps_[i7].equals(otherMyClass.minimal_footsteps_[i7])) return false;
      }
      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("MinimalFootstepListMessage {");
      builder.append("minimal_footsteps=");
      builder.append(java.util.Arrays.toString(this.minimal_footsteps_));
      builder.append("}");
      return builder.toString();
   }
}
