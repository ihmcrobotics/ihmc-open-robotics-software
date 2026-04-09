package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Input message for the PDVelocityBasedGoalReacher.
       * This tells the goal reacher to stop trying to walk to a goal.
       */
public class ControllerReleaseGoalMessage extends Packet<ControllerReleaseGoalMessage> implements Settable<ControllerReleaseGoalMessage>, EpsilonComparable<ControllerReleaseGoalMessage>
{
   public boolean release_goal_;

   public ControllerReleaseGoalMessage()
   {
   }

   public ControllerReleaseGoalMessage(ControllerReleaseGoalMessage other)
   {
      this();
      set(other);
   }

   public void set(ControllerReleaseGoalMessage other)
   {
      release_goal_ = other.release_goal_;

   }

   public void setReleaseGoal(boolean release_goal)
   {
      release_goal_ = release_goal;
   }
   public boolean getReleaseGoal()
   {
      return release_goal_;
   }


   public static Supplier<ControllerReleaseGoalMessagePubSubType> getPubSubType()
   {
      return ControllerReleaseGoalMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ControllerReleaseGoalMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ControllerReleaseGoalMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.release_goal_, other.release_goal_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ControllerReleaseGoalMessage)) return false;

      ControllerReleaseGoalMessage otherMyClass = (ControllerReleaseGoalMessage) other;

      if(this.release_goal_ != otherMyClass.release_goal_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ControllerReleaseGoalMessage {");
      builder.append("release_goal=");
      builder.append(this.release_goal_);
      builder.append("}");
      return builder.toString();
   }
}
