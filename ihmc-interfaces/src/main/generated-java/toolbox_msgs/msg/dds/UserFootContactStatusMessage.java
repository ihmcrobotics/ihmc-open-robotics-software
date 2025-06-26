package toolbox_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * UserFootContactStatusMessage.msg
       * This message communicates the state of the user's feet as detected by VR trackers.
       * The state is true if the user's foot is considered to be in contact with the ground.
       */
public class UserFootContactStatusMessage extends Packet<UserFootContactStatusMessage> implements Settable<UserFootContactStatusMessage>, EpsilonComparable<UserFootContactStatusMessage>
{
   public boolean left_foot_in_contact_;
   public boolean right_foot_in_contact_;

   public UserFootContactStatusMessage()
   {
   }

   public UserFootContactStatusMessage(UserFootContactStatusMessage other)
   {
      this();
      set(other);
   }

   public void set(UserFootContactStatusMessage other)
   {
      left_foot_in_contact_ = other.left_foot_in_contact_;

      right_foot_in_contact_ = other.right_foot_in_contact_;

   }

   public void setLeftFootInContact(boolean left_foot_in_contact)
   {
      left_foot_in_contact_ = left_foot_in_contact;
   }
   public boolean getLeftFootInContact()
   {
      return left_foot_in_contact_;
   }

   public void setRightFootInContact(boolean right_foot_in_contact)
   {
      right_foot_in_contact_ = right_foot_in_contact;
   }
   public boolean getRightFootInContact()
   {
      return right_foot_in_contact_;
   }


   public static Supplier<UserFootContactStatusMessagePubSubType> getPubSubType()
   {
      return UserFootContactStatusMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return UserFootContactStatusMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(UserFootContactStatusMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.left_foot_in_contact_, other.left_foot_in_contact_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.right_foot_in_contact_, other.right_foot_in_contact_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof UserFootContactStatusMessage)) return false;

      UserFootContactStatusMessage otherMyClass = (UserFootContactStatusMessage) other;

      if(this.left_foot_in_contact_ != otherMyClass.left_foot_in_contact_) return false;

      if(this.right_foot_in_contact_ != otherMyClass.right_foot_in_contact_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("UserFootContactStatusMessage {");
      builder.append("left_foot_in_contact=");
      builder.append(this.left_foot_in_contact_);      builder.append(", ");
      builder.append("right_foot_in_contact=");
      builder.append(this.right_foot_in_contact_);
      builder.append("}");
      return builder.toString();
   }
}
