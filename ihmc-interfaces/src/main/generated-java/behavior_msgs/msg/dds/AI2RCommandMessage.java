package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class AI2RCommandMessage extends Packet<AI2RCommandMessage> implements Settable<AI2RCommandMessage>, EpsilonComparable<AI2RCommandMessage>
{
   /**
            * Behavior to execute (checkpoint to jump to in the pre-loaded behavior collection)
            */
   public java.lang.StringBuilder behavior_to_execute_;
   public boolean adapting_behavior_;
   public behavior_msgs.msg.dds.AI2RHandPoseAdaptationMessage hand_pose_adaptation_;
   public behavior_msgs.msg.dds.AI2RNavigationMessage navigation_;
   public behavior_msgs.msg.dds.AI2RReceiveObjectMessage receive_object_;
   public behavior_msgs.msg.dds.AI2RScanMessage scan_;

   public AI2RCommandMessage()
   {
      behavior_to_execute_ = new java.lang.StringBuilder(255);
      hand_pose_adaptation_ = new behavior_msgs.msg.dds.AI2RHandPoseAdaptationMessage();
      navigation_ = new behavior_msgs.msg.dds.AI2RNavigationMessage();
      receive_object_ = new behavior_msgs.msg.dds.AI2RReceiveObjectMessage();
      scan_ = new behavior_msgs.msg.dds.AI2RScanMessage();
   }

   public AI2RCommandMessage(AI2RCommandMessage other)
   {
      this();
      set(other);
   }

   public void set(AI2RCommandMessage other)
   {
      behavior_to_execute_.setLength(0);
      behavior_to_execute_.append(other.behavior_to_execute_);

      adapting_behavior_ = other.adapting_behavior_;

      behavior_msgs.msg.dds.AI2RHandPoseAdaptationMessagePubSubType.staticCopy(other.hand_pose_adaptation_, hand_pose_adaptation_);
      behavior_msgs.msg.dds.AI2RNavigationMessagePubSubType.staticCopy(other.navigation_, navigation_);
      behavior_msgs.msg.dds.AI2RReceiveObjectMessagePubSubType.staticCopy(other.receive_object_, receive_object_);
      behavior_msgs.msg.dds.AI2RScanMessagePubSubType.staticCopy(other.scan_, scan_);
   }

   /**
            * Behavior to execute (checkpoint to jump to in the pre-loaded behavior collection)
            */
   public void setBehaviorToExecute(java.lang.String behavior_to_execute)
   {
      behavior_to_execute_.setLength(0);
      behavior_to_execute_.append(behavior_to_execute);
   }

   /**
            * Behavior to execute (checkpoint to jump to in the pre-loaded behavior collection)
            */
   public java.lang.String getBehaviorToExecuteAsString()
   {
      return getBehaviorToExecute().toString();
   }
   /**
            * Behavior to execute (checkpoint to jump to in the pre-loaded behavior collection)
            */
   public java.lang.StringBuilder getBehaviorToExecute()
   {
      return behavior_to_execute_;
   }

   public void setAdaptingBehavior(boolean adapting_behavior)
   {
      adapting_behavior_ = adapting_behavior;
   }
   public boolean getAdaptingBehavior()
   {
      return adapting_behavior_;
   }


   public behavior_msgs.msg.dds.AI2RHandPoseAdaptationMessage getHandPoseAdaptation()
   {
      return hand_pose_adaptation_;
   }


   public behavior_msgs.msg.dds.AI2RNavigationMessage getNavigation()
   {
      return navigation_;
   }


   public behavior_msgs.msg.dds.AI2RReceiveObjectMessage getReceiveObject()
   {
      return receive_object_;
   }


   public behavior_msgs.msg.dds.AI2RScanMessage getScan()
   {
      return scan_;
   }


   public static Supplier<AI2RCommandMessagePubSubType> getPubSubType()
   {
      return AI2RCommandMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return AI2RCommandMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(AI2RCommandMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.behavior_to_execute_, other.behavior_to_execute_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.adapting_behavior_, other.adapting_behavior_, epsilon)) return false;

      if (!this.hand_pose_adaptation_.epsilonEquals(other.hand_pose_adaptation_, epsilon)) return false;
      if (!this.navigation_.epsilonEquals(other.navigation_, epsilon)) return false;
      if (!this.receive_object_.epsilonEquals(other.receive_object_, epsilon)) return false;
      if (!this.scan_.epsilonEquals(other.scan_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof AI2RCommandMessage)) return false;

      AI2RCommandMessage otherMyClass = (AI2RCommandMessage) other;

      if (!us.ihmc.idl.IDLTools.equals(this.behavior_to_execute_, otherMyClass.behavior_to_execute_)) return false;

      if(this.adapting_behavior_ != otherMyClass.adapting_behavior_) return false;

      if (!this.hand_pose_adaptation_.equals(otherMyClass.hand_pose_adaptation_)) return false;
      if (!this.navigation_.equals(otherMyClass.navigation_)) return false;
      if (!this.receive_object_.equals(otherMyClass.receive_object_)) return false;
      if (!this.scan_.equals(otherMyClass.scan_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AI2RCommandMessage {");
      builder.append("behavior_to_execute=");
      builder.append(this.behavior_to_execute_);      builder.append(", ");
      builder.append("adapting_behavior=");
      builder.append(this.adapting_behavior_);      builder.append(", ");
      builder.append("hand_pose_adaptation=");
      builder.append(this.hand_pose_adaptation_);      builder.append(", ");
      builder.append("navigation=");
      builder.append(this.navigation_);      builder.append(", ");
      builder.append("receive_object=");
      builder.append(this.receive_object_);      builder.append(", ");
      builder.append("scan=");
      builder.append(this.scan_);
      builder.append("}");
      return builder.toString();
   }
}
