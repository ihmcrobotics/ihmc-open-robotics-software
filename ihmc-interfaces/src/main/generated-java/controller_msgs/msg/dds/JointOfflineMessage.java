package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * This message notifies the controller of a joint going offline, which causes the controller to go into an emergency fall-prevention mode.
       */
public class JointOfflineMessage extends Packet<JointOfflineMessage> implements Settable<JointOfflineMessage>, EpsilonComparable<JointOfflineMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Hash-code of the joint which is offline. The hash-code is computed from OneDoFJoint#hashcode().
            */
   public int joint_offline_hash_code_;

   public JointOfflineMessage()
   {
   }

   public JointOfflineMessage(JointOfflineMessage other)
   {
      this();
      set(other);
   }

   public void set(JointOfflineMessage other)
   {
      sequence_id_ = other.sequence_id_;

      joint_offline_hash_code_ = other.joint_offline_hash_code_;

   }

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public void setSequenceId(long sequence_id)
   {
      sequence_id_ = sequence_id;
   }
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long getSequenceId()
   {
      return sequence_id_;
   }

   /**
            * Hash-code of the joint which is offline. The hash-code is computed from OneDoFJoint#hashcode().
            */
   public void setJointOfflineHashCode(int joint_offline_hash_code)
   {
      joint_offline_hash_code_ = joint_offline_hash_code;
   }
   /**
            * Hash-code of the joint which is offline. The hash-code is computed from OneDoFJoint#hashcode().
            */
   public int getJointOfflineHashCode()
   {
      return joint_offline_hash_code_;
   }


   public static Supplier<JointOfflineMessagePubSubType> getPubSubType()
   {
      return JointOfflineMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return JointOfflineMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(JointOfflineMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.joint_offline_hash_code_, other.joint_offline_hash_code_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof JointOfflineMessage)) return false;

      JointOfflineMessage otherMyClass = (JointOfflineMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.joint_offline_hash_code_ != otherMyClass.joint_offline_hash_code_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("JointOfflineMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("joint_offline_hash_code=");
      builder.append(this.joint_offline_hash_code_);
      builder.append("}");
      return builder.toString();
   }
}
