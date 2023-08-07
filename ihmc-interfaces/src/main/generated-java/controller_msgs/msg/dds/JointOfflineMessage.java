package controller_msgs.msg.dds;

import java.util.function.Supplier;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.pubsub.TopicDataType;

/**
 * This message is part of the IHMC whole-body controller API. Message carrying the information
 * needed to enable load bearing for a end-effector. A contact point will be used to enable this
 * feature. This point is attached to the end-effector.
 */
public class JointOfflineMessage extends Packet<JointOfflineMessage> implements Settable<JointOfflineMessage>, EpsilonComparable<JointOfflineMessage>
{
   /**
    * Unique ID used to identify this message, should preferably be consecutively increasing.
    */
   public long sequence_id_;

//   public OneDoFJointBasics jointToGoOffline;

   /**
    * If set to true this will load the contact point. Otherwise the rigid body will stop bearing load.
    */

   public JointOfflineMessage()
   {
      //TODO
   }

   public JointOfflineMessage(JointOfflineMessage other)
   {
      this();
      set(other);
   }

   public void set(JointOfflineMessage other)
   {
      sequence_id_ = other.sequence_id_;

      //TODO
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

//   /**
//    * If set to true this will load the contact point. Otherwise the rigid body will stop bearing load.
//    */
//   public void setJointToGoOffline(OneDofJointBasics jointToGoOffline)
//   {
//      jointToGoOffline_ = jointToGoOffline;
//   }
//
//   /**
//    * If set to true this will load the contact point. Otherwise the rigid body will stop bearing load.
//    */
//   public OneDofJointBasics getJointToGoOffline()
//   {
//      return jointToGoOffline_;
//   }

   public static Supplier<LoadBearingMessagePubSubType> getPubSubType()
   {
      return LoadBearingMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return LoadBearingMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(JointOfflineMessage other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon))
         return false;
      //TODO Update this
      //      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.load_, other.load_, epsilon))
      //         return false;
      //
      //      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.coefficient_of_friction_, other.coefficient_of_friction_, epsilon))
      //         return false;
      //
      //      if (!this.body_frame_to_contact_frame_.epsilonEquals(other.body_frame_to_contact_frame_, epsilon))
      //         return false;
      //      if (!this.contact_normal_in_world_frame_.epsilonEquals(other.contact_normal_in_world_frame_, epsilon))
      //         return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;
      if (!(other instanceof JointOfflineMessage))
         return false;

      JointOfflineMessage otherMyClass = (JointOfflineMessage) other;

      if (this.sequence_id_ != otherMyClass.sequence_id_)
         return false;

      //TODO update
      //      if (this.load_ != otherMyClass.load_)
      //         return false;
      //
      //      if (this.coefficient_of_friction_ != otherMyClass.coefficient_of_friction_)
      //         return false;
      //
      //      if (!this.body_frame_to_contact_frame_.equals(otherMyClass.body_frame_to_contact_frame_))
      //         return false;
      //      if (!this.contact_normal_in_world_frame_.equals(otherMyClass.contact_normal_in_world_frame_))
      //         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();
      //TODO Update this
      builder.append("JointOfflineMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);
      //            builder.append(", ");
      //            builder.append("load=");
      //            builder.append(this.load_);
      //            builder.append(", ");
      //            builder.append("coefficient_of_friction=");
      //            builder.append(this.coefficient_of_friction_);
      //            builder.append(", ");
      //            builder.append("body_frame_to_contact_frame=");
      //            builder.append(this.body_frame_to_contact_frame_);
      //            builder.append(", ");
      //            builder.append("contact_normal_in_world_frame=");
      //            builder.append(this.contact_normal_in_world_frame_);
      builder.append("}");
      return builder.toString();
   }
}
