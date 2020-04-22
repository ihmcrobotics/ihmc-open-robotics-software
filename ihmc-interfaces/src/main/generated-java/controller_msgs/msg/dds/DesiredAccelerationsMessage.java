package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * General message that carries desired joint accelerations.
       * It is used by ArmDesiredAccelerationsMessage, SpineDesiredAccelerationsMessage, NeckDesiredAccelerationsMessage.
       */
public class DesiredAccelerationsMessage extends Packet<DesiredAccelerationsMessage> implements Settable<DesiredAccelerationsMessage>, EpsilonComparable<DesiredAccelerationsMessage>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   /**
            * Specifies the desired joint accelerations.
            */
   public us.ihmc.idl.IDLSequence.Double  desired_joint_accelerations_;

   /**
            * Properties for queueing commands.
            */
   public controller_msgs.msg.dds.QueueableMessage queueing_properties_;

   public DesiredAccelerationsMessage()
   {


      desired_joint_accelerations_ = new us.ihmc.idl.IDLSequence.Double (100, "type_6");


      queueing_properties_ = new controller_msgs.msg.dds.QueueableMessage();

   }

   public DesiredAccelerationsMessage(DesiredAccelerationsMessage other)
   {
      this();
      set(other);
   }

   public void set(DesiredAccelerationsMessage other)
   {

      sequence_id_ = other.sequence_id_;


      desired_joint_accelerations_.set(other.desired_joint_accelerations_);

      controller_msgs.msg.dds.QueueableMessagePubSubType.staticCopy(other.queueing_properties_, queueing_properties_);
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
            * Specifies the desired joint accelerations.
            */
   public us.ihmc.idl.IDLSequence.Double  getDesiredJointAccelerations()
   {
      return desired_joint_accelerations_;
   }



   /**
            * Properties for queueing commands.
            */
   public controller_msgs.msg.dds.QueueableMessage getQueueingProperties()
   {
      return queueing_properties_;
   }


   public static Supplier<DesiredAccelerationsMessagePubSubType> getPubSubType()
   {
      return DesiredAccelerationsMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return DesiredAccelerationsMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(DesiredAccelerationsMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsDoubleSequence(this.desired_joint_accelerations_, other.desired_joint_accelerations_, epsilon)) return false;


      if (!this.queueing_properties_.epsilonEquals(other.queueing_properties_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof DesiredAccelerationsMessage)) return false;

      DesiredAccelerationsMessage otherMyClass = (DesiredAccelerationsMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if (!this.desired_joint_accelerations_.equals(otherMyClass.desired_joint_accelerations_)) return false;

      if (!this.queueing_properties_.equals(otherMyClass.queueing_properties_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("DesiredAccelerationsMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("desired_joint_accelerations=");
      builder.append(this.desired_joint_accelerations_);      builder.append(", ");

      builder.append("queueing_properties=");
      builder.append(this.queueing_properties_);
      builder.append("}");
      return builder.toString();
   }
}
