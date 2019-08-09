package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the KinematicsStreamingToolbox API.
       */
public class KinematicsStreamingToolboxInputMessage extends Packet<KinematicsStreamingToolboxInputMessage> implements Settable<KinematicsStreamingToolboxInputMessage>, EpsilonComparable<KinematicsStreamingToolboxInputMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * When false, the toolbox will only publish a status with KinematicsToolboxOutputStatus such that the user can validate
            * that the solver is working properly.
            * When true, the toolbox will stream to the IHMC walking controller the desired robot configuration. The status mentioned is
            * still published so the user can compare the solver desired state against the actual robot state.
            */
   public boolean stream_to_controller_;
   /**
            * The list of inputs the solver is to be tracking.
            * When streaming inputs from a VR UI environment, it is convenient to use the fields control_frame_position_in_end_effector and
            * control_frame_orientation_in_end_effector from KinematicsToolboxRigidBodyMessage to adjust the user's controllers with respect
            * to the robot's end-effectors.
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage>  inputs_;

   public KinematicsStreamingToolboxInputMessage()
   {
      inputs_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage> (10, new controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessagePubSubType());

   }

   public KinematicsStreamingToolboxInputMessage(KinematicsStreamingToolboxInputMessage other)
   {
      this();
      set(other);
   }

   public void set(KinematicsStreamingToolboxInputMessage other)
   {
      sequence_id_ = other.sequence_id_;

      stream_to_controller_ = other.stream_to_controller_;

      inputs_.set(other.inputs_);
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
            * When false, the toolbox will only publish a status with KinematicsToolboxOutputStatus such that the user can validate
            * that the solver is working properly.
            * When true, the toolbox will stream to the IHMC walking controller the desired robot configuration. The status mentioned is
            * still published so the user can compare the solver desired state against the actual robot state.
            */
   public void setStreamToController(boolean stream_to_controller)
   {
      stream_to_controller_ = stream_to_controller;
   }
   /**
            * When false, the toolbox will only publish a status with KinematicsToolboxOutputStatus such that the user can validate
            * that the solver is working properly.
            * When true, the toolbox will stream to the IHMC walking controller the desired robot configuration. The status mentioned is
            * still published so the user can compare the solver desired state against the actual robot state.
            */
   public boolean getStreamToController()
   {
      return stream_to_controller_;
   }


   /**
            * The list of inputs the solver is to be tracking.
            * When streaming inputs from a VR UI environment, it is convenient to use the fields control_frame_position_in_end_effector and
            * control_frame_orientation_in_end_effector from KinematicsToolboxRigidBodyMessage to adjust the user's controllers with respect
            * to the robot's end-effectors.
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage>  getInputs()
   {
      return inputs_;
   }


   public static Supplier<KinematicsStreamingToolboxInputMessagePubSubType> getPubSubType()
   {
      return KinematicsStreamingToolboxInputMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return KinematicsStreamingToolboxInputMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(KinematicsStreamingToolboxInputMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.stream_to_controller_, other.stream_to_controller_, epsilon)) return false;

      if (this.inputs_.size() != other.inputs_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.inputs_.size(); i++)
         {  if (!this.inputs_.get(i).epsilonEquals(other.inputs_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof KinematicsStreamingToolboxInputMessage)) return false;

      KinematicsStreamingToolboxInputMessage otherMyClass = (KinematicsStreamingToolboxInputMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.stream_to_controller_ != otherMyClass.stream_to_controller_) return false;

      if (!this.inputs_.equals(otherMyClass.inputs_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("KinematicsStreamingToolboxInputMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("stream_to_controller=");
      builder.append(this.stream_to_controller_);      builder.append(", ");
      builder.append("inputs=");
      builder.append(this.inputs_);
      builder.append("}");
      return builder.toString();
   }
}
