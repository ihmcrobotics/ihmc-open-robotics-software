package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC walking controller preview module: WalkingControllerPreviewToolbox.
       */
public class WalkingControllerPreviewOutputMessage extends Packet<WalkingControllerPreviewOutputMessage> implements Settable<WalkingControllerPreviewOutputMessage>, EpsilonComparable<WalkingControllerPreviewOutputMessage>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   /**
            * Interval in time between two frames.
            */
   public double frame_dt_;

   /**
            * List of configurations for each key frames.
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.KinematicsToolboxOutputStatus>  robot_configurations_;

   public WalkingControllerPreviewOutputMessage()
   {



      robot_configurations_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.KinematicsToolboxOutputStatus> (1000, new controller_msgs.msg.dds.KinematicsToolboxOutputStatusPubSubType());

   }

   public WalkingControllerPreviewOutputMessage(WalkingControllerPreviewOutputMessage other)
   {
      this();
      set(other);
   }

   public void set(WalkingControllerPreviewOutputMessage other)
   {

      sequence_id_ = other.sequence_id_;


      frame_dt_ = other.frame_dt_;


      robot_configurations_.set(other.robot_configurations_);
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
            * Interval in time between two frames.
            */
   public void setFrameDt(double frame_dt)
   {
      frame_dt_ = frame_dt;
   }
   /**
            * Interval in time between two frames.
            */
   public double getFrameDt()
   {
      return frame_dt_;
   }



   /**
            * List of configurations for each key frames.
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.KinematicsToolboxOutputStatus>  getRobotConfigurations()
   {
      return robot_configurations_;
   }


   public static Supplier<WalkingControllerPreviewOutputMessagePubSubType> getPubSubType()
   {
      return WalkingControllerPreviewOutputMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return WalkingControllerPreviewOutputMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(WalkingControllerPreviewOutputMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.frame_dt_, other.frame_dt_, epsilon)) return false;


      if (this.robot_configurations_.size() != other.robot_configurations_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.robot_configurations_.size(); i++)
         {  if (!this.robot_configurations_.get(i).epsilonEquals(other.robot_configurations_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof WalkingControllerPreviewOutputMessage)) return false;

      WalkingControllerPreviewOutputMessage otherMyClass = (WalkingControllerPreviewOutputMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.frame_dt_ != otherMyClass.frame_dt_) return false;


      if (!this.robot_configurations_.equals(otherMyClass.robot_configurations_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("WalkingControllerPreviewOutputMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("frame_dt=");
      builder.append(this.frame_dt_);      builder.append(", ");

      builder.append("robot_configurations=");
      builder.append(this.robot_configurations_);
      builder.append("}");
      return builder.toString();
   }
}
