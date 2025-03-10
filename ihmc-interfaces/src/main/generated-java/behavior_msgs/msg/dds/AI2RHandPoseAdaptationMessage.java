package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class AI2RHandPoseAdaptationMessage extends Packet<AI2RHandPoseAdaptationMessage> implements Settable<AI2RHandPoseAdaptationMessage>, EpsilonComparable<AI2RHandPoseAdaptationMessage>
{
   /**
            * Name of the action to adapt
            */
   public java.lang.StringBuilder action_name_;
   /**
            * Hand Pose action - Reference frame for the action
            */
   public java.lang.StringBuilder reference_frame_name_;
   /**
            * Hand Pose action - The position of the hand specified in reference_frame_name
            */
   public us.ihmc.euclid.tuple3D.Point3D new_position_;
   /**
            * Hand Pose action - The orientation of the hand specified in reference_frame_name
            */
   public us.ihmc.euclid.tuple4D.Quaternion new_orientation_;

   public AI2RHandPoseAdaptationMessage()
   {
      action_name_ = new java.lang.StringBuilder(255);
      reference_frame_name_ = new java.lang.StringBuilder(255);
      new_position_ = new us.ihmc.euclid.tuple3D.Point3D();
      new_orientation_ = new us.ihmc.euclid.tuple4D.Quaternion();
   }

   public AI2RHandPoseAdaptationMessage(AI2RHandPoseAdaptationMessage other)
   {
      this();
      set(other);
   }

   public void set(AI2RHandPoseAdaptationMessage other)
   {
      action_name_.setLength(0);
      action_name_.append(other.action_name_);

      reference_frame_name_.setLength(0);
      reference_frame_name_.append(other.reference_frame_name_);

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.new_position_, new_position_);
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.new_orientation_, new_orientation_);
   }

   /**
            * Name of the action to adapt
            */
   public void setActionName(java.lang.String action_name)
   {
      action_name_.setLength(0);
      action_name_.append(action_name);
   }

   /**
            * Name of the action to adapt
            */
   public java.lang.String getActionNameAsString()
   {
      return getActionName().toString();
   }
   /**
            * Name of the action to adapt
            */
   public java.lang.StringBuilder getActionName()
   {
      return action_name_;
   }

   /**
            * Hand Pose action - Reference frame for the action
            */
   public void setReferenceFrameName(java.lang.String reference_frame_name)
   {
      reference_frame_name_.setLength(0);
      reference_frame_name_.append(reference_frame_name);
   }

   /**
            * Hand Pose action - Reference frame for the action
            */
   public java.lang.String getReferenceFrameNameAsString()
   {
      return getReferenceFrameName().toString();
   }
   /**
            * Hand Pose action - Reference frame for the action
            */
   public java.lang.StringBuilder getReferenceFrameName()
   {
      return reference_frame_name_;
   }


   /**
            * Hand Pose action - The position of the hand specified in reference_frame_name
            */
   public us.ihmc.euclid.tuple3D.Point3D getNewPosition()
   {
      return new_position_;
   }


   /**
            * Hand Pose action - The orientation of the hand specified in reference_frame_name
            */
   public us.ihmc.euclid.tuple4D.Quaternion getNewOrientation()
   {
      return new_orientation_;
   }


   public static Supplier<AI2RHandPoseAdaptationMessagePubSubType> getPubSubType()
   {
      return AI2RHandPoseAdaptationMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return AI2RHandPoseAdaptationMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(AI2RHandPoseAdaptationMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.action_name_, other.action_name_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.reference_frame_name_, other.reference_frame_name_, epsilon)) return false;

      if (!this.new_position_.epsilonEquals(other.new_position_, epsilon)) return false;
      if (!this.new_orientation_.epsilonEquals(other.new_orientation_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof AI2RHandPoseAdaptationMessage)) return false;

      AI2RHandPoseAdaptationMessage otherMyClass = (AI2RHandPoseAdaptationMessage) other;

      if (!us.ihmc.idl.IDLTools.equals(this.action_name_, otherMyClass.action_name_)) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.reference_frame_name_, otherMyClass.reference_frame_name_)) return false;

      if (!this.new_position_.equals(otherMyClass.new_position_)) return false;
      if (!this.new_orientation_.equals(otherMyClass.new_orientation_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AI2RHandPoseAdaptationMessage {");
      builder.append("action_name=");
      builder.append(this.action_name_);      builder.append(", ");
      builder.append("reference_frame_name=");
      builder.append(this.reference_frame_name_);      builder.append(", ");
      builder.append("new_position=");
      builder.append(this.new_position_);      builder.append(", ");
      builder.append("new_orientation=");
      builder.append(this.new_orientation_);
      builder.append("}");
      return builder.toString();
   }
}
