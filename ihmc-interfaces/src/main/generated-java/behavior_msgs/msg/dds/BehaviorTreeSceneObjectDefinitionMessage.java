package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class BehaviorTreeSceneObjectDefinitionMessage extends Packet<BehaviorTreeSceneObjectDefinitionMessage> implements Settable<BehaviorTreeSceneObjectDefinitionMessage>, EpsilonComparable<BehaviorTreeSceneObjectDefinitionMessage>
{
   /**
            * Object type: YOLO only, FoundationPose, or specific heurustic
            */
   public byte object_type_;
   /**
            * YOLO model name to use
            */
   public java.lang.StringBuilder yolo_model_name_;
   /**
            * Class name to use for the object
            */
   public java.lang.StringBuilder yolo_class_name_;
   /**
            * FoundationPose object type (IsaacROSFoundationPoseObject enum ordinal)
            */
   public byte foundation_pose_object_type_;

   public BehaviorTreeSceneObjectDefinitionMessage()
   {
      yolo_model_name_ = new java.lang.StringBuilder(255);
      yolo_class_name_ = new java.lang.StringBuilder(255);
   }

   public BehaviorTreeSceneObjectDefinitionMessage(BehaviorTreeSceneObjectDefinitionMessage other)
   {
      this();
      set(other);
   }

   public void set(BehaviorTreeSceneObjectDefinitionMessage other)
   {
      object_type_ = other.object_type_;

      yolo_model_name_.setLength(0);
      yolo_model_name_.append(other.yolo_model_name_);

      yolo_class_name_.setLength(0);
      yolo_class_name_.append(other.yolo_class_name_);

      foundation_pose_object_type_ = other.foundation_pose_object_type_;

   }

   /**
            * Object type: YOLO only, FoundationPose, or specific heurustic
            */
   public void setObjectType(byte object_type)
   {
      object_type_ = object_type;
   }
   /**
            * Object type: YOLO only, FoundationPose, or specific heurustic
            */
   public byte getObjectType()
   {
      return object_type_;
   }

   /**
            * YOLO model name to use
            */
   public void setYoloModelName(java.lang.String yolo_model_name)
   {
      yolo_model_name_.setLength(0);
      yolo_model_name_.append(yolo_model_name);
   }

   /**
            * YOLO model name to use
            */
   public java.lang.String getYoloModelNameAsString()
   {
      return getYoloModelName().toString();
   }
   /**
            * YOLO model name to use
            */
   public java.lang.StringBuilder getYoloModelName()
   {
      return yolo_model_name_;
   }

   /**
            * Class name to use for the object
            */
   public void setYoloClassName(java.lang.String yolo_class_name)
   {
      yolo_class_name_.setLength(0);
      yolo_class_name_.append(yolo_class_name);
   }

   /**
            * Class name to use for the object
            */
   public java.lang.String getYoloClassNameAsString()
   {
      return getYoloClassName().toString();
   }
   /**
            * Class name to use for the object
            */
   public java.lang.StringBuilder getYoloClassName()
   {
      return yolo_class_name_;
   }

   /**
            * FoundationPose object type (IsaacROSFoundationPoseObject enum ordinal)
            */
   public void setFoundationPoseObjectType(byte foundation_pose_object_type)
   {
      foundation_pose_object_type_ = foundation_pose_object_type;
   }
   /**
            * FoundationPose object type (IsaacROSFoundationPoseObject enum ordinal)
            */
   public byte getFoundationPoseObjectType()
   {
      return foundation_pose_object_type_;
   }


   public static Supplier<BehaviorTreeSceneObjectDefinitionMessagePubSubType> getPubSubType()
   {
      return BehaviorTreeSceneObjectDefinitionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return BehaviorTreeSceneObjectDefinitionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(BehaviorTreeSceneObjectDefinitionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.object_type_, other.object_type_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.yolo_model_name_, other.yolo_model_name_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.yolo_class_name_, other.yolo_class_name_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.foundation_pose_object_type_, other.foundation_pose_object_type_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof BehaviorTreeSceneObjectDefinitionMessage)) return false;

      BehaviorTreeSceneObjectDefinitionMessage otherMyClass = (BehaviorTreeSceneObjectDefinitionMessage) other;

      if(this.object_type_ != otherMyClass.object_type_) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.yolo_model_name_, otherMyClass.yolo_model_name_)) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.yolo_class_name_, otherMyClass.yolo_class_name_)) return false;

      if(this.foundation_pose_object_type_ != otherMyClass.foundation_pose_object_type_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BehaviorTreeSceneObjectDefinitionMessage {");
      builder.append("object_type=");
      builder.append(this.object_type_);      builder.append(", ");
      builder.append("yolo_model_name=");
      builder.append(this.yolo_model_name_);      builder.append(", ");
      builder.append("yolo_class_name=");
      builder.append(this.yolo_class_name_);      builder.append(", ");
      builder.append("foundation_pose_object_type=");
      builder.append(this.foundation_pose_object_type_);
      builder.append("}");
      return builder.toString();
   }
}
