package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * YOLO:
       * FoundationPose:
       * Door Frame
       * Custom Reference Frame: Creates a Z-up from where X points to frame B from frame A and is some distance from frame B
       */
public class BehaviorTreeSceneObjectDefinitionMessage extends Packet<BehaviorTreeSceneObjectDefinitionMessage> implements Settable<BehaviorTreeSceneObjectDefinitionMessage>, EpsilonComparable<BehaviorTreeSceneObjectDefinitionMessage>
{
   /**
            * Object type: YOLO only, FoundationPose, custom frame, or specific heurustic
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
   /**
            * Minimum number of points to detect the post
            */
   public int min_post_points_;
   /**
            * Minimum number of point to detect the door recess
            */
   public int min_recess_points_;
   /**
            * Custom frame name
            */
   public java.lang.StringBuilder custom_frame_name_;
   /**
            * The name of frame A
            */
   public java.lang.StringBuilder frame_a_;
   /**
            * The name of frame B
            */
   public java.lang.StringBuilder frame_b_;
   /**
            * Distance from frame B
            */
   public float distance_;

   public BehaviorTreeSceneObjectDefinitionMessage()
   {
      yolo_model_name_ = new java.lang.StringBuilder(255);
      yolo_class_name_ = new java.lang.StringBuilder(255);
      custom_frame_name_ = new java.lang.StringBuilder(255);
      frame_a_ = new java.lang.StringBuilder(255);
      frame_b_ = new java.lang.StringBuilder(255);
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

      min_post_points_ = other.min_post_points_;

      min_recess_points_ = other.min_recess_points_;

      custom_frame_name_.setLength(0);
      custom_frame_name_.append(other.custom_frame_name_);

      frame_a_.setLength(0);
      frame_a_.append(other.frame_a_);

      frame_b_.setLength(0);
      frame_b_.append(other.frame_b_);

      distance_ = other.distance_;

   }

   /**
            * Object type: YOLO only, FoundationPose, custom frame, or specific heurustic
            */
   public void setObjectType(byte object_type)
   {
      object_type_ = object_type;
   }
   /**
            * Object type: YOLO only, FoundationPose, custom frame, or specific heurustic
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

   /**
            * Minimum number of points to detect the post
            */
   public void setMinPostPoints(int min_post_points)
   {
      min_post_points_ = min_post_points;
   }
   /**
            * Minimum number of points to detect the post
            */
   public int getMinPostPoints()
   {
      return min_post_points_;
   }

   /**
            * Minimum number of point to detect the door recess
            */
   public void setMinRecessPoints(int min_recess_points)
   {
      min_recess_points_ = min_recess_points;
   }
   /**
            * Minimum number of point to detect the door recess
            */
   public int getMinRecessPoints()
   {
      return min_recess_points_;
   }

   /**
            * Custom frame name
            */
   public void setCustomFrameName(java.lang.String custom_frame_name)
   {
      custom_frame_name_.setLength(0);
      custom_frame_name_.append(custom_frame_name);
   }

   /**
            * Custom frame name
            */
   public java.lang.String getCustomFrameNameAsString()
   {
      return getCustomFrameName().toString();
   }
   /**
            * Custom frame name
            */
   public java.lang.StringBuilder getCustomFrameName()
   {
      return custom_frame_name_;
   }

   /**
            * The name of frame A
            */
   public void setFrameA(java.lang.String frame_a)
   {
      frame_a_.setLength(0);
      frame_a_.append(frame_a);
   }

   /**
            * The name of frame A
            */
   public java.lang.String getFrameAAsString()
   {
      return getFrameA().toString();
   }
   /**
            * The name of frame A
            */
   public java.lang.StringBuilder getFrameA()
   {
      return frame_a_;
   }

   /**
            * The name of frame B
            */
   public void setFrameB(java.lang.String frame_b)
   {
      frame_b_.setLength(0);
      frame_b_.append(frame_b);
   }

   /**
            * The name of frame B
            */
   public java.lang.String getFrameBAsString()
   {
      return getFrameB().toString();
   }
   /**
            * The name of frame B
            */
   public java.lang.StringBuilder getFrameB()
   {
      return frame_b_;
   }

   /**
            * Distance from frame B
            */
   public void setDistance(float distance)
   {
      distance_ = distance;
   }
   /**
            * Distance from frame B
            */
   public float getDistance()
   {
      return distance_;
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

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.min_post_points_, other.min_post_points_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.min_recess_points_, other.min_recess_points_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.custom_frame_name_, other.custom_frame_name_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.frame_a_, other.frame_a_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.frame_b_, other.frame_b_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.distance_, other.distance_, epsilon)) return false;


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

      if(this.min_post_points_ != otherMyClass.min_post_points_) return false;

      if(this.min_recess_points_ != otherMyClass.min_recess_points_) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.custom_frame_name_, otherMyClass.custom_frame_name_)) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.frame_a_, otherMyClass.frame_a_)) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.frame_b_, otherMyClass.frame_b_)) return false;

      if(this.distance_ != otherMyClass.distance_) return false;


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
      builder.append(this.foundation_pose_object_type_);      builder.append(", ");
      builder.append("min_post_points=");
      builder.append(this.min_post_points_);      builder.append(", ");
      builder.append("min_recess_points=");
      builder.append(this.min_recess_points_);      builder.append(", ");
      builder.append("custom_frame_name=");
      builder.append(this.custom_frame_name_);      builder.append(", ");
      builder.append("frame_a=");
      builder.append(this.frame_a_);      builder.append(", ");
      builder.append("frame_b=");
      builder.append(this.frame_b_);      builder.append(", ");
      builder.append("distance=");
      builder.append(this.distance_);
      builder.append("}");
      return builder.toString();
   }
}
