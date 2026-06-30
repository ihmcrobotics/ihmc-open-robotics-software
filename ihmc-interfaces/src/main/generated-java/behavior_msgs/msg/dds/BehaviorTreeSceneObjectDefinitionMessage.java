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
       * Composite Reference Frame:
       * Approach table
       */
public class BehaviorTreeSceneObjectDefinitionMessage extends Packet<BehaviorTreeSceneObjectDefinitionMessage> implements Settable<BehaviorTreeSceneObjectDefinitionMessage>, EpsilonComparable<BehaviorTreeSceneObjectDefinitionMessage>
{
   /**
          * From A towards, backed up by X distance, Zup
          */
   public static final byte COMPOSITE_FRAME_APPROACH = (byte) 0;
   public static final byte COMPOSITE_FRAME_HYBRID = (byte) 1;
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
            * Composite frame name
            */
   public java.lang.StringBuilder composite_frame_name_;
   /**
            * The name of frame A
            */
   public java.lang.StringBuilder composite_frame_a_;
   /**
            * The name of frame B
            */
   public java.lang.StringBuilder composite_frame_b_;
   /**
            * The type of composite frame
            */
   public byte composite_frame_type_;
   /**
            * Distance from frame B
            */
   public float composite_distance_;
   /**
            * Consider the table edge when there are this many points in the capsule
            */
   public int min_capsule_points_;
   /**
            * Initial capsule distance to start from robot, tunable to avoid detecting self as the table
            */
   public float search_start_x_;

   public BehaviorTreeSceneObjectDefinitionMessage()
   {
      yolo_model_name_ = new java.lang.StringBuilder(255);
      yolo_class_name_ = new java.lang.StringBuilder(255);
      composite_frame_name_ = new java.lang.StringBuilder(255);
      composite_frame_a_ = new java.lang.StringBuilder(255);
      composite_frame_b_ = new java.lang.StringBuilder(255);
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

      composite_frame_name_.setLength(0);
      composite_frame_name_.append(other.composite_frame_name_);

      composite_frame_a_.setLength(0);
      composite_frame_a_.append(other.composite_frame_a_);

      composite_frame_b_.setLength(0);
      composite_frame_b_.append(other.composite_frame_b_);

      composite_frame_type_ = other.composite_frame_type_;

      composite_distance_ = other.composite_distance_;

      min_capsule_points_ = other.min_capsule_points_;

      search_start_x_ = other.search_start_x_;

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
            * Composite frame name
            */
   public void setCompositeFrameName(java.lang.String composite_frame_name)
   {
      composite_frame_name_.setLength(0);
      composite_frame_name_.append(composite_frame_name);
   }

   /**
            * Composite frame name
            */
   public java.lang.String getCompositeFrameNameAsString()
   {
      return getCompositeFrameName().toString();
   }
   /**
            * Composite frame name
            */
   public java.lang.StringBuilder getCompositeFrameName()
   {
      return composite_frame_name_;
   }

   /**
            * The name of frame A
            */
   public void setCompositeFrameA(java.lang.String composite_frame_a)
   {
      composite_frame_a_.setLength(0);
      composite_frame_a_.append(composite_frame_a);
   }

   /**
            * The name of frame A
            */
   public java.lang.String getCompositeFrameAAsString()
   {
      return getCompositeFrameA().toString();
   }
   /**
            * The name of frame A
            */
   public java.lang.StringBuilder getCompositeFrameA()
   {
      return composite_frame_a_;
   }

   /**
            * The name of frame B
            */
   public void setCompositeFrameB(java.lang.String composite_frame_b)
   {
      composite_frame_b_.setLength(0);
      composite_frame_b_.append(composite_frame_b);
   }

   /**
            * The name of frame B
            */
   public java.lang.String getCompositeFrameBAsString()
   {
      return getCompositeFrameB().toString();
   }
   /**
            * The name of frame B
            */
   public java.lang.StringBuilder getCompositeFrameB()
   {
      return composite_frame_b_;
   }

   /**
            * The type of composite frame
            */
   public void setCompositeFrameType(byte composite_frame_type)
   {
      composite_frame_type_ = composite_frame_type;
   }
   /**
            * The type of composite frame
            */
   public byte getCompositeFrameType()
   {
      return composite_frame_type_;
   }

   /**
            * Distance from frame B
            */
   public void setCompositeDistance(float composite_distance)
   {
      composite_distance_ = composite_distance;
   }
   /**
            * Distance from frame B
            */
   public float getCompositeDistance()
   {
      return composite_distance_;
   }

   /**
            * Consider the table edge when there are this many points in the capsule
            */
   public void setMinCapsulePoints(int min_capsule_points)
   {
      min_capsule_points_ = min_capsule_points;
   }
   /**
            * Consider the table edge when there are this many points in the capsule
            */
   public int getMinCapsulePoints()
   {
      return min_capsule_points_;
   }

   /**
            * Initial capsule distance to start from robot, tunable to avoid detecting self as the table
            */
   public void setSearchStartX(float search_start_x)
   {
      search_start_x_ = search_start_x;
   }
   /**
            * Initial capsule distance to start from robot, tunable to avoid detecting self as the table
            */
   public float getSearchStartX()
   {
      return search_start_x_;
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

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.composite_frame_name_, other.composite_frame_name_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.composite_frame_a_, other.composite_frame_a_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.composite_frame_b_, other.composite_frame_b_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.composite_frame_type_, other.composite_frame_type_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.composite_distance_, other.composite_distance_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.min_capsule_points_, other.min_capsule_points_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.search_start_x_, other.search_start_x_, epsilon)) return false;


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

      if (!us.ihmc.idl.IDLTools.equals(this.composite_frame_name_, otherMyClass.composite_frame_name_)) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.composite_frame_a_, otherMyClass.composite_frame_a_)) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.composite_frame_b_, otherMyClass.composite_frame_b_)) return false;

      if(this.composite_frame_type_ != otherMyClass.composite_frame_type_) return false;

      if(this.composite_distance_ != otherMyClass.composite_distance_) return false;

      if(this.min_capsule_points_ != otherMyClass.min_capsule_points_) return false;

      if(this.search_start_x_ != otherMyClass.search_start_x_) return false;


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
      builder.append("composite_frame_name=");
      builder.append(this.composite_frame_name_);      builder.append(", ");
      builder.append("composite_frame_a=");
      builder.append(this.composite_frame_a_);      builder.append(", ");
      builder.append("composite_frame_b=");
      builder.append(this.composite_frame_b_);      builder.append(", ");
      builder.append("composite_frame_type=");
      builder.append(this.composite_frame_type_);      builder.append(", ");
      builder.append("composite_distance=");
      builder.append(this.composite_distance_);      builder.append(", ");
      builder.append("min_capsule_points=");
      builder.append(this.min_capsule_points_);      builder.append(", ");
      builder.append("search_start_x=");
      builder.append(this.search_start_x_);
      builder.append("}");
      return builder.toString();
   }
}
