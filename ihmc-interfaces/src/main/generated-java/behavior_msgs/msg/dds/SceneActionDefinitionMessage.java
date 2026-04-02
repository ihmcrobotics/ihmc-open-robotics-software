package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Setup, freeze, delete object:
       * Configure YOLO:
       * Configure FoundationPose:
       */
public class SceneActionDefinitionMessage extends Packet<SceneActionDefinitionMessage> implements Settable<SceneActionDefinitionMessage>, EpsilonComparable<SceneActionDefinitionMessage>
{
   public static final byte SETUP_OBJECT = (byte) 0;
   public static final byte FREEZE_OBJECT = (byte) 1;
   public static final byte DELETE_OBJECT = (byte) 2;
   public static final byte CLEAR_SCENE = (byte) 3;
   public static final byte FREEZE_SCENE = (byte) 4;
   public static final byte CONFIGURE_YOLO = (byte) 5;
   public static final byte CONFIGURE_FOUNDATION_POSE = (byte) 6;
   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.ActionNodeDefinitionMessage definition_;
   /**
            * The type of action as defined above
            */
   public byte scene_action_type_;
   /**
            * Definition of the object to setup
            */
   public behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage scene_object_definition_;
   /**
            * If this action can't do what it needs to do in this amount of time, give up.
            */
   public float timeout_;
   /**
            * Filter for persistent detection history size
            */
   public int minimum_history_size_;
   /**
            * The nominal object pose for behavior previewing
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage nominal_object_pose_;
   /**
            * YOLO models that should be enabled at this point (by available index)
            */
   public us.ihmc.idl.IDLSequence.Integer  enabled_yolo_models_;
   /**
            * Pararmeters of the enabled YOLO models
            */
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.YOLOv8ModelParameters>  yolo_model_parameters_;
   /**
            * FoundationPose models that should be enabled at this point
            */
   public us.ihmc.idl.IDLSequence.Byte  enabled_foundation_pose_models_;

   public SceneActionDefinitionMessage()
   {
      definition_ = new behavior_msgs.msg.dds.ActionNodeDefinitionMessage();
      scene_object_definition_ = new behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage();
      nominal_object_pose_ = new controller_msgs.msg.dds.RigidBodyTransformMessage();
      enabled_yolo_models_ = new us.ihmc.idl.IDLSequence.Integer (10, "type_3");

      yolo_model_parameters_ = new us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.YOLOv8ModelParameters> (10, new perception_msgs.msg.dds.YOLOv8ModelParametersPubSubType());
      enabled_foundation_pose_models_ = new us.ihmc.idl.IDLSequence.Byte (10, "type_9");


   }

   public SceneActionDefinitionMessage(SceneActionDefinitionMessage other)
   {
      this();
      set(other);
   }

   public void set(SceneActionDefinitionMessage other)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      scene_action_type_ = other.scene_action_type_;

      behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessagePubSubType.staticCopy(other.scene_object_definition_, scene_object_definition_);
      timeout_ = other.timeout_;

      minimum_history_size_ = other.minimum_history_size_;

      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.staticCopy(other.nominal_object_pose_, nominal_object_pose_);
      enabled_yolo_models_.set(other.enabled_yolo_models_);
      yolo_model_parameters_.set(other.yolo_model_parameters_);
      enabled_foundation_pose_models_.set(other.enabled_foundation_pose_models_);
   }


   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.ActionNodeDefinitionMessage getDefinition()
   {
      return definition_;
   }

   /**
            * The type of action as defined above
            */
   public void setSceneActionType(byte scene_action_type)
   {
      scene_action_type_ = scene_action_type;
   }
   /**
            * The type of action as defined above
            */
   public byte getSceneActionType()
   {
      return scene_action_type_;
   }


   /**
            * Definition of the object to setup
            */
   public behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage getSceneObjectDefinition()
   {
      return scene_object_definition_;
   }

   /**
            * If this action can't do what it needs to do in this amount of time, give up.
            */
   public void setTimeout(float timeout)
   {
      timeout_ = timeout;
   }
   /**
            * If this action can't do what it needs to do in this amount of time, give up.
            */
   public float getTimeout()
   {
      return timeout_;
   }

   /**
            * Filter for persistent detection history size
            */
   public void setMinimumHistorySize(int minimum_history_size)
   {
      minimum_history_size_ = minimum_history_size;
   }
   /**
            * Filter for persistent detection history size
            */
   public int getMinimumHistorySize()
   {
      return minimum_history_size_;
   }


   /**
            * The nominal object pose for behavior previewing
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage getNominalObjectPose()
   {
      return nominal_object_pose_;
   }


   /**
            * YOLO models that should be enabled at this point (by available index)
            */
   public us.ihmc.idl.IDLSequence.Integer  getEnabledYoloModels()
   {
      return enabled_yolo_models_;
   }


   /**
            * Pararmeters of the enabled YOLO models
            */
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.YOLOv8ModelParameters>  getYoloModelParameters()
   {
      return yolo_model_parameters_;
   }


   /**
            * FoundationPose models that should be enabled at this point
            */
   public us.ihmc.idl.IDLSequence.Byte  getEnabledFoundationPoseModels()
   {
      return enabled_foundation_pose_models_;
   }


   public static Supplier<SceneActionDefinitionMessagePubSubType> getPubSubType()
   {
      return SceneActionDefinitionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return SceneActionDefinitionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(SceneActionDefinitionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.scene_action_type_, other.scene_action_type_, epsilon)) return false;

      if (!this.scene_object_definition_.epsilonEquals(other.scene_object_definition_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.timeout_, other.timeout_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.minimum_history_size_, other.minimum_history_size_, epsilon)) return false;

      if (!this.nominal_object_pose_.epsilonEquals(other.nominal_object_pose_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsIntegerSequence(this.enabled_yolo_models_, other.enabled_yolo_models_, epsilon)) return false;

      if (this.yolo_model_parameters_.size() != other.yolo_model_parameters_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.yolo_model_parameters_.size(); i++)
         {  if (!this.yolo_model_parameters_.get(i).epsilonEquals(other.yolo_model_parameters_.get(i), epsilon)) return false; }
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.enabled_foundation_pose_models_, other.enabled_foundation_pose_models_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof SceneActionDefinitionMessage)) return false;

      SceneActionDefinitionMessage otherMyClass = (SceneActionDefinitionMessage) other;

      if (!this.definition_.equals(otherMyClass.definition_)) return false;
      if(this.scene_action_type_ != otherMyClass.scene_action_type_) return false;

      if (!this.scene_object_definition_.equals(otherMyClass.scene_object_definition_)) return false;
      if(this.timeout_ != otherMyClass.timeout_) return false;

      if(this.minimum_history_size_ != otherMyClass.minimum_history_size_) return false;

      if (!this.nominal_object_pose_.equals(otherMyClass.nominal_object_pose_)) return false;
      if (!this.enabled_yolo_models_.equals(otherMyClass.enabled_yolo_models_)) return false;
      if (!this.yolo_model_parameters_.equals(otherMyClass.yolo_model_parameters_)) return false;
      if (!this.enabled_foundation_pose_models_.equals(otherMyClass.enabled_foundation_pose_models_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("SceneActionDefinitionMessage {");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("scene_action_type=");
      builder.append(this.scene_action_type_);      builder.append(", ");
      builder.append("scene_object_definition=");
      builder.append(this.scene_object_definition_);      builder.append(", ");
      builder.append("timeout=");
      builder.append(this.timeout_);      builder.append(", ");
      builder.append("minimum_history_size=");
      builder.append(this.minimum_history_size_);      builder.append(", ");
      builder.append("nominal_object_pose=");
      builder.append(this.nominal_object_pose_);      builder.append(", ");
      builder.append("enabled_yolo_models=");
      builder.append(this.enabled_yolo_models_);      builder.append(", ");
      builder.append("yolo_model_parameters=");
      builder.append(this.yolo_model_parameters_);      builder.append(", ");
      builder.append("enabled_foundation_pose_models=");
      builder.append(this.enabled_foundation_pose_models_);
      builder.append("}");
      return builder.toString();
   }
}
