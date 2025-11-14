package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Status of a persistent detection
       */
public class PersistentDetectionStatusMessage extends Packet<PersistentDetectionStatusMessage> implements Settable<PersistentDetectionStatusMessage>, EpsilonComparable<PersistentDetectionStatusMessage>
{
   /**
            * Last 4 digits of UUID as a string
            */
   public java.lang.StringBuilder id_;
   /**
            * Instant detection simple class name (e.g. YOLO, FoundationPose)
            */
   public java.lang.StringBuilder detection_type_;
   /**
            * Object class name (e.g., "bottle")
            */
   public java.lang.StringBuilder object_class_;
   /**
            * Detection frequency (Hz) with decay
            */
   public double decaying_frequency_;
   /**
            * Number of detections in history
            */
   public int history_size_;
   /**
            * Whether the detection is stable
            */
   public boolean is_stable_;
   /**
            * Alpha filtered transform to world
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage transform_to_world_;
   /**
            * Alpha filtered transform to camera
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage transform_to_camera_;

   public PersistentDetectionStatusMessage()
   {
      id_ = new java.lang.StringBuilder(255);
      detection_type_ = new java.lang.StringBuilder(255);
      object_class_ = new java.lang.StringBuilder(255);
      transform_to_world_ = new controller_msgs.msg.dds.RigidBodyTransformMessage();
      transform_to_camera_ = new controller_msgs.msg.dds.RigidBodyTransformMessage();
   }

   public PersistentDetectionStatusMessage(PersistentDetectionStatusMessage other)
   {
      this();
      set(other);
   }

   public void set(PersistentDetectionStatusMessage other)
   {
      id_.setLength(0);
      id_.append(other.id_);

      detection_type_.setLength(0);
      detection_type_.append(other.detection_type_);

      object_class_.setLength(0);
      object_class_.append(other.object_class_);

      decaying_frequency_ = other.decaying_frequency_;

      history_size_ = other.history_size_;

      is_stable_ = other.is_stable_;

      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.staticCopy(other.transform_to_world_, transform_to_world_);
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.staticCopy(other.transform_to_camera_, transform_to_camera_);
   }

   /**
            * Last 4 digits of UUID as a string
            */
   public void setId(java.lang.String id)
   {
      id_.setLength(0);
      id_.append(id);
   }

   /**
            * Last 4 digits of UUID as a string
            */
   public java.lang.String getIdAsString()
   {
      return getId().toString();
   }
   /**
            * Last 4 digits of UUID as a string
            */
   public java.lang.StringBuilder getId()
   {
      return id_;
   }

   /**
            * Instant detection simple class name (e.g. YOLO, FoundationPose)
            */
   public void setDetectionType(java.lang.String detection_type)
   {
      detection_type_.setLength(0);
      detection_type_.append(detection_type);
   }

   /**
            * Instant detection simple class name (e.g. YOLO, FoundationPose)
            */
   public java.lang.String getDetectionTypeAsString()
   {
      return getDetectionType().toString();
   }
   /**
            * Instant detection simple class name (e.g. YOLO, FoundationPose)
            */
   public java.lang.StringBuilder getDetectionType()
   {
      return detection_type_;
   }

   /**
            * Object class name (e.g., "bottle")
            */
   public void setObjectClass(java.lang.String object_class)
   {
      object_class_.setLength(0);
      object_class_.append(object_class);
   }

   /**
            * Object class name (e.g., "bottle")
            */
   public java.lang.String getObjectClassAsString()
   {
      return getObjectClass().toString();
   }
   /**
            * Object class name (e.g., "bottle")
            */
   public java.lang.StringBuilder getObjectClass()
   {
      return object_class_;
   }

   /**
            * Detection frequency (Hz) with decay
            */
   public void setDecayingFrequency(double decaying_frequency)
   {
      decaying_frequency_ = decaying_frequency;
   }
   /**
            * Detection frequency (Hz) with decay
            */
   public double getDecayingFrequency()
   {
      return decaying_frequency_;
   }

   /**
            * Number of detections in history
            */
   public void setHistorySize(int history_size)
   {
      history_size_ = history_size;
   }
   /**
            * Number of detections in history
            */
   public int getHistorySize()
   {
      return history_size_;
   }

   /**
            * Whether the detection is stable
            */
   public void setIsStable(boolean is_stable)
   {
      is_stable_ = is_stable;
   }
   /**
            * Whether the detection is stable
            */
   public boolean getIsStable()
   {
      return is_stable_;
   }


   /**
            * Alpha filtered transform to world
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage getTransformToWorld()
   {
      return transform_to_world_;
   }


   /**
            * Alpha filtered transform to camera
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage getTransformToCamera()
   {
      return transform_to_camera_;
   }


   public static Supplier<PersistentDetectionStatusMessagePubSubType> getPubSubType()
   {
      return PersistentDetectionStatusMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return PersistentDetectionStatusMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(PersistentDetectionStatusMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.id_, other.id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.detection_type_, other.detection_type_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.object_class_, other.object_class_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.decaying_frequency_, other.decaying_frequency_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.history_size_, other.history_size_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.is_stable_, other.is_stable_, epsilon)) return false;

      if (!this.transform_to_world_.epsilonEquals(other.transform_to_world_, epsilon)) return false;
      if (!this.transform_to_camera_.epsilonEquals(other.transform_to_camera_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof PersistentDetectionStatusMessage)) return false;

      PersistentDetectionStatusMessage otherMyClass = (PersistentDetectionStatusMessage) other;

      if (!us.ihmc.idl.IDLTools.equals(this.id_, otherMyClass.id_)) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.detection_type_, otherMyClass.detection_type_)) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.object_class_, otherMyClass.object_class_)) return false;

      if(this.decaying_frequency_ != otherMyClass.decaying_frequency_) return false;

      if(this.history_size_ != otherMyClass.history_size_) return false;

      if(this.is_stable_ != otherMyClass.is_stable_) return false;

      if (!this.transform_to_world_.equals(otherMyClass.transform_to_world_)) return false;
      if (!this.transform_to_camera_.equals(otherMyClass.transform_to_camera_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("PersistentDetectionStatusMessage {");
      builder.append("id=");
      builder.append(this.id_);      builder.append(", ");
      builder.append("detection_type=");
      builder.append(this.detection_type_);      builder.append(", ");
      builder.append("object_class=");
      builder.append(this.object_class_);      builder.append(", ");
      builder.append("decaying_frequency=");
      builder.append(this.decaying_frequency_);      builder.append(", ");
      builder.append("history_size=");
      builder.append(this.history_size_);      builder.append(", ");
      builder.append("is_stable=");
      builder.append(this.is_stable_);      builder.append(", ");
      builder.append("transform_to_world=");
      builder.append(this.transform_to_world_);      builder.append(", ");
      builder.append("transform_to_camera=");
      builder.append(this.transform_to_camera_);
      builder.append("}");
      return builder.toString();
   }
}
