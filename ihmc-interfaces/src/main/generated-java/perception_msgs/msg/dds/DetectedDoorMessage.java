package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Message representing a DetectedDoor
       */
public class DetectedDoorMessage extends Packet<DetectedDoorMessage> implements Settable<DetectedDoorMessage>, EpsilonComparable<DetectedDoorMessage>
{
   /**
            * UUID of this detection
            */
   public ihmc_common_msgs.msg.dds.UUIDMessage detection_uuid_;
   /**
            * The opening mechanism
            */
   public perception_msgs.msg.dds.DetectedDoorOpeningMechanismMessage opening_mechanism_;
   /**
            * Panel pose
            * May contain NaN in the position and/or orientation values if unknown.
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage panel_transform_to_world_;
   /**
            * Planar region of the panel
            */
   public perception_msgs.msg.dds.PlanarRegionMessage panel_planar_region_;
   /**
            * Last detection instant
            */
   public ihmc_common_msgs.msg.dds.InstantMessage last_detection_time_;
   /**
            * Internal statistics
            */
   public ihmc_common_msgs.msg.dds.InstantMessage opening_mechanism_first_detection_time_;
   public int opening_mechanism_detection_count_;
   public ihmc_common_msgs.msg.dds.InstantMessage panel_first_detection_time_;
   public int panel_detection_count_;

   public DetectedDoorMessage()
   {
      detection_uuid_ = new ihmc_common_msgs.msg.dds.UUIDMessage();
      opening_mechanism_ = new perception_msgs.msg.dds.DetectedDoorOpeningMechanismMessage();
      panel_transform_to_world_ = new controller_msgs.msg.dds.RigidBodyTransformMessage();
      panel_planar_region_ = new perception_msgs.msg.dds.PlanarRegionMessage();
      last_detection_time_ = new ihmc_common_msgs.msg.dds.InstantMessage();
      opening_mechanism_first_detection_time_ = new ihmc_common_msgs.msg.dds.InstantMessage();
      panel_first_detection_time_ = new ihmc_common_msgs.msg.dds.InstantMessage();
   }

   public DetectedDoorMessage(DetectedDoorMessage other)
   {
      this();
      set(other);
   }

   public void set(DetectedDoorMessage other)
   {
      ihmc_common_msgs.msg.dds.UUIDMessagePubSubType.staticCopy(other.detection_uuid_, detection_uuid_);
      perception_msgs.msg.dds.DetectedDoorOpeningMechanismMessagePubSubType.staticCopy(other.opening_mechanism_, opening_mechanism_);
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.staticCopy(other.panel_transform_to_world_, panel_transform_to_world_);
      perception_msgs.msg.dds.PlanarRegionMessagePubSubType.staticCopy(other.panel_planar_region_, panel_planar_region_);
      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.staticCopy(other.last_detection_time_, last_detection_time_);
      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.staticCopy(other.opening_mechanism_first_detection_time_, opening_mechanism_first_detection_time_);
      opening_mechanism_detection_count_ = other.opening_mechanism_detection_count_;

      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.staticCopy(other.panel_first_detection_time_, panel_first_detection_time_);
      panel_detection_count_ = other.panel_detection_count_;

   }


   /**
            * UUID of this detection
            */
   public ihmc_common_msgs.msg.dds.UUIDMessage getDetectionUuid()
   {
      return detection_uuid_;
   }


   /**
            * The opening mechanism
            */
   public perception_msgs.msg.dds.DetectedDoorOpeningMechanismMessage getOpeningMechanism()
   {
      return opening_mechanism_;
   }


   /**
            * Panel pose
            * May contain NaN in the position and/or orientation values if unknown.
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage getPanelTransformToWorld()
   {
      return panel_transform_to_world_;
   }


   /**
            * Planar region of the panel
            */
   public perception_msgs.msg.dds.PlanarRegionMessage getPanelPlanarRegion()
   {
      return panel_planar_region_;
   }


   /**
            * Last detection instant
            */
   public ihmc_common_msgs.msg.dds.InstantMessage getLastDetectionTime()
   {
      return last_detection_time_;
   }


   /**
            * Internal statistics
            */
   public ihmc_common_msgs.msg.dds.InstantMessage getOpeningMechanismFirstDetectionTime()
   {
      return opening_mechanism_first_detection_time_;
   }

   public void setOpeningMechanismDetectionCount(int opening_mechanism_detection_count)
   {
      opening_mechanism_detection_count_ = opening_mechanism_detection_count;
   }
   public int getOpeningMechanismDetectionCount()
   {
      return opening_mechanism_detection_count_;
   }


   public ihmc_common_msgs.msg.dds.InstantMessage getPanelFirstDetectionTime()
   {
      return panel_first_detection_time_;
   }

   public void setPanelDetectionCount(int panel_detection_count)
   {
      panel_detection_count_ = panel_detection_count;
   }
   public int getPanelDetectionCount()
   {
      return panel_detection_count_;
   }


   public static Supplier<DetectedDoorMessagePubSubType> getPubSubType()
   {
      return DetectedDoorMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return DetectedDoorMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(DetectedDoorMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.detection_uuid_.epsilonEquals(other.detection_uuid_, epsilon)) return false;
      if (!this.opening_mechanism_.epsilonEquals(other.opening_mechanism_, epsilon)) return false;
      if (!this.panel_transform_to_world_.epsilonEquals(other.panel_transform_to_world_, epsilon)) return false;
      if (!this.panel_planar_region_.epsilonEquals(other.panel_planar_region_, epsilon)) return false;
      if (!this.last_detection_time_.epsilonEquals(other.last_detection_time_, epsilon)) return false;
      if (!this.opening_mechanism_first_detection_time_.epsilonEquals(other.opening_mechanism_first_detection_time_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.opening_mechanism_detection_count_, other.opening_mechanism_detection_count_, epsilon)) return false;

      if (!this.panel_first_detection_time_.epsilonEquals(other.panel_first_detection_time_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.panel_detection_count_, other.panel_detection_count_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof DetectedDoorMessage)) return false;

      DetectedDoorMessage otherMyClass = (DetectedDoorMessage) other;

      if (!this.detection_uuid_.equals(otherMyClass.detection_uuid_)) return false;
      if (!this.opening_mechanism_.equals(otherMyClass.opening_mechanism_)) return false;
      if (!this.panel_transform_to_world_.equals(otherMyClass.panel_transform_to_world_)) return false;
      if (!this.panel_planar_region_.equals(otherMyClass.panel_planar_region_)) return false;
      if (!this.last_detection_time_.equals(otherMyClass.last_detection_time_)) return false;
      if (!this.opening_mechanism_first_detection_time_.equals(otherMyClass.opening_mechanism_first_detection_time_)) return false;
      if(this.opening_mechanism_detection_count_ != otherMyClass.opening_mechanism_detection_count_) return false;

      if (!this.panel_first_detection_time_.equals(otherMyClass.panel_first_detection_time_)) return false;
      if(this.panel_detection_count_ != otherMyClass.panel_detection_count_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("DetectedDoorMessage {");
      builder.append("detection_uuid=");
      builder.append(this.detection_uuid_);      builder.append(", ");
      builder.append("opening_mechanism=");
      builder.append(this.opening_mechanism_);      builder.append(", ");
      builder.append("panel_transform_to_world=");
      builder.append(this.panel_transform_to_world_);      builder.append(", ");
      builder.append("panel_planar_region=");
      builder.append(this.panel_planar_region_);      builder.append(", ");
      builder.append("last_detection_time=");
      builder.append(this.last_detection_time_);      builder.append(", ");
      builder.append("opening_mechanism_first_detection_time=");
      builder.append(this.opening_mechanism_first_detection_time_);      builder.append(", ");
      builder.append("opening_mechanism_detection_count=");
      builder.append(this.opening_mechanism_detection_count_);      builder.append(", ");
      builder.append("panel_first_detection_time=");
      builder.append(this.panel_first_detection_time_);      builder.append(", ");
      builder.append("panel_detection_count=");
      builder.append(this.panel_detection_count_);
      builder.append("}");
      return builder.toString();
   }
}
