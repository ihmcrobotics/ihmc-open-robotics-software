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
            * The opening mechanism
            */
   public perception_msgs.msg.dds.DetectedDoorOpeningMechanismMessage opening_mechanism_;
   /**
            * Panel pose
            * May contain NaN in the position and/or orientation values if unknown.
            */
   public us.ihmc.euclid.geometry.Pose3D panel_pose_;
   /**
            * Planar region of the panel
            */
   public perception_msgs.msg.dds.PlanarRegionMessage panel_planar_region_;
   /**
            * Last detection instant
            */
   public ihmc_common_msgs.msg.dds.InstantMessage last_detection_time_;

   public DetectedDoorMessage()
   {
      opening_mechanism_ = new perception_msgs.msg.dds.DetectedDoorOpeningMechanismMessage();
      panel_pose_ = new us.ihmc.euclid.geometry.Pose3D();
      panel_planar_region_ = new perception_msgs.msg.dds.PlanarRegionMessage();
      last_detection_time_ = new ihmc_common_msgs.msg.dds.InstantMessage();
   }

   public DetectedDoorMessage(DetectedDoorMessage other)
   {
      this();
      set(other);
   }

   public void set(DetectedDoorMessage other)
   {
      perception_msgs.msg.dds.DetectedDoorOpeningMechanismMessagePubSubType.staticCopy(other.opening_mechanism_, opening_mechanism_);
      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.panel_pose_, panel_pose_);
      perception_msgs.msg.dds.PlanarRegionMessagePubSubType.staticCopy(other.panel_planar_region_, panel_planar_region_);
      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.staticCopy(other.last_detection_time_, last_detection_time_);
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
   public us.ihmc.euclid.geometry.Pose3D getPanelPose()
   {
      return panel_pose_;
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

      if (!this.opening_mechanism_.epsilonEquals(other.opening_mechanism_, epsilon)) return false;
      if (!this.panel_pose_.epsilonEquals(other.panel_pose_, epsilon)) return false;
      if (!this.panel_planar_region_.epsilonEquals(other.panel_planar_region_, epsilon)) return false;
      if (!this.last_detection_time_.epsilonEquals(other.last_detection_time_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof DetectedDoorMessage)) return false;

      DetectedDoorMessage otherMyClass = (DetectedDoorMessage) other;

      if (!this.opening_mechanism_.equals(otherMyClass.opening_mechanism_)) return false;
      if (!this.panel_pose_.equals(otherMyClass.panel_pose_)) return false;
      if (!this.panel_planar_region_.equals(otherMyClass.panel_planar_region_)) return false;
      if (!this.last_detection_time_.equals(otherMyClass.last_detection_time_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("DetectedDoorMessage {");
      builder.append("opening_mechanism=");
      builder.append(this.opening_mechanism_);      builder.append(", ");
      builder.append("panel_pose=");
      builder.append(this.panel_pose_);      builder.append(", ");
      builder.append("panel_planar_region=");
      builder.append(this.panel_planar_region_);      builder.append(", ");
      builder.append("last_detection_time=");
      builder.append(this.last_detection_time_);
      builder.append("}");
      return builder.toString();
   }
}
