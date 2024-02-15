package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Perception scene node for YOLOv8 ICP detection
       */
public class YOLOv8ICPNodeMessage extends Packet<YOLOv8ICPNodeMessage> implements Settable<YOLOv8ICPNodeMessage>, EpsilonComparable<YOLOv8ICPNodeMessage>
{
   /**
            * Detectable scene node information
            */
   public perception_msgs.msg.dds.DetectableSceneNodeMessage detectable_scene_node_;
   /**
            * YOLO/ICP Parameters
            */
   public int mask_erosion_kernel_radius_;
   public double outlier_filter_threshold_;
   public int icp_iterations_;
   public double base_distance_threshold_;
   public boolean run_icp_;
   /**
            * Status
            */
   public double movement_distance_threshold_;
   public double detection_frequency_;

   public YOLOv8ICPNodeMessage()
   {
      detectable_scene_node_ = new perception_msgs.msg.dds.DetectableSceneNodeMessage();
   }

   public YOLOv8ICPNodeMessage(YOLOv8ICPNodeMessage other)
   {
      this();
      set(other);
   }

   public void set(YOLOv8ICPNodeMessage other)
   {
      perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType.staticCopy(other.detectable_scene_node_, detectable_scene_node_);
      mask_erosion_kernel_radius_ = other.mask_erosion_kernel_radius_;

      outlier_filter_threshold_ = other.outlier_filter_threshold_;

      icp_iterations_ = other.icp_iterations_;

      base_distance_threshold_ = other.base_distance_threshold_;

      run_icp_ = other.run_icp_;

      movement_distance_threshold_ = other.movement_distance_threshold_;

      detection_frequency_ = other.detection_frequency_;

   }


   /**
            * Detectable scene node information
            */
   public perception_msgs.msg.dds.DetectableSceneNodeMessage getDetectableSceneNode()
   {
      return detectable_scene_node_;
   }

   /**
            * YOLO/ICP Parameters
            */
   public void setMaskErosionKernelRadius(int mask_erosion_kernel_radius)
   {
      mask_erosion_kernel_radius_ = mask_erosion_kernel_radius;
   }
   /**
            * YOLO/ICP Parameters
            */
   public int getMaskErosionKernelRadius()
   {
      return mask_erosion_kernel_radius_;
   }

   public void setOutlierFilterThreshold(double outlier_filter_threshold)
   {
      outlier_filter_threshold_ = outlier_filter_threshold;
   }
   public double getOutlierFilterThreshold()
   {
      return outlier_filter_threshold_;
   }

   public void setIcpIterations(int icp_iterations)
   {
      icp_iterations_ = icp_iterations;
   }
   public int getIcpIterations()
   {
      return icp_iterations_;
   }

   public void setBaseDistanceThreshold(double base_distance_threshold)
   {
      base_distance_threshold_ = base_distance_threshold;
   }
   public double getBaseDistanceThreshold()
   {
      return base_distance_threshold_;
   }

   public void setRunIcp(boolean run_icp)
   {
      run_icp_ = run_icp;
   }
   public boolean getRunIcp()
   {
      return run_icp_;
   }

   /**
            * Status
            */
   public void setMovementDistanceThreshold(double movement_distance_threshold)
   {
      movement_distance_threshold_ = movement_distance_threshold;
   }
   /**
            * Status
            */
   public double getMovementDistanceThreshold()
   {
      return movement_distance_threshold_;
   }

   public void setDetectionFrequency(double detection_frequency)
   {
      detection_frequency_ = detection_frequency;
   }
   public double getDetectionFrequency()
   {
      return detection_frequency_;
   }


   public static Supplier<YOLOv8ICPNodeMessagePubSubType> getPubSubType()
   {
      return YOLOv8ICPNodeMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return YOLOv8ICPNodeMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(YOLOv8ICPNodeMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.detectable_scene_node_.epsilonEquals(other.detectable_scene_node_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.mask_erosion_kernel_radius_, other.mask_erosion_kernel_radius_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.outlier_filter_threshold_, other.outlier_filter_threshold_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.icp_iterations_, other.icp_iterations_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.base_distance_threshold_, other.base_distance_threshold_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.run_icp_, other.run_icp_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.movement_distance_threshold_, other.movement_distance_threshold_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.detection_frequency_, other.detection_frequency_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof YOLOv8ICPNodeMessage)) return false;

      YOLOv8ICPNodeMessage otherMyClass = (YOLOv8ICPNodeMessage) other;

      if (!this.detectable_scene_node_.equals(otherMyClass.detectable_scene_node_)) return false;
      if(this.mask_erosion_kernel_radius_ != otherMyClass.mask_erosion_kernel_radius_) return false;

      if(this.outlier_filter_threshold_ != otherMyClass.outlier_filter_threshold_) return false;

      if(this.icp_iterations_ != otherMyClass.icp_iterations_) return false;

      if(this.base_distance_threshold_ != otherMyClass.base_distance_threshold_) return false;

      if(this.run_icp_ != otherMyClass.run_icp_) return false;

      if(this.movement_distance_threshold_ != otherMyClass.movement_distance_threshold_) return false;

      if(this.detection_frequency_ != otherMyClass.detection_frequency_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("YOLOv8ICPNodeMessage {");
      builder.append("detectable_scene_node=");
      builder.append(this.detectable_scene_node_);      builder.append(", ");
      builder.append("mask_erosion_kernel_radius=");
      builder.append(this.mask_erosion_kernel_radius_);      builder.append(", ");
      builder.append("outlier_filter_threshold=");
      builder.append(this.outlier_filter_threshold_);      builder.append(", ");
      builder.append("icp_iterations=");
      builder.append(this.icp_iterations_);      builder.append(", ");
      builder.append("base_distance_threshold=");
      builder.append(this.base_distance_threshold_);      builder.append(", ");
      builder.append("run_icp=");
      builder.append(this.run_icp_);      builder.append(", ");
      builder.append("movement_distance_threshold=");
      builder.append(this.movement_distance_threshold_);      builder.append(", ");
      builder.append("detection_frequency=");
      builder.append(this.detection_frequency_);
      builder.append("}");
      return builder.toString();
   }
}
