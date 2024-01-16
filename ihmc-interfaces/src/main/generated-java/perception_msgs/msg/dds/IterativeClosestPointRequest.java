package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class IterativeClosestPointRequest extends Packet<IterativeClosestPointRequest> implements Settable<IterativeClosestPointRequest>, EpsilonComparable<IterativeClosestPointRequest>
{
   /**
          * Shape of the object
          */
   public static final byte SHAPE_BOX = (byte) 0;
   public static final byte SHAPE_PRISM = (byte) 1;
   public static final byte SHAPE_ELLIPSOID = (byte) 2;
   public static final byte SHAPE_CYLINDER = (byte) 3;
   public static final byte SHAPE_CONE = (byte) 4;
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Scene node ID
            */
   public long node_id_;
   /**
            * Shape of the object
            */
   public byte shape_;
   /**
            * Dimensions of the object
            */
   public us.ihmc.euclid.tuple3D.Vector3D lengths_;
   public us.ihmc.euclid.tuple3D.Vector3D radii_;
   /**
            * User defined pose of virtual object
            */
   public us.ihmc.euclid.geometry.Pose3D provided_pose_;
   /**
            * ICP parameters
            */
   public int number_of_shape_samples_;
   public int number_of_correspondences_;
   public int number_of_iterations_;
   public float segmentation_radius_;
   /**
            * Start/stop flag (true = run, false = stop)
            */
   public boolean run_icp_;
   /**
            * Whether to use the user defined pose, or ICP objects centroid
            */
   public boolean use_provided_pose_;

   public IterativeClosestPointRequest()
   {
      lengths_ = new us.ihmc.euclid.tuple3D.Vector3D();
      radii_ = new us.ihmc.euclid.tuple3D.Vector3D();
      provided_pose_ = new us.ihmc.euclid.geometry.Pose3D();
   }

   public IterativeClosestPointRequest(IterativeClosestPointRequest other)
   {
      this();
      set(other);
   }

   public void set(IterativeClosestPointRequest other)
   {
      sequence_id_ = other.sequence_id_;

      node_id_ = other.node_id_;

      shape_ = other.shape_;

      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.lengths_, lengths_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.radii_, radii_);
      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.provided_pose_, provided_pose_);
      number_of_shape_samples_ = other.number_of_shape_samples_;

      number_of_correspondences_ = other.number_of_correspondences_;

      number_of_iterations_ = other.number_of_iterations_;

      segmentation_radius_ = other.segmentation_radius_;

      run_icp_ = other.run_icp_;

      use_provided_pose_ = other.use_provided_pose_;

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
            * Scene node ID
            */
   public void setNodeId(long node_id)
   {
      node_id_ = node_id;
   }
   /**
            * Scene node ID
            */
   public long getNodeId()
   {
      return node_id_;
   }

   /**
            * Shape of the object
            */
   public void setShape(byte shape)
   {
      shape_ = shape;
   }
   /**
            * Shape of the object
            */
   public byte getShape()
   {
      return shape_;
   }


   /**
            * Dimensions of the object
            */
   public us.ihmc.euclid.tuple3D.Vector3D getLengths()
   {
      return lengths_;
   }


   public us.ihmc.euclid.tuple3D.Vector3D getRadii()
   {
      return radii_;
   }


   /**
            * User defined pose of virtual object
            */
   public us.ihmc.euclid.geometry.Pose3D getProvidedPose()
   {
      return provided_pose_;
   }

   /**
            * ICP parameters
            */
   public void setNumberOfShapeSamples(int number_of_shape_samples)
   {
      number_of_shape_samples_ = number_of_shape_samples;
   }
   /**
            * ICP parameters
            */
   public int getNumberOfShapeSamples()
   {
      return number_of_shape_samples_;
   }

   public void setNumberOfCorrespondences(int number_of_correspondences)
   {
      number_of_correspondences_ = number_of_correspondences;
   }
   public int getNumberOfCorrespondences()
   {
      return number_of_correspondences_;
   }

   public void setNumberOfIterations(int number_of_iterations)
   {
      number_of_iterations_ = number_of_iterations;
   }
   public int getNumberOfIterations()
   {
      return number_of_iterations_;
   }

   public void setSegmentationRadius(float segmentation_radius)
   {
      segmentation_radius_ = segmentation_radius;
   }
   public float getSegmentationRadius()
   {
      return segmentation_radius_;
   }

   /**
            * Start/stop flag (true = run, false = stop)
            */
   public void setRunIcp(boolean run_icp)
   {
      run_icp_ = run_icp;
   }
   /**
            * Start/stop flag (true = run, false = stop)
            */
   public boolean getRunIcp()
   {
      return run_icp_;
   }

   /**
            * Whether to use the user defined pose, or ICP objects centroid
            */
   public void setUseProvidedPose(boolean use_provided_pose)
   {
      use_provided_pose_ = use_provided_pose;
   }
   /**
            * Whether to use the user defined pose, or ICP objects centroid
            */
   public boolean getUseProvidedPose()
   {
      return use_provided_pose_;
   }


   public static Supplier<IterativeClosestPointRequestPubSubType> getPubSubType()
   {
      return IterativeClosestPointRequestPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return IterativeClosestPointRequestPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(IterativeClosestPointRequest other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.node_id_, other.node_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.shape_, other.shape_, epsilon)) return false;

      if (!this.lengths_.epsilonEquals(other.lengths_, epsilon)) return false;
      if (!this.radii_.epsilonEquals(other.radii_, epsilon)) return false;
      if (!this.provided_pose_.epsilonEquals(other.provided_pose_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.number_of_shape_samples_, other.number_of_shape_samples_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.number_of_correspondences_, other.number_of_correspondences_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.number_of_iterations_, other.number_of_iterations_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.segmentation_radius_, other.segmentation_radius_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.run_icp_, other.run_icp_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.use_provided_pose_, other.use_provided_pose_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof IterativeClosestPointRequest)) return false;

      IterativeClosestPointRequest otherMyClass = (IterativeClosestPointRequest) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.node_id_ != otherMyClass.node_id_) return false;

      if(this.shape_ != otherMyClass.shape_) return false;

      if (!this.lengths_.equals(otherMyClass.lengths_)) return false;
      if (!this.radii_.equals(otherMyClass.radii_)) return false;
      if (!this.provided_pose_.equals(otherMyClass.provided_pose_)) return false;
      if(this.number_of_shape_samples_ != otherMyClass.number_of_shape_samples_) return false;

      if(this.number_of_correspondences_ != otherMyClass.number_of_correspondences_) return false;

      if(this.number_of_iterations_ != otherMyClass.number_of_iterations_) return false;

      if(this.segmentation_radius_ != otherMyClass.segmentation_radius_) return false;

      if(this.run_icp_ != otherMyClass.run_icp_) return false;

      if(this.use_provided_pose_ != otherMyClass.use_provided_pose_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("IterativeClosestPointRequest {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("node_id=");
      builder.append(this.node_id_);      builder.append(", ");
      builder.append("shape=");
      builder.append(this.shape_);      builder.append(", ");
      builder.append("lengths=");
      builder.append(this.lengths_);      builder.append(", ");
      builder.append("radii=");
      builder.append(this.radii_);      builder.append(", ");
      builder.append("provided_pose=");
      builder.append(this.provided_pose_);      builder.append(", ");
      builder.append("number_of_shape_samples=");
      builder.append(this.number_of_shape_samples_);      builder.append(", ");
      builder.append("number_of_correspondences=");
      builder.append(this.number_of_correspondences_);      builder.append(", ");
      builder.append("number_of_iterations=");
      builder.append(this.number_of_iterations_);      builder.append(", ");
      builder.append("segmentation_radius=");
      builder.append(this.segmentation_radius_);      builder.append(", ");
      builder.append("run_icp=");
      builder.append(this.run_icp_);      builder.append(", ");
      builder.append("use_provided_pose=");
      builder.append(this.use_provided_pose_);
      builder.append("}");
      return builder.toString();
   }
}
