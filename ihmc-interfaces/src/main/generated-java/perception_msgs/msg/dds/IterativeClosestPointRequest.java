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
   public float x_length_;
   public float y_length_;
   public float z_length_;
   public float x_radius_;
   public float y_radius_;
   public float z_radius_;
   /**
            * User defined pose of virtual object
            */
   public us.ihmc.euclid.geometry.Pose3D provided_pose_;
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

      x_length_ = other.x_length_;

      y_length_ = other.y_length_;

      z_length_ = other.z_length_;

      x_radius_ = other.x_radius_;

      y_radius_ = other.y_radius_;

      z_radius_ = other.z_radius_;

      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.provided_pose_, provided_pose_);
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
   public void setXLength(float x_length)
   {
      x_length_ = x_length;
   }
   /**
            * Dimensions of the object
            */
   public float getXLength()
   {
      return x_length_;
   }

   public void setYLength(float y_length)
   {
      y_length_ = y_length;
   }
   public float getYLength()
   {
      return y_length_;
   }

   public void setZLength(float z_length)
   {
      z_length_ = z_length;
   }
   public float getZLength()
   {
      return z_length_;
   }

   public void setXRadius(float x_radius)
   {
      x_radius_ = x_radius;
   }
   public float getXRadius()
   {
      return x_radius_;
   }

   public void setYRadius(float y_radius)
   {
      y_radius_ = y_radius;
   }
   public float getYRadius()
   {
      return y_radius_;
   }

   public void setZRadius(float z_radius)
   {
      z_radius_ = z_radius;
   }
   public float getZRadius()
   {
      return z_radius_;
   }


   /**
            * User defined pose of virtual object
            */
   public us.ihmc.euclid.geometry.Pose3D getProvidedPose()
   {
      return provided_pose_;
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

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.x_length_, other.x_length_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.y_length_, other.y_length_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.z_length_, other.z_length_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.x_radius_, other.x_radius_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.y_radius_, other.y_radius_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.z_radius_, other.z_radius_, epsilon)) return false;

      if (!this.provided_pose_.epsilonEquals(other.provided_pose_, epsilon)) return false;
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

      if(this.x_length_ != otherMyClass.x_length_) return false;

      if(this.y_length_ != otherMyClass.y_length_) return false;

      if(this.z_length_ != otherMyClass.z_length_) return false;

      if(this.x_radius_ != otherMyClass.x_radius_) return false;

      if(this.y_radius_ != otherMyClass.y_radius_) return false;

      if(this.z_radius_ != otherMyClass.z_radius_) return false;

      if (!this.provided_pose_.equals(otherMyClass.provided_pose_)) return false;
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
      builder.append("x_length=");
      builder.append(this.x_length_);      builder.append(", ");
      builder.append("y_length=");
      builder.append(this.y_length_);      builder.append(", ");
      builder.append("z_length=");
      builder.append(this.z_length_);      builder.append(", ");
      builder.append("x_radius=");
      builder.append(this.x_radius_);      builder.append(", ");
      builder.append("y_radius=");
      builder.append(this.y_radius_);      builder.append(", ");
      builder.append("z_radius=");
      builder.append(this.z_radius_);      builder.append(", ");
      builder.append("provided_pose=");
      builder.append(this.provided_pose_);      builder.append(", ");
      builder.append("run_icp=");
      builder.append(this.run_icp_);      builder.append(", ");
      builder.append("use_provided_pose=");
      builder.append(this.use_provided_pose_);
      builder.append("}");
      return builder.toString();
   }
}
