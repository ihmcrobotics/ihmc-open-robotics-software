package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC height map module
       */
public class HeightMapMessage extends Packet<HeightMapMessage> implements Settable<HeightMapMessage>, EpsilonComparable<HeightMapMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Discretization of the height map grid
            */
   public double xy_resolution_ = -1.0;
   /**
            * The height map covers a square of this width
            */
   public double grid_size_xy_ = -1.0;
   /**
            * X coordinate of the center of the height map
            */
   public double grid_center_x_;
   /**
            * Y coordinate of the center of the height map
            */
   public double grid_center_y_;
   /**
            * Height of the ground plane, which is assumed to be flat
            */
   public double estimated_ground_height_;
   /**
            * List of height map keys. See HeightMapTools for converting keys to coordinates
            */
   public us.ihmc.idl.IDLSequence.Integer  keys_;
   /**
            * List of heights, which correspond to the list of keys
            */
   public us.ihmc.idl.IDLSequence.Float  heights_;
   /**
            * List of variances, which correspond to the list of keys. May be empty.
            */
   public us.ihmc.idl.IDLSequence.Float  variances_;
   /**
            * List of centroids for each cell, which correspond to the list of keys. May be empty
            * Note: The z coordinate of each point is ignored, but should correspond to the height.
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  centroids_;
   /**
            * List of normals for each cell, which correspond to the list of keys. May be empty.
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Vector3D>  normals_;

   public HeightMapMessage()
   {
      keys_ = new us.ihmc.idl.IDLSequence.Integer (45000, "type_2");

      heights_ = new us.ihmc.idl.IDLSequence.Float (45000, "type_5");

      variances_ = new us.ihmc.idl.IDLSequence.Float (45000, "type_5");

      centroids_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D> (45000, new geometry_msgs.msg.dds.PointPubSubType());
      normals_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Vector3D> (45000, new geometry_msgs.msg.dds.Vector3PubSubType());

   }

   public HeightMapMessage(HeightMapMessage other)
   {
      this();
      set(other);
   }

   public void set(HeightMapMessage other)
   {
      sequence_id_ = other.sequence_id_;

      xy_resolution_ = other.xy_resolution_;

      grid_size_xy_ = other.grid_size_xy_;

      grid_center_x_ = other.grid_center_x_;

      grid_center_y_ = other.grid_center_y_;

      estimated_ground_height_ = other.estimated_ground_height_;

      keys_.set(other.keys_);
      heights_.set(other.heights_);
      variances_.set(other.variances_);
      centroids_.set(other.centroids_);
      normals_.set(other.normals_);
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
            * Discretization of the height map grid
            */
   public void setXyResolution(double xy_resolution)
   {
      xy_resolution_ = xy_resolution;
   }
   /**
            * Discretization of the height map grid
            */
   public double getXyResolution()
   {
      return xy_resolution_;
   }

   /**
            * The height map covers a square of this width
            */
   public void setGridSizeXy(double grid_size_xy)
   {
      grid_size_xy_ = grid_size_xy;
   }
   /**
            * The height map covers a square of this width
            */
   public double getGridSizeXy()
   {
      return grid_size_xy_;
   }

   /**
            * X coordinate of the center of the height map
            */
   public void setGridCenterX(double grid_center_x)
   {
      grid_center_x_ = grid_center_x;
   }
   /**
            * X coordinate of the center of the height map
            */
   public double getGridCenterX()
   {
      return grid_center_x_;
   }

   /**
            * Y coordinate of the center of the height map
            */
   public void setGridCenterY(double grid_center_y)
   {
      grid_center_y_ = grid_center_y;
   }
   /**
            * Y coordinate of the center of the height map
            */
   public double getGridCenterY()
   {
      return grid_center_y_;
   }

   /**
            * Height of the ground plane, which is assumed to be flat
            */
   public void setEstimatedGroundHeight(double estimated_ground_height)
   {
      estimated_ground_height_ = estimated_ground_height;
   }
   /**
            * Height of the ground plane, which is assumed to be flat
            */
   public double getEstimatedGroundHeight()
   {
      return estimated_ground_height_;
   }


   /**
            * List of height map keys. See HeightMapTools for converting keys to coordinates
            */
   public us.ihmc.idl.IDLSequence.Integer  getKeys()
   {
      return keys_;
   }


   /**
            * List of heights, which correspond to the list of keys
            */
   public us.ihmc.idl.IDLSequence.Float  getHeights()
   {
      return heights_;
   }


   /**
            * List of variances, which correspond to the list of keys. May be empty.
            */
   public us.ihmc.idl.IDLSequence.Float  getVariances()
   {
      return variances_;
   }


   /**
            * List of centroids for each cell, which correspond to the list of keys. May be empty
            * Note: The z coordinate of each point is ignored, but should correspond to the height.
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  getCentroids()
   {
      return centroids_;
   }


   /**
            * List of normals for each cell, which correspond to the list of keys. May be empty.
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Vector3D>  getNormals()
   {
      return normals_;
   }


   public static Supplier<HeightMapMessagePubSubType> getPubSubType()
   {
      return HeightMapMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return HeightMapMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(HeightMapMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.xy_resolution_, other.xy_resolution_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.grid_size_xy_, other.grid_size_xy_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.grid_center_x_, other.grid_center_x_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.grid_center_y_, other.grid_center_y_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.estimated_ground_height_, other.estimated_ground_height_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsIntegerSequence(this.keys_, other.keys_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.heights_, other.heights_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.variances_, other.variances_, epsilon)) return false;

      if (this.centroids_.size() != other.centroids_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.centroids_.size(); i++)
         {  if (!this.centroids_.get(i).epsilonEquals(other.centroids_.get(i), epsilon)) return false; }
      }

      if (this.normals_.size() != other.normals_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.normals_.size(); i++)
         {  if (!this.normals_.get(i).epsilonEquals(other.normals_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof HeightMapMessage)) return false;

      HeightMapMessage otherMyClass = (HeightMapMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.xy_resolution_ != otherMyClass.xy_resolution_) return false;

      if(this.grid_size_xy_ != otherMyClass.grid_size_xy_) return false;

      if(this.grid_center_x_ != otherMyClass.grid_center_x_) return false;

      if(this.grid_center_y_ != otherMyClass.grid_center_y_) return false;

      if(this.estimated_ground_height_ != otherMyClass.estimated_ground_height_) return false;

      if (!this.keys_.equals(otherMyClass.keys_)) return false;
      if (!this.heights_.equals(otherMyClass.heights_)) return false;
      if (!this.variances_.equals(otherMyClass.variances_)) return false;
      if (!this.centroids_.equals(otherMyClass.centroids_)) return false;
      if (!this.normals_.equals(otherMyClass.normals_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("HeightMapMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("xy_resolution=");
      builder.append(this.xy_resolution_);      builder.append(", ");
      builder.append("grid_size_xy=");
      builder.append(this.grid_size_xy_);      builder.append(", ");
      builder.append("grid_center_x=");
      builder.append(this.grid_center_x_);      builder.append(", ");
      builder.append("grid_center_y=");
      builder.append(this.grid_center_y_);      builder.append(", ");
      builder.append("estimated_ground_height=");
      builder.append(this.estimated_ground_height_);      builder.append(", ");
      builder.append("keys=");
      builder.append(this.keys_);      builder.append(", ");
      builder.append("heights=");
      builder.append(this.heights_);      builder.append(", ");
      builder.append("variances=");
      builder.append(this.variances_);      builder.append(", ");
      builder.append("centroids=");
      builder.append(this.centroids_);      builder.append(", ");
      builder.append("normals=");
      builder.append(this.normals_);
      builder.append("}");
      return builder.toString();
   }
}
