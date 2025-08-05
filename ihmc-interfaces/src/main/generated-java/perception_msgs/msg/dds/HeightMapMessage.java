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
            * Z height offset for converting between floats and shorts
            */
   public double height_offset_;
   /**
            * Z height scale factor for converting between floats and shorts
            */
   public double height_scale_factor_;
   /**
            * Height of the ground plane, which is assumed to be flat
            */
   public double estimated_ground_height_;
   /**
            * Width of the height map in meters
            */
   public double width_in_meters_;
   /**
            * Cell size in meters of an individual cell
            */
   public double cell_size_in_meters_;
   /**
            * Cells per axis
            */
   public int cells_per_axis_;
   /**
            * List of height map keys. See HeightMapTools for converting keys to coordinates
            */
   public us.ihmc.idl.IDLSequence.Integer  keys_;
   /**
            * List of heights, which correspond to the list of keys
            */
   public us.ihmc.idl.IDLSequence.Integer  heights_;
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
      keys_ = new us.ihmc.idl.IDLSequence.Integer (255000, "type_2");

      heights_ = new us.ihmc.idl.IDLSequence.Integer (255000, "type_2");

      variances_ = new us.ihmc.idl.IDLSequence.Float (255000, "type_5");

      centroids_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D> (255000, new geometry_msgs.msg.dds.PointPubSubType());
      normals_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Vector3D> (255000, new geometry_msgs.msg.dds.Vector3PubSubType());

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

      height_offset_ = other.height_offset_;

      height_scale_factor_ = other.height_scale_factor_;

      estimated_ground_height_ = other.estimated_ground_height_;

      width_in_meters_ = other.width_in_meters_;

      cell_size_in_meters_ = other.cell_size_in_meters_;

      cells_per_axis_ = other.cells_per_axis_;

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
            * Z height offset for converting between floats and shorts
            */
   public void setHeightOffset(double height_offset)
   {
      height_offset_ = height_offset;
   }
   /**
            * Z height offset for converting between floats and shorts
            */
   public double getHeightOffset()
   {
      return height_offset_;
   }

   /**
            * Z height scale factor for converting between floats and shorts
            */
   public void setHeightScaleFactor(double height_scale_factor)
   {
      height_scale_factor_ = height_scale_factor;
   }
   /**
            * Z height scale factor for converting between floats and shorts
            */
   public double getHeightScaleFactor()
   {
      return height_scale_factor_;
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
            * Width of the height map in meters
            */
   public void setWidthInMeters(double width_in_meters)
   {
      width_in_meters_ = width_in_meters;
   }
   /**
            * Width of the height map in meters
            */
   public double getWidthInMeters()
   {
      return width_in_meters_;
   }

   /**
            * Cell size in meters of an individual cell
            */
   public void setCellSizeInMeters(double cell_size_in_meters)
   {
      cell_size_in_meters_ = cell_size_in_meters;
   }
   /**
            * Cell size in meters of an individual cell
            */
   public double getCellSizeInMeters()
   {
      return cell_size_in_meters_;
   }

   /**
            * Cells per axis
            */
   public void setCellsPerAxis(int cells_per_axis)
   {
      cells_per_axis_ = cells_per_axis;
   }
   /**
            * Cells per axis
            */
   public int getCellsPerAxis()
   {
      return cells_per_axis_;
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
   public us.ihmc.idl.IDLSequence.Integer  getHeights()
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

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.height_offset_, other.height_offset_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.height_scale_factor_, other.height_scale_factor_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.estimated_ground_height_, other.estimated_ground_height_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.width_in_meters_, other.width_in_meters_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.cell_size_in_meters_, other.cell_size_in_meters_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.cells_per_axis_, other.cells_per_axis_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsIntegerSequence(this.keys_, other.keys_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsIntegerSequence(this.heights_, other.heights_, epsilon)) return false;

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

      if(this.height_offset_ != otherMyClass.height_offset_) return false;

      if(this.height_scale_factor_ != otherMyClass.height_scale_factor_) return false;

      if(this.estimated_ground_height_ != otherMyClass.estimated_ground_height_) return false;

      if(this.width_in_meters_ != otherMyClass.width_in_meters_) return false;

      if(this.cell_size_in_meters_ != otherMyClass.cell_size_in_meters_) return false;

      if(this.cells_per_axis_ != otherMyClass.cells_per_axis_) return false;

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
      builder.append("height_offset=");
      builder.append(this.height_offset_);      builder.append(", ");
      builder.append("height_scale_factor=");
      builder.append(this.height_scale_factor_);      builder.append(", ");
      builder.append("estimated_ground_height=");
      builder.append(this.estimated_ground_height_);      builder.append(", ");
      builder.append("width_in_meters=");
      builder.append(this.width_in_meters_);      builder.append(", ");
      builder.append("cell_size_in_meters=");
      builder.append(this.cell_size_in_meters_);      builder.append(", ");
      builder.append("cells_per_axis=");
      builder.append(this.cells_per_axis_);      builder.append(", ");
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
