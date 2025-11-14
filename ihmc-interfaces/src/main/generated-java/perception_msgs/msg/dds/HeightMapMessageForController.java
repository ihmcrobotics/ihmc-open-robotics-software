package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC height map module
       */
public class HeightMapMessageForController extends Packet<HeightMapMessageForController> implements Settable<HeightMapMessageForController>, EpsilonComparable<HeightMapMessageForController>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * X coordinate of the center of the height map
            */
   public double grid_center_x_;
   /**
            * Y coordinate of the center of the height map
            */
   public double grid_center_y_;
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
            * List of heights, which correspond to the list of keys
            */
   public us.ihmc.idl.IDLSequence.Float  heights_;

   public HeightMapMessageForController()
   {
      heights_ = new us.ihmc.idl.IDLSequence.Float (255000, "type_5");

   }

   public HeightMapMessageForController(HeightMapMessageForController other)
   {
      this();
      set(other);
   }

   public void set(HeightMapMessageForController other)
   {
      sequence_id_ = other.sequence_id_;

      grid_center_x_ = other.grid_center_x_;

      grid_center_y_ = other.grid_center_y_;

      width_in_meters_ = other.width_in_meters_;

      cell_size_in_meters_ = other.cell_size_in_meters_;

      cells_per_axis_ = other.cells_per_axis_;

      heights_.set(other.heights_);
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
            * List of heights, which correspond to the list of keys
            */
   public us.ihmc.idl.IDLSequence.Float  getHeights()
   {
      return heights_;
   }


   public static Supplier<HeightMapMessageForControllerPubSubType> getPubSubType()
   {
      return HeightMapMessageForControllerPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return HeightMapMessageForControllerPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(HeightMapMessageForController other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.grid_center_x_, other.grid_center_x_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.grid_center_y_, other.grid_center_y_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.width_in_meters_, other.width_in_meters_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.cell_size_in_meters_, other.cell_size_in_meters_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.cells_per_axis_, other.cells_per_axis_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.heights_, other.heights_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof HeightMapMessageForController)) return false;

      HeightMapMessageForController otherMyClass = (HeightMapMessageForController) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.grid_center_x_ != otherMyClass.grid_center_x_) return false;

      if(this.grid_center_y_ != otherMyClass.grid_center_y_) return false;

      if(this.width_in_meters_ != otherMyClass.width_in_meters_) return false;

      if(this.cell_size_in_meters_ != otherMyClass.cell_size_in_meters_) return false;

      if(this.cells_per_axis_ != otherMyClass.cells_per_axis_) return false;

      if (!this.heights_.equals(otherMyClass.heights_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("HeightMapMessageForController {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("grid_center_x=");
      builder.append(this.grid_center_x_);      builder.append(", ");
      builder.append("grid_center_y=");
      builder.append(this.grid_center_y_);      builder.append(", ");
      builder.append("width_in_meters=");
      builder.append(this.width_in_meters_);      builder.append(", ");
      builder.append("cell_size_in_meters=");
      builder.append(this.cell_size_in_meters_);      builder.append(", ");
      builder.append("cells_per_axis=");
      builder.append(this.cells_per_axis_);      builder.append(", ");
      builder.append("heights=");
      builder.append(this.heights_);
      builder.append("}");
      return builder.toString();
   }
}
