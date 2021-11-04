package controller_msgs.msg.dds;

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
   public double xy_resolution_ = -1.0;
   public double grid_size_xy_ = -1.0;
   public double grid_center_x_;
   public double grid_center_y_;
   public us.ihmc.idl.IDLSequence.Integer  x_cells_;
   public us.ihmc.idl.IDLSequence.Integer  y_cells_;
   public us.ihmc.idl.IDLSequence.Float  heights_;

   public HeightMapMessage()
   {
      x_cells_ = new us.ihmc.idl.IDLSequence.Integer (30000, "type_2");

      y_cells_ = new us.ihmc.idl.IDLSequence.Integer (30000, "type_2");

      heights_ = new us.ihmc.idl.IDLSequence.Float (30000, "type_5");

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

      x_cells_.set(other.x_cells_);
      y_cells_.set(other.y_cells_);
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

   public void setXyResolution(double xy_resolution)
   {
      xy_resolution_ = xy_resolution;
   }
   public double getXyResolution()
   {
      return xy_resolution_;
   }

   public void setGridSizeXy(double grid_size_xy)
   {
      grid_size_xy_ = grid_size_xy;
   }
   public double getGridSizeXy()
   {
      return grid_size_xy_;
   }

   public void setGridCenterX(double grid_center_x)
   {
      grid_center_x_ = grid_center_x;
   }
   public double getGridCenterX()
   {
      return grid_center_x_;
   }

   public void setGridCenterY(double grid_center_y)
   {
      grid_center_y_ = grid_center_y;
   }
   public double getGridCenterY()
   {
      return grid_center_y_;
   }


   public us.ihmc.idl.IDLSequence.Integer  getXCells()
   {
      return x_cells_;
   }


   public us.ihmc.idl.IDLSequence.Integer  getYCells()
   {
      return y_cells_;
   }


   public us.ihmc.idl.IDLSequence.Float  getHeights()
   {
      return heights_;
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

      if (!us.ihmc.idl.IDLTools.epsilonEqualsIntegerSequence(this.x_cells_, other.x_cells_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsIntegerSequence(this.y_cells_, other.y_cells_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.heights_, other.heights_, epsilon)) return false;


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

      if (!this.x_cells_.equals(otherMyClass.x_cells_)) return false;
      if (!this.y_cells_.equals(otherMyClass.y_cells_)) return false;
      if (!this.heights_.equals(otherMyClass.heights_)) return false;

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
      builder.append("x_cells=");
      builder.append(this.x_cells_);      builder.append(", ");
      builder.append("y_cells=");
      builder.append(this.y_cells_);      builder.append(", ");
      builder.append("heights=");
      builder.append(this.heights_);
      builder.append("}");
      return builder.toString();
   }
}
