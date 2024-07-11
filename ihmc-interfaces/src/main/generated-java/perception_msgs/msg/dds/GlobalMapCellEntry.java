package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class GlobalMapCellEntry extends Packet<GlobalMapCellEntry> implements Settable<GlobalMapCellEntry>, EpsilonComparable<GlobalMapCellEntry>
{
   public int key_;
   public int x_index_;
   public int y_index_;
   public double cell_height_;
   public double resolution_;

   public GlobalMapCellEntry()
   {
   }

   public GlobalMapCellEntry(GlobalMapCellEntry other)
   {
      this();
      set(other);
   }

   public void set(GlobalMapCellEntry other)
   {
      key_ = other.key_;

      x_index_ = other.x_index_;

      y_index_ = other.y_index_;

      cell_height_ = other.cell_height_;

      resolution_ = other.resolution_;

   }

   public void setKey(int key)
   {
      key_ = key;
   }
   public int getKey()
   {
      return key_;
   }

   public void setXIndex(int x_index)
   {
      x_index_ = x_index;
   }
   public int getXIndex()
   {
      return x_index_;
   }

   public void setYIndex(int y_index)
   {
      y_index_ = y_index;
   }
   public int getYIndex()
   {
      return y_index_;
   }

   public void setCellHeight(double cell_height)
   {
      cell_height_ = cell_height;
   }
   public double getCellHeight()
   {
      return cell_height_;
   }

   public void setResolution(double resolution)
   {
      resolution_ = resolution;
   }
   public double getResolution()
   {
      return resolution_;
   }


   public static Supplier<GlobalMapCellEntryPubSubType> getPubSubType()
   {
      return GlobalMapCellEntryPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return GlobalMapCellEntryPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(GlobalMapCellEntry other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.key_, other.key_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.x_index_, other.x_index_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.y_index_, other.y_index_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.cell_height_, other.cell_height_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.resolution_, other.resolution_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof GlobalMapCellEntry)) return false;

      GlobalMapCellEntry otherMyClass = (GlobalMapCellEntry) other;

      if(this.key_ != otherMyClass.key_) return false;

      if(this.x_index_ != otherMyClass.x_index_) return false;

      if(this.y_index_ != otherMyClass.y_index_) return false;

      if(this.cell_height_ != otherMyClass.cell_height_) return false;

      if(this.resolution_ != otherMyClass.resolution_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("GlobalMapCellEntry {");
      builder.append("key=");
      builder.append(this.key_);      builder.append(", ");
      builder.append("x_index=");
      builder.append(this.x_index_);      builder.append(", ");
      builder.append("y_index=");
      builder.append(this.y_index_);      builder.append(", ");
      builder.append("cell_height=");
      builder.append(this.cell_height_);      builder.append(", ");
      builder.append("resolution=");
      builder.append(this.resolution_);
      builder.append("}");
      return builder.toString();
   }
}
