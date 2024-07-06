package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class GlobalMapCellMap extends Packet<GlobalMapCellMap> implements Settable<GlobalMapCellMap>, EpsilonComparable<GlobalMapCellMap>
{
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.GlobalMapCellEntry>  global_map_cells_;

   public GlobalMapCellMap()
   {
      global_map_cells_ = new us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.GlobalMapCellEntry> (100, new perception_msgs.msg.dds.GlobalMapCellEntryPubSubType());

   }

   public GlobalMapCellMap(GlobalMapCellMap other)
   {
      this();
      set(other);
   }

   public void set(GlobalMapCellMap other)
   {
      global_map_cells_.set(other.global_map_cells_);
   }


   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.GlobalMapCellEntry>  getGlobalMapCells()
   {
      return global_map_cells_;
   }


   public static Supplier<GlobalMapCellMapPubSubType> getPubSubType()
   {
      return GlobalMapCellMapPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return GlobalMapCellMapPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(GlobalMapCellMap other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (this.global_map_cells_.size() != other.global_map_cells_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.global_map_cells_.size(); i++)
         {  if (!this.global_map_cells_.get(i).epsilonEquals(other.global_map_cells_.get(i), epsilon)) return false; }
      }

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof GlobalMapCellMap)) return false;

      GlobalMapCellMap otherMyClass = (GlobalMapCellMap) other;

      if (!this.global_map_cells_.equals(otherMyClass.global_map_cells_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("GlobalMapCellMap {");
      builder.append("global_map_cells=");
      builder.append(this.global_map_cells_);
      builder.append("}");
      return builder.toString();
   }
}
