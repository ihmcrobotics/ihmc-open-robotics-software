package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class GlobalMapTileMessage extends Packet<GlobalMapTileMessage> implements Settable<GlobalMapTileMessage>, EpsilonComparable<GlobalMapTileMessage>
{
   public int center_x_;
   public int center_y_;
   public int hash_code_of_tile_;
   public perception_msgs.msg.dds.HeightMapMessage height_map_;

   public GlobalMapTileMessage()
   {
      height_map_ = new perception_msgs.msg.dds.HeightMapMessage();
   }

   public GlobalMapTileMessage(GlobalMapTileMessage other)
   {
      this();
      set(other);
   }

   public void set(GlobalMapTileMessage other)
   {
      center_x_ = other.center_x_;

      center_y_ = other.center_y_;

      hash_code_of_tile_ = other.hash_code_of_tile_;

      perception_msgs.msg.dds.HeightMapMessagePubSubType.staticCopy(other.height_map_, height_map_);
   }

   public void setCenterX(int center_x)
   {
      center_x_ = center_x;
   }
   public int getCenterX()
   {
      return center_x_;
   }

   public void setCenterY(int center_y)
   {
      center_y_ = center_y;
   }
   public int getCenterY()
   {
      return center_y_;
   }

   public void setHashCodeOfTile(int hash_code_of_tile)
   {
      hash_code_of_tile_ = hash_code_of_tile;
   }
   public int getHashCodeOfTile()
   {
      return hash_code_of_tile_;
   }


   public perception_msgs.msg.dds.HeightMapMessage getHeightMap()
   {
      return height_map_;
   }


   public static Supplier<GlobalMapTileMessagePubSubType> getPubSubType()
   {
      return GlobalMapTileMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return GlobalMapTileMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(GlobalMapTileMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.center_x_, other.center_x_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.center_y_, other.center_y_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.hash_code_of_tile_, other.hash_code_of_tile_, epsilon)) return false;

      if (!this.height_map_.epsilonEquals(other.height_map_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof GlobalMapTileMessage)) return false;

      GlobalMapTileMessage otherMyClass = (GlobalMapTileMessage) other;

      if(this.center_x_ != otherMyClass.center_x_) return false;

      if(this.center_y_ != otherMyClass.center_y_) return false;

      if(this.hash_code_of_tile_ != otherMyClass.hash_code_of_tile_) return false;

      if (!this.height_map_.equals(otherMyClass.height_map_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("GlobalMapTileMessage {");
      builder.append("center_x=");
      builder.append(this.center_x_);      builder.append(", ");
      builder.append("center_y=");
      builder.append(this.center_y_);      builder.append(", ");
      builder.append("hash_code_of_tile=");
      builder.append(this.hash_code_of_tile_);      builder.append(", ");
      builder.append("height_map=");
      builder.append(this.height_map_);
      builder.append("}");
      return builder.toString();
   }
}
