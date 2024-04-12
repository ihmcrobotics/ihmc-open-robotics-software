package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class YOLOv8DoorNodeMessage extends Packet<YOLOv8DoorNodeMessage> implements Settable<YOLOv8DoorNodeMessage>, EpsilonComparable<YOLOv8DoorNodeMessage>
{
   /**
            * TODO: Add door stuff
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D32>  presumed_door_point_cloud_;

   public YOLOv8DoorNodeMessage()
   {
      presumed_door_point_cloud_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D32> (5000, new geometry_msgs.msg.dds.Point32PubSubType());

   }

   public YOLOv8DoorNodeMessage(YOLOv8DoorNodeMessage other)
   {
      this();
      set(other);
   }

   public void set(YOLOv8DoorNodeMessage other)
   {
      presumed_door_point_cloud_.set(other.presumed_door_point_cloud_);
   }


   /**
            * TODO: Add door stuff
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D32>  getPresumedDoorPointCloud()
   {
      return presumed_door_point_cloud_;
   }


   public static Supplier<YOLOv8DoorNodeMessagePubSubType> getPubSubType()
   {
      return YOLOv8DoorNodeMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return YOLOv8DoorNodeMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(YOLOv8DoorNodeMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (this.presumed_door_point_cloud_.size() != other.presumed_door_point_cloud_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.presumed_door_point_cloud_.size(); i++)
         {  if (!this.presumed_door_point_cloud_.get(i).epsilonEquals(other.presumed_door_point_cloud_.get(i), epsilon)) return false; }
      }

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof YOLOv8DoorNodeMessage)) return false;

      YOLOv8DoorNodeMessage otherMyClass = (YOLOv8DoorNodeMessage) other;

      if (!this.presumed_door_point_cloud_.equals(otherMyClass.presumed_door_point_cloud_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("YOLOv8DoorNodeMessage {");
      builder.append("presumed_door_point_cloud=");
      builder.append(this.presumed_door_point_cloud_);
      builder.append("}");
      return builder.toString();
   }
}
