package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

/**
 * This message is part of the IHMC humanoid behavior module.
 */
public class DoorLocationPacket extends Packet<DoorLocationPacket> implements Settable<DoorLocationPacket>, EpsilonComparable<DoorLocationPacket>
{
   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header header_;
   public us.ihmc.euclid.geometry.Pose3D door_transform_to_world_;

   public DoorLocationPacket()
   {
      header_ = new std_msgs.msg.dds.Header();
      door_transform_to_world_ = new us.ihmc.euclid.geometry.Pose3D();
   }

   public DoorLocationPacket(DoorLocationPacket other)
   {
      this();
      set(other);
   }

   public void set(DoorLocationPacket other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.door_transform_to_world_, door_transform_to_world_);
   }

   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }

   public us.ihmc.euclid.geometry.Pose3D getDoorTransformToWorld()
   {
      return door_transform_to_world_;
   }

   @Override
   public boolean epsilonEquals(DoorLocationPacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon))
         return false;
      if (!this.door_transform_to_world_.epsilonEquals(other.door_transform_to_world_, epsilon))
         return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;
      if (!(other instanceof DoorLocationPacket))
         return false;

      DoorLocationPacket otherMyClass = (DoorLocationPacket) other;

      if (!this.header_.equals(otherMyClass.header_))
         return false;
      if (!this.door_transform_to_world_.equals(otherMyClass.door_transform_to_world_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("DoorLocationPacket {");
      builder.append("header=");
      builder.append(this.header_);
      builder.append(", ");
      builder.append("door_transform_to_world=");
      builder.append(this.door_transform_to_world_);
      builder.append("}");
      return builder.toString();
   }
}
