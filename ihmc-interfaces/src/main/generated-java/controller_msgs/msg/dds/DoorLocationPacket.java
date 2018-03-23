package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * This message is part of the IHMC humanoid behavior module.
 */
public class DoorLocationPacket extends Packet<DoorLocationPacket> implements Settable<DoorLocationPacket>, EpsilonComparable<DoorLocationPacket>
{
   public us.ihmc.euclid.geometry.Pose3D door_transform_to_world_;

   public DoorLocationPacket()
   {
      door_transform_to_world_ = new us.ihmc.euclid.geometry.Pose3D();
   }

   public DoorLocationPacket(DoorLocationPacket other)
   {
      set(other);
   }

   public void set(DoorLocationPacket other)
   {
      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.door_transform_to_world_, door_transform_to_world_);
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

      if (!this.door_transform_to_world_.equals(otherMyClass.door_transform_to_world_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("DoorLocationPacket {");
      builder.append("door_transform_to_world=");
      builder.append(this.door_transform_to_world_);

      builder.append("}");
      return builder.toString();
   }
}
