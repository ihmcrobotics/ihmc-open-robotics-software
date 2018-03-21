package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * This message is part of the IHMC humanoid behavior module.
 */
public class ValveLocationPacket extends Packet<ValveLocationPacket> implements Settable<ValveLocationPacket>, EpsilonComparable<ValveLocationPacket>
{
   public us.ihmc.euclid.geometry.Pose3D valve_pose_in_world_;
   public double valve_radius_;

   public ValveLocationPacket()
   {
      valve_pose_in_world_ = new us.ihmc.euclid.geometry.Pose3D();
   }

   public ValveLocationPacket(ValveLocationPacket other)
   {
      set(other);
   }

   public void set(ValveLocationPacket other)
   {
      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.valve_pose_in_world_, valve_pose_in_world_);
      valve_radius_ = other.valve_radius_;
   }

   public us.ihmc.euclid.geometry.Pose3D getValvePoseInWorld()
   {
      return valve_pose_in_world_;
   }

   public double getValveRadius()
   {
      return valve_radius_;
   }

   public void setValveRadius(double valve_radius)
   {
      valve_radius_ = valve_radius;
   }

   @Override
   public boolean epsilonEquals(ValveLocationPacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.valve_pose_in_world_.epsilonEquals(other.valve_pose_in_world_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.valve_radius_, other.valve_radius_, epsilon))
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
      if (!(other instanceof ValveLocationPacket))
         return false;

      ValveLocationPacket otherMyClass = (ValveLocationPacket) other;

      if (!this.valve_pose_in_world_.equals(otherMyClass.valve_pose_in_world_))
         return false;

      if (this.valve_radius_ != otherMyClass.valve_radius_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ValveLocationPacket {");
      builder.append("valve_pose_in_world=");
      builder.append(this.valve_pose_in_world_);

      builder.append(", ");
      builder.append("valve_radius=");
      builder.append(this.valve_radius_);

      builder.append("}");
      return builder.toString();
   }
}