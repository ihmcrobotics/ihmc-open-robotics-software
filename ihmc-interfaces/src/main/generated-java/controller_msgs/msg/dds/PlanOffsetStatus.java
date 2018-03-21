package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * This message is part of the IHMC whole-body controller API.
 * The controller sends this message to notify that it has shifted the remaining footsteps to be executed due to some execution error.
 */
public class PlanOffsetStatus extends Packet<PlanOffsetStatus> implements Settable<PlanOffsetStatus>, EpsilonComparable<PlanOffsetStatus>
{
   /**
    * The amount by which the remaining footsteps have been translated.
    */
   public us.ihmc.euclid.tuple3D.Vector3D offset_vector_;

   public PlanOffsetStatus()
   {
      offset_vector_ = new us.ihmc.euclid.tuple3D.Vector3D();
   }

   public PlanOffsetStatus(PlanOffsetStatus other)
   {
      set(other);
   }

   public void set(PlanOffsetStatus other)
   {
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.offset_vector_, offset_vector_);
   }

   /**
    * The amount by which the remaining footsteps have been translated.
    */
   public us.ihmc.euclid.tuple3D.Vector3D getOffsetVector()
   {
      return offset_vector_;
   }

   @Override
   public boolean epsilonEquals(PlanOffsetStatus other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.offset_vector_.epsilonEquals(other.offset_vector_, epsilon))
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
      if (!(other instanceof PlanOffsetStatus))
         return false;

      PlanOffsetStatus otherMyClass = (PlanOffsetStatus) other;

      if (!this.offset_vector_.equals(otherMyClass.offset_vector_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("PlanOffsetStatus {");
      builder.append("offset_vector=");
      builder.append(this.offset_vector_);

      builder.append("}");
      return builder.toString();
   }
}