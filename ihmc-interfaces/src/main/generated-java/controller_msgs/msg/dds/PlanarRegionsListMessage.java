package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

/**
 * This message is part of the IHMC robot environment awareness module. This message contains a list
 * of planar regions.
 */
public class PlanarRegionsListMessage extends Packet<PlanarRegionsListMessage>
      implements Settable<PlanarRegionsListMessage>, EpsilonComparable<PlanarRegionsListMessage>
{
   /**
    * Unique ID used to identify this message, should preferably be consecutively increasing.
    */
   public long sequence_id_;
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.PlanarRegionMessage> planar_regions_;

   public PlanarRegionsListMessage()
   {
      planar_regions_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.PlanarRegionMessage>(100, controller_msgs.msg.dds.PlanarRegionMessage.class,
                                                                                                        new controller_msgs.msg.dds.PlanarRegionMessagePubSubType());

   }

   public PlanarRegionsListMessage(PlanarRegionsListMessage other)
   {
      this();
      set(other);
   }

   public void set(PlanarRegionsListMessage other)
   {
      sequence_id_ = other.sequence_id_;

      planar_regions_.set(other.planar_regions_);
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

   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.PlanarRegionMessage> getPlanarRegions()
   {
      return planar_regions_;
   }

   @Override
   public boolean epsilonEquals(PlanarRegionsListMessage other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon))
         return false;

      if (this.planar_regions_.size() == other.planar_regions_.size())
      {
         return false;
      }
      else
      {
         for (int i = 0; i < this.planar_regions_.size(); i++)
         {
            if (!this.planar_regions_.get(i).epsilonEquals(other.planar_regions_.get(i), epsilon))
               return false;
         }
      }

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;
      if (!(other instanceof PlanarRegionsListMessage))
         return false;

      PlanarRegionsListMessage otherMyClass = (PlanarRegionsListMessage) other;

      if (this.sequence_id_ != otherMyClass.sequence_id_)
         return false;

      if (!this.planar_regions_.equals(otherMyClass.planar_regions_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("PlanarRegionsListMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);
      builder.append(", ");
      builder.append("planar_regions=");
      builder.append(this.planar_regions_);
      builder.append("}");
      return builder.toString();
   }
}
