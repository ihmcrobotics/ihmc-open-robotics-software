package controller_msgs.msg.dds;

import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * This message is part of the IHMC robot environment awareness module.
 * This message contains a list of planar regions.
 */
public class PlanarRegionsListMessage implements Settable<PlanarRegionsListMessage>, EpsilonComparable<PlanarRegionsListMessage>
{
   private us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.PlanarRegionMessage> planar_regions_;

   public PlanarRegionsListMessage()
   {
      planar_regions_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.PlanarRegionMessage>(100, controller_msgs.msg.dds.PlanarRegionMessage.class,
                                                                                                        new controller_msgs.msg.dds.PlanarRegionMessagePubSubType());
   }

   public PlanarRegionsListMessage(PlanarRegionsListMessage other)
   {
      set(other);
   }

   public void set(PlanarRegionsListMessage other)
   {
      planar_regions_.set(other.planar_regions_);
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

      if (this.planar_regions_.isEnum())
      {
         if (!this.planar_regions_.equals(other.planar_regions_))
            return false;
      }
      else if (this.planar_regions_.size() == other.planar_regions_.size())
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

      if (!this.planar_regions_.equals(otherMyClass.planar_regions_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("PlanarRegionsListMessage {");
      builder.append("planar_regions=");
      builder.append(this.planar_regions_);

      builder.append("}");
      return builder.toString();
   }
}