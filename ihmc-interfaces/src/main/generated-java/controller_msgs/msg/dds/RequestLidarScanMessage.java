package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

/**
 * Message used to request a new LIDAR scan, and also enable some filters.
 */
public class RequestLidarScanMessage extends Packet<RequestLidarScanMessage>
      implements Settable<RequestLidarScanMessage>, EpsilonComparable<RequestLidarScanMessage>
{
   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header header_;
   public boolean remove_shadows_ = true;
   public boolean remove_self_collisions_ = true;

   public RequestLidarScanMessage()
   {
      header_ = new std_msgs.msg.dds.Header();
   }

   public RequestLidarScanMessage(RequestLidarScanMessage other)
   {
      this();
      set(other);
   }

   public void set(RequestLidarScanMessage other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      remove_shadows_ = other.remove_shadows_;

      remove_self_collisions_ = other.remove_self_collisions_;

   }

   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }

   public void setRemoveShadows(boolean remove_shadows)
   {
      remove_shadows_ = remove_shadows;
   }

   public boolean getRemoveShadows()
   {
      return remove_shadows_;
   }

   public void setRemoveSelfCollisions(boolean remove_self_collisions)
   {
      remove_self_collisions_ = remove_self_collisions;
   }

   public boolean getRemoveSelfCollisions()
   {
      return remove_self_collisions_;
   }

   @Override
   public boolean epsilonEquals(RequestLidarScanMessage other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon))
         return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.remove_shadows_, other.remove_shadows_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.remove_self_collisions_, other.remove_self_collisions_, epsilon))
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
      if (!(other instanceof RequestLidarScanMessage))
         return false;

      RequestLidarScanMessage otherMyClass = (RequestLidarScanMessage) other;

      if (!this.header_.equals(otherMyClass.header_))
         return false;
      if (this.remove_shadows_ != otherMyClass.remove_shadows_)
         return false;

      if (this.remove_self_collisions_ != otherMyClass.remove_self_collisions_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("RequestLidarScanMessage {");
      builder.append("header=");
      builder.append(this.header_);
      builder.append(", ");
      builder.append("remove_shadows=");
      builder.append(this.remove_shadows_);
      builder.append(", ");
      builder.append("remove_self_collisions=");
      builder.append(this.remove_self_collisions_);
      builder.append("}");
      return builder.toString();
   }
}
