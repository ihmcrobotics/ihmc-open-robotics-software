package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

/**
 * This message is part of the IHMC whole-body controller API. This message is sent by the
 * controller to notify the user that the current manipulation task has been aborted.
 */
public class ManipulationAbortedStatus extends Packet<ManipulationAbortedStatus>
      implements Settable<ManipulationAbortedStatus>, EpsilonComparable<ManipulationAbortedStatus>
{
   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header header_;

   public ManipulationAbortedStatus()
   {
      header_ = new std_msgs.msg.dds.Header();
   }

   public ManipulationAbortedStatus(ManipulationAbortedStatus other)
   {
      this();
      set(other);
   }

   public void set(ManipulationAbortedStatus other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
   }

   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }

   @Override
   public boolean epsilonEquals(ManipulationAbortedStatus other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon))
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
      if (!(other instanceof ManipulationAbortedStatus))
         return false;

      ManipulationAbortedStatus otherMyClass = (ManipulationAbortedStatus) other;

      if (!this.header_.equals(otherMyClass.header_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ManipulationAbortedStatus {");
      builder.append("header=");
      builder.append(this.header_);
      builder.append("}");
      return builder.toString();
   }
}
