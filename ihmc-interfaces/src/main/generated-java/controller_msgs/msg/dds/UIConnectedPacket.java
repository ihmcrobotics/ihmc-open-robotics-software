package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

/**
 * TODO: This message is not really used, the feature it provides needs to be fixed or this message
 * needs to be deleted.
 */
public class UIConnectedPacket extends Packet<UIConnectedPacket> implements Settable<UIConnectedPacket>, EpsilonComparable<UIConnectedPacket>
{
   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header header_;

   public UIConnectedPacket()
   {
      header_ = new std_msgs.msg.dds.Header();
   }

   public UIConnectedPacket(UIConnectedPacket other)
   {
      this();
      set(other);
   }

   public void set(UIConnectedPacket other)
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
   public boolean epsilonEquals(UIConnectedPacket other, double epsilon)
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
      if (!(other instanceof UIConnectedPacket))
         return false;

      UIConnectedPacket otherMyClass = (UIConnectedPacket) other;

      if (!this.header_.equals(otherMyClass.header_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("UIConnectedPacket {");
      builder.append("header=");
      builder.append(this.header_);
      builder.append("}");
      return builder.toString();
   }
}
