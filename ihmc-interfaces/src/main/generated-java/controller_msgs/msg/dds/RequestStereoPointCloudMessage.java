package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

/**
 * Use this message to request a new point cloud from the stereo camera.
 */
public class RequestStereoPointCloudMessage extends Packet<RequestStereoPointCloudMessage>
      implements Settable<RequestStereoPointCloudMessage>, EpsilonComparable<RequestStereoPointCloudMessage>
{
   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header header_;

   public RequestStereoPointCloudMessage()
   {
      header_ = new std_msgs.msg.dds.Header();
   }

   public RequestStereoPointCloudMessage(RequestStereoPointCloudMessage other)
   {
      this();
      set(other);
   }

   public void set(RequestStereoPointCloudMessage other)
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
   public boolean epsilonEquals(RequestStereoPointCloudMessage other, double epsilon)
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
      if (!(other instanceof RequestStereoPointCloudMessage))
         return false;

      RequestStereoPointCloudMessage otherMyClass = (RequestStereoPointCloudMessage) other;

      if (!this.header_.equals(otherMyClass.header_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("RequestStereoPointCloudMessage {");
      builder.append("header=");
      builder.append(this.header_);
      builder.append("}");
      return builder.toString();
   }
}
