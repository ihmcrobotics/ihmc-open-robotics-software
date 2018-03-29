package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

/**
 * This message is part of the IHMC whole-body controller API. The controller will send this message
 * when detecting a fall.
 */
public class WalkingControllerFailureStatusMessage extends Packet<WalkingControllerFailureStatusMessage>
      implements Settable<WalkingControllerFailureStatusMessage>, EpsilonComparable<WalkingControllerFailureStatusMessage>
{
   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header header_;
   /**
    * Specifies the estimated falling direction in 2D
    */
   public us.ihmc.euclid.tuple3D.Vector3D falling_direction_;

   public WalkingControllerFailureStatusMessage()
   {
      header_ = new std_msgs.msg.dds.Header();
      falling_direction_ = new us.ihmc.euclid.tuple3D.Vector3D();
   }

   public WalkingControllerFailureStatusMessage(WalkingControllerFailureStatusMessage other)
   {
      this();
      set(other);
   }

   public void set(WalkingControllerFailureStatusMessage other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.falling_direction_, falling_direction_);
   }

   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }

   /**
    * Specifies the estimated falling direction in 2D
    */
   public us.ihmc.euclid.tuple3D.Vector3D getFallingDirection()
   {
      return falling_direction_;
   }

   @Override
   public boolean epsilonEquals(WalkingControllerFailureStatusMessage other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon))
         return false;
      if (!this.falling_direction_.epsilonEquals(other.falling_direction_, epsilon))
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
      if (!(other instanceof WalkingControllerFailureStatusMessage))
         return false;

      WalkingControllerFailureStatusMessage otherMyClass = (WalkingControllerFailureStatusMessage) other;

      if (!this.header_.equals(otherMyClass.header_))
         return false;
      if (!this.falling_direction_.equals(otherMyClass.falling_direction_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("WalkingControllerFailureStatusMessage {");
      builder.append("header=");
      builder.append(this.header_);
      builder.append(", ");
      builder.append("falling_direction=");
      builder.append(this.falling_direction_);
      builder.append("}");
      return builder.toString();
   }
}
