package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

/**
 * This message is part of the IHMC whole-body controller API. This message will request a
 * calibration the wrist force sensors to the IHMC controller (does not do hardware calibration). It
 * is strongly suggested to perform the calibration when the hands are not moving nor interacting
 * with the environment.
 */
public class RequestWristForceSensorCalibrationPacket extends Packet<RequestWristForceSensorCalibrationPacket>
      implements Settable<RequestWristForceSensorCalibrationPacket>, EpsilonComparable<RequestWristForceSensorCalibrationPacket>
{
   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header header_;

   public RequestWristForceSensorCalibrationPacket()
   {
      header_ = new std_msgs.msg.dds.Header();
   }

   public RequestWristForceSensorCalibrationPacket(RequestWristForceSensorCalibrationPacket other)
   {
      this();
      set(other);
   }

   public void set(RequestWristForceSensorCalibrationPacket other)
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
   public boolean epsilonEquals(RequestWristForceSensorCalibrationPacket other, double epsilon)
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
      if (!(other instanceof RequestWristForceSensorCalibrationPacket))
         return false;

      RequestWristForceSensorCalibrationPacket otherMyClass = (RequestWristForceSensorCalibrationPacket) other;

      if (!this.header_.equals(otherMyClass.header_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("RequestWristForceSensorCalibrationPacket {");
      builder.append("header=");
      builder.append(this.header_);
      builder.append("}");
      return builder.toString();
   }
}
