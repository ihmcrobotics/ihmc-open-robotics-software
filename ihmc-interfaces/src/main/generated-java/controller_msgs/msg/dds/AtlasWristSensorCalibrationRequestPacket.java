package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

/**
 * Atlas specific message: request taring of the wrist force/torque sensors.
 */
public class AtlasWristSensorCalibrationRequestPacket extends Packet<AtlasWristSensorCalibrationRequestPacket>
      implements Settable<AtlasWristSensorCalibrationRequestPacket>, EpsilonComparable<AtlasWristSensorCalibrationRequestPacket>
{
   public static final byte ROBOT_SIDE_LEFT = (byte) 0;
   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;
   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header header_;
   /**
    * The robot side (left or right) for the wrist sensor you would like to request calibration for.
    */
   public byte robot_side_ = (byte) 255;

   public AtlasWristSensorCalibrationRequestPacket()
   {
      header_ = new std_msgs.msg.dds.Header();
   }

   public AtlasWristSensorCalibrationRequestPacket(AtlasWristSensorCalibrationRequestPacket other)
   {
      this();
      set(other);
   }

   public void set(AtlasWristSensorCalibrationRequestPacket other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      robot_side_ = other.robot_side_;

   }

   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }

   /**
    * The robot side (left or right) for the wrist sensor you would like to request calibration for.
    */
   public void setRobotSide(byte robot_side)
   {
      robot_side_ = robot_side;
   }

   /**
    * The robot side (left or right) for the wrist sensor you would like to request calibration for.
    */
   public byte getRobotSide()
   {
      return robot_side_;
   }

   @Override
   public boolean epsilonEquals(AtlasWristSensorCalibrationRequestPacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon))
         return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon))
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
      if (!(other instanceof AtlasWristSensorCalibrationRequestPacket))
         return false;

      AtlasWristSensorCalibrationRequestPacket otherMyClass = (AtlasWristSensorCalibrationRequestPacket) other;

      if (!this.header_.equals(otherMyClass.header_))
         return false;
      if (this.robot_side_ != otherMyClass.robot_side_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AtlasWristSensorCalibrationRequestPacket {");
      builder.append("header=");
      builder.append(this.header_);
      builder.append(", ");
      builder.append("robot_side=");
      builder.append(this.robot_side_);
      builder.append("}");
      return builder.toString();
   }
}
