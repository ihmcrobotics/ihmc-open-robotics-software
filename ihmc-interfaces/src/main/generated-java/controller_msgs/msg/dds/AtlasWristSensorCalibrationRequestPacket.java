package controller_msgs.msg.dds;

import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * Atlas specific message: request taring of the wrist force/torque sensors.
 */
public class AtlasWristSensorCalibrationRequestPacket
      implements Settable<AtlasWristSensorCalibrationRequestPacket>, EpsilonComparable<AtlasWristSensorCalibrationRequestPacket>
{
   public static final byte ROBOT_SIDE_LEFT = (byte) 0;
   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;
   /**
    * The robot side (left or right) for the wrist sensor you would like to request calibration for.
    */
   private byte robot_side_ = (byte) 255;

   public AtlasWristSensorCalibrationRequestPacket()
   {
   }

   public AtlasWristSensorCalibrationRequestPacket(AtlasWristSensorCalibrationRequestPacket other)
   {
      set(other);
   }

   public void set(AtlasWristSensorCalibrationRequestPacket other)
   {
      robot_side_ = other.robot_side_;
   }

   /**
    * The robot side (left or right) for the wrist sensor you would like to request calibration for.
    */
   public byte getRobotSide()
   {
      return robot_side_;
   }

   /**
    * The robot side (left or right) for the wrist sensor you would like to request calibration for.
    */
   public void setRobotSide(byte robot_side)
   {
      robot_side_ = robot_side;
   }

   @Override
   public boolean epsilonEquals(AtlasWristSensorCalibrationRequestPacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

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

      if (this.robot_side_ != otherMyClass.robot_side_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AtlasWristSensorCalibrationRequestPacket {");
      builder.append("robot_side=");
      builder.append(this.robot_side_);

      builder.append("}");
      return builder.toString();
   }
}