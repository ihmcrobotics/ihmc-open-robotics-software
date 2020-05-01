package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Atlas specific message: request taring of the wrist force/torque sensors.
       */
public class AtlasWristSensorCalibrationRequestPacket extends Packet<AtlasWristSensorCalibrationRequestPacket> implements Settable<AtlasWristSensorCalibrationRequestPacket>, EpsilonComparable<AtlasWristSensorCalibrationRequestPacket>
{

   public static final byte ROBOT_SIDE_LEFT = (byte) 0;

   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   /**
            * The robot side (left or right) for the wrist sensor you would like to request calibration for.
            */
   public byte robot_side_ = (byte) 255;

   public AtlasWristSensorCalibrationRequestPacket()
   {



   }

   public AtlasWristSensorCalibrationRequestPacket(AtlasWristSensorCalibrationRequestPacket other)
   {
      this();
      set(other);
   }

   public void set(AtlasWristSensorCalibrationRequestPacket other)
   {

      sequence_id_ = other.sequence_id_;


      robot_side_ = other.robot_side_;

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


   public static Supplier<AtlasWristSensorCalibrationRequestPacketPubSubType> getPubSubType()
   {
      return AtlasWristSensorCalibrationRequestPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return AtlasWristSensorCalibrationRequestPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(AtlasWristSensorCalibrationRequestPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof AtlasWristSensorCalibrationRequestPacket)) return false;

      AtlasWristSensorCalibrationRequestPacket otherMyClass = (AtlasWristSensorCalibrationRequestPacket) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.robot_side_ != otherMyClass.robot_side_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AtlasWristSensorCalibrationRequestPacket {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("robot_side=");
      builder.append(this.robot_side_);
      builder.append("}");
      return builder.toString();
   }
}
