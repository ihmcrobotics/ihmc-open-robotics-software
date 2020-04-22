package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * This message will request a calibration the wrist force sensors to the IHMC controller (does not do hardware calibration).
       * It is strongly suggested to perform the calibration when the hands are not moving nor interacting with the environment.
       */
public class RequestWristForceSensorCalibrationPacket extends Packet<RequestWristForceSensorCalibrationPacket> implements Settable<RequestWristForceSensorCalibrationPacket>, EpsilonComparable<RequestWristForceSensorCalibrationPacket>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public RequestWristForceSensorCalibrationPacket()
   {


   }

   public RequestWristForceSensorCalibrationPacket(RequestWristForceSensorCalibrationPacket other)
   {
      this();
      set(other);
   }

   public void set(RequestWristForceSensorCalibrationPacket other)
   {

      sequence_id_ = other.sequence_id_;

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


   public static Supplier<RequestWristForceSensorCalibrationPacketPubSubType> getPubSubType()
   {
      return RequestWristForceSensorCalibrationPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return RequestWristForceSensorCalibrationPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(RequestWristForceSensorCalibrationPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof RequestWristForceSensorCalibrationPacket)) return false;

      RequestWristForceSensorCalibrationPacket otherMyClass = (RequestWristForceSensorCalibrationPacket) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("RequestWristForceSensorCalibrationPacket {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);
      builder.append("}");
      return builder.toString();
   }
}
