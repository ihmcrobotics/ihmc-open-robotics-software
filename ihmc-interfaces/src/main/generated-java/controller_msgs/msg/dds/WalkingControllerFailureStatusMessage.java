package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * The controller will send this message when detecting a fall.
       */
public class WalkingControllerFailureStatusMessage extends Packet<WalkingControllerFailureStatusMessage> implements Settable<WalkingControllerFailureStatusMessage>, EpsilonComparable<WalkingControllerFailureStatusMessage>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   /**
            * Specifies the estimated falling direction in 2D
            */
   public us.ihmc.euclid.tuple3D.Vector3D falling_direction_;

   public WalkingControllerFailureStatusMessage()
   {


      falling_direction_ = new us.ihmc.euclid.tuple3D.Vector3D();

   }

   public WalkingControllerFailureStatusMessage(WalkingControllerFailureStatusMessage other)
   {
      this();
      set(other);
   }

   public void set(WalkingControllerFailureStatusMessage other)
   {

      sequence_id_ = other.sequence_id_;


      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.falling_direction_, falling_direction_);
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
            * Specifies the estimated falling direction in 2D
            */
   public us.ihmc.euclid.tuple3D.Vector3D getFallingDirection()
   {
      return falling_direction_;
   }


   public static Supplier<WalkingControllerFailureStatusMessagePubSubType> getPubSubType()
   {
      return WalkingControllerFailureStatusMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return WalkingControllerFailureStatusMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(WalkingControllerFailureStatusMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!this.falling_direction_.epsilonEquals(other.falling_direction_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof WalkingControllerFailureStatusMessage)) return false;

      WalkingControllerFailureStatusMessage otherMyClass = (WalkingControllerFailureStatusMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if (!this.falling_direction_.equals(otherMyClass.falling_direction_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("WalkingControllerFailureStatusMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("falling_direction=");
      builder.append(this.falling_direction_);
      builder.append("}");
      return builder.toString();
   }
}
