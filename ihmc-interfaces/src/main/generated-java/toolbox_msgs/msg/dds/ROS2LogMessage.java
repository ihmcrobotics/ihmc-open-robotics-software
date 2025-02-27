package toolbox_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is used to start and stop recording ROS logs.
       * This is used both for recording and replaying, see ROS2LogRecord and ROS2LogReplay for more information.
       */
public class ROS2LogMessage extends Packet<ROS2LogMessage> implements Settable<ROS2LogMessage>, EpsilonComparable<ROS2LogMessage>
{
   public static final byte ROS2_LOG_REQUESTED_STATE_START = (byte) 0;
   public static final byte ROS2_LOG_REQUESTED_STATE_FINISH = (byte) 1;
   /**
            * Requested state for the ROS 2 log recorder
            */
   public byte requested_state_ = (byte) 255;

   public ROS2LogMessage()
   {
   }

   public ROS2LogMessage(ROS2LogMessage other)
   {
      this();
      set(other);
   }

   public void set(ROS2LogMessage other)
   {
      requested_state_ = other.requested_state_;

   }

   /**
            * Requested state for the ROS 2 log recorder
            */
   public void setRequestedState(byte requested_state)
   {
      requested_state_ = requested_state;
   }
   /**
            * Requested state for the ROS 2 log recorder
            */
   public byte getRequestedState()
   {
      return requested_state_;
   }


   public static Supplier<ROS2LogMessagePubSubType> getPubSubType()
   {
      return ROS2LogMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ROS2LogMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ROS2LogMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.requested_state_, other.requested_state_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ROS2LogMessage)) return false;

      ROS2LogMessage otherMyClass = (ROS2LogMessage) other;

      if(this.requested_state_ != otherMyClass.requested_state_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ROS2LogMessage {");
      builder.append("requested_state=");
      builder.append(this.requested_state_);
      builder.append("}");
      return builder.toString();
   }
}
