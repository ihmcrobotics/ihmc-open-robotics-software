package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC quadruped controller API.
       * This message commands the controller to start loading a foot.
       * This message will only load the foot when the robot is in Sole Waypoint mode, which is triggered by a SoleTrajectoryMessage
       * If the robot is walking, the message is ignored
       */
public class QuadrupedFootLoadBearingMessage extends Packet<QuadrupedFootLoadBearingMessage> implements Settable<QuadrupedFootLoadBearingMessage>, EpsilonComparable<QuadrupedFootLoadBearingMessage>
{
   public static final byte FRONT_LEFT = (byte) 0;
   public static final byte FRONT_RIGHT = (byte) 1;
   public static final byte HIND_RIGHT = (byte) 2;
   public static final byte HIND_LEFT = (byte) 3;
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Specifies which quadrant should be loaded.
            */
   public byte robot_quadrant_ = (byte) 255;
   /**
            * The time to delay this command on the controller side before being executed.
            */
   public double execution_delay_time_;

   public QuadrupedFootLoadBearingMessage()
   {
   }

   public QuadrupedFootLoadBearingMessage(QuadrupedFootLoadBearingMessage other)
   {
      this();
      set(other);
   }

   public void set(QuadrupedFootLoadBearingMessage other)
   {
      sequence_id_ = other.sequence_id_;

      robot_quadrant_ = other.robot_quadrant_;

      execution_delay_time_ = other.execution_delay_time_;

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
            * Specifies which quadrant should be loaded.
            */
   public void setRobotQuadrant(byte robot_quadrant)
   {
      robot_quadrant_ = robot_quadrant;
   }
   /**
            * Specifies which quadrant should be loaded.
            */
   public byte getRobotQuadrant()
   {
      return robot_quadrant_;
   }

   /**
            * The time to delay this command on the controller side before being executed.
            */
   public void setExecutionDelayTime(double execution_delay_time)
   {
      execution_delay_time_ = execution_delay_time;
   }
   /**
            * The time to delay this command on the controller side before being executed.
            */
   public double getExecutionDelayTime()
   {
      return execution_delay_time_;
   }


   public static Supplier<QuadrupedFootLoadBearingMessagePubSubType> getPubSubType()
   {
      return QuadrupedFootLoadBearingMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return QuadrupedFootLoadBearingMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(QuadrupedFootLoadBearingMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_quadrant_, other.robot_quadrant_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.execution_delay_time_, other.execution_delay_time_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof QuadrupedFootLoadBearingMessage)) return false;

      QuadrupedFootLoadBearingMessage otherMyClass = (QuadrupedFootLoadBearingMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.robot_quadrant_ != otherMyClass.robot_quadrant_) return false;

      if(this.execution_delay_time_ != otherMyClass.execution_delay_time_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("QuadrupedFootLoadBearingMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("robot_quadrant=");
      builder.append(this.robot_quadrant_);      builder.append(", ");
      builder.append("execution_delay_time=");
      builder.append(this.execution_delay_time_);
      builder.append("}");
      return builder.toString();
   }
}
