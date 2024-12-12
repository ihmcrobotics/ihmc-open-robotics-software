package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * This class is used to change the PID gains over time.
       * It holds the necessary information for one trajectory point.
       */
public class SE3PIDGainsTrajectoryPointMessage extends Packet<SE3PIDGainsTrajectoryPointMessage> implements Settable<SE3PIDGainsTrajectoryPointMessage>, EpsilonComparable<SE3PIDGainsTrajectoryPointMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Time at which the trajectory point has to be reached. The time is relative to when the trajectory starts.
            */
   public double time_;
   /**
            * Define the desired linear PID gains for the trajectory point.
            */
   public controller_msgs.msg.dds.PID3DGains linear_gains_;
   /**
            * Define the desired angular PID gains for the trajectory point.
            */
   public controller_msgs.msg.dds.PID3DGains angular_gains_;

   public SE3PIDGainsTrajectoryPointMessage()
   {
      linear_gains_ = new controller_msgs.msg.dds.PID3DGains();
      angular_gains_ = new controller_msgs.msg.dds.PID3DGains();
   }

   public SE3PIDGainsTrajectoryPointMessage(SE3PIDGainsTrajectoryPointMessage other)
   {
      this();
      set(other);
   }

   public void set(SE3PIDGainsTrajectoryPointMessage other)
   {
      sequence_id_ = other.sequence_id_;

      time_ = other.time_;

      controller_msgs.msg.dds.PID3DGainsPubSubType.staticCopy(other.linear_gains_, linear_gains_);
      controller_msgs.msg.dds.PID3DGainsPubSubType.staticCopy(other.angular_gains_, angular_gains_);
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
            * Time at which the trajectory point has to be reached. The time is relative to when the trajectory starts.
            */
   public void setTime(double time)
   {
      time_ = time;
   }
   /**
            * Time at which the trajectory point has to be reached. The time is relative to when the trajectory starts.
            */
   public double getTime()
   {
      return time_;
   }


   /**
            * Define the desired linear PID gains for the trajectory point.
            */
   public controller_msgs.msg.dds.PID3DGains getLinearGains()
   {
      return linear_gains_;
   }


   /**
            * Define the desired angular PID gains for the trajectory point.
            */
   public controller_msgs.msg.dds.PID3DGains getAngularGains()
   {
      return angular_gains_;
   }


   public static Supplier<SE3PIDGainsTrajectoryPointMessagePubSubType> getPubSubType()
   {
      return SE3PIDGainsTrajectoryPointMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return SE3PIDGainsTrajectoryPointMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(SE3PIDGainsTrajectoryPointMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.time_, other.time_, epsilon)) return false;

      if (!this.linear_gains_.epsilonEquals(other.linear_gains_, epsilon)) return false;
      if (!this.angular_gains_.epsilonEquals(other.angular_gains_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof SE3PIDGainsTrajectoryPointMessage)) return false;

      SE3PIDGainsTrajectoryPointMessage otherMyClass = (SE3PIDGainsTrajectoryPointMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.time_ != otherMyClass.time_) return false;

      if (!this.linear_gains_.equals(otherMyClass.linear_gains_)) return false;
      if (!this.angular_gains_.equals(otherMyClass.angular_gains_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("SE3PIDGainsTrajectoryPointMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("time=");
      builder.append(this.time_);      builder.append(", ");
      builder.append("linear_gains=");
      builder.append(this.linear_gains_);      builder.append(", ");
      builder.append("angular_gains=");
      builder.append(this.angular_gains_);
      builder.append("}");
      return builder.toString();
   }
}
