package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * This class is used to build a wrench (force & moment) profile over time.
       * It holds the necessary information for one trajectory point.
       */
public class WrenchTrajectoryPointMessage extends Packet<WrenchTrajectoryPointMessage> implements Settable<WrenchTrajectoryPointMessage>, EpsilonComparable<WrenchTrajectoryPointMessage>
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
            * Define the desired wrench (force & moment) to be achieved at this trajectory point.
            */
   public geometry_msgs.msg.dds.Wrench wrench_;

   public WrenchTrajectoryPointMessage()
   {



      wrench_ = new geometry_msgs.msg.dds.Wrench();

   }

   public WrenchTrajectoryPointMessage(WrenchTrajectoryPointMessage other)
   {
      this();
      set(other);
   }

   public void set(WrenchTrajectoryPointMessage other)
   {

      sequence_id_ = other.sequence_id_;


      time_ = other.time_;


      geometry_msgs.msg.dds.WrenchPubSubType.staticCopy(other.wrench_, wrench_);
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
            * Define the desired wrench (force & moment) to be achieved at this trajectory point.
            */
   public geometry_msgs.msg.dds.Wrench getWrench()
   {
      return wrench_;
   }


   public static Supplier<WrenchTrajectoryPointMessagePubSubType> getPubSubType()
   {
      return WrenchTrajectoryPointMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return WrenchTrajectoryPointMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(WrenchTrajectoryPointMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.time_, other.time_, epsilon)) return false;


      if (!this.wrench_.epsilonEquals(other.wrench_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof WrenchTrajectoryPointMessage)) return false;

      WrenchTrajectoryPointMessage otherMyClass = (WrenchTrajectoryPointMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.time_ != otherMyClass.time_) return false;


      if (!this.wrench_.equals(otherMyClass.wrench_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("WrenchTrajectoryPointMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("time=");
      builder.append(this.time_);      builder.append(", ");

      builder.append("wrench=");
      builder.append(this.wrench_);
      builder.append("}");
      return builder.toString();
   }
}
