package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * This message commands the controller to move in taskspace a foot to the desired position  while going through the specified trajectory points.
       * To execute a single straight line trajectory to reach a desired foot pose, set only one trajectory point with zero velocity and its time to be equal to the desired trajectory time.
       */
public class SoleTrajectoryMessage extends Packet<SoleTrajectoryMessage> implements Settable<SoleTrajectoryMessage>, EpsilonComparable<SoleTrajectoryMessage>
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
            * Specifies which foot will execute the trajectory.
            */
   public byte robot_quadrant_ = (byte) 255;

   /**
            * The position trajectory information.
            */
   public controller_msgs.msg.dds.EuclideanTrajectoryMessage position_trajectory_;

   public SoleTrajectoryMessage()
   {



      position_trajectory_ = new controller_msgs.msg.dds.EuclideanTrajectoryMessage();

   }

   public SoleTrajectoryMessage(SoleTrajectoryMessage other)
   {
      this();
      set(other);
   }

   public void set(SoleTrajectoryMessage other)
   {

      sequence_id_ = other.sequence_id_;


      robot_quadrant_ = other.robot_quadrant_;


      controller_msgs.msg.dds.EuclideanTrajectoryMessagePubSubType.staticCopy(other.position_trajectory_, position_trajectory_);
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
            * Specifies which foot will execute the trajectory.
            */
   public void setRobotQuadrant(byte robot_quadrant)
   {
      robot_quadrant_ = robot_quadrant;
   }
   /**
            * Specifies which foot will execute the trajectory.
            */
   public byte getRobotQuadrant()
   {
      return robot_quadrant_;
   }



   /**
            * The position trajectory information.
            */
   public controller_msgs.msg.dds.EuclideanTrajectoryMessage getPositionTrajectory()
   {
      return position_trajectory_;
   }


   public static Supplier<SoleTrajectoryMessagePubSubType> getPubSubType()
   {
      return SoleTrajectoryMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return SoleTrajectoryMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(SoleTrajectoryMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_quadrant_, other.robot_quadrant_, epsilon)) return false;


      if (!this.position_trajectory_.epsilonEquals(other.position_trajectory_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof SoleTrajectoryMessage)) return false;

      SoleTrajectoryMessage otherMyClass = (SoleTrajectoryMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.robot_quadrant_ != otherMyClass.robot_quadrant_) return false;


      if (!this.position_trajectory_.equals(otherMyClass.position_trajectory_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("SoleTrajectoryMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("robot_quadrant=");
      builder.append(this.robot_quadrant_);      builder.append(", ");

      builder.append("position_trajectory=");
      builder.append(this.position_trajectory_);
      builder.append("}");
      return builder.toString();
   }
}
