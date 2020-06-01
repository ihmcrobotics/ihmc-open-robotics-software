package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * This class is used to build trajectory messages in jointspace
       * It holds all the trajectory points to go through with a one-dimensional trajectory.
       * A third order polynomial function is used to interpolate between trajectory points.
       */
public class OneDoFJointTrajectoryMessage extends Packet<OneDoFJointTrajectoryMessage> implements Settable<OneDoFJointTrajectoryMessage>, EpsilonComparable<OneDoFJointTrajectoryMessage>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   /**
            * The list of trajectory points to go through while executing the trajectory.
            * The time has to be strictly increasing.
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.TrajectoryPoint1DMessage>  trajectory_points_;

   /**
            * Weight used to encode the priority for achieving this trajectory:
            * - if too low, in the event the controller can't achieve all of the objectives it may lower the trajectory tracking quality.
            * - if too high, the controller will favor this trajectory over other objectives.
            * - if set to NaN or to a negative value, the controller will use the default weight for that trajectory.
            * The priority of this trajectory is determined from the relative weight of this trajectory and the weight of the other objectives.
            */
   public double weight_ = -1.0;

   public OneDoFJointTrajectoryMessage()
   {


      trajectory_points_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.TrajectoryPoint1DMessage> (50, new controller_msgs.msg.dds.TrajectoryPoint1DMessagePubSubType());


   }

   public OneDoFJointTrajectoryMessage(OneDoFJointTrajectoryMessage other)
   {
      this();
      set(other);
   }

   public void set(OneDoFJointTrajectoryMessage other)
   {

      sequence_id_ = other.sequence_id_;


      trajectory_points_.set(other.trajectory_points_);

      weight_ = other.weight_;

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
            * The list of trajectory points to go through while executing the trajectory.
            * The time has to be strictly increasing.
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.TrajectoryPoint1DMessage>  getTrajectoryPoints()
   {
      return trajectory_points_;
   }


   /**
            * Weight used to encode the priority for achieving this trajectory:
            * - if too low, in the event the controller can't achieve all of the objectives it may lower the trajectory tracking quality.
            * - if too high, the controller will favor this trajectory over other objectives.
            * - if set to NaN or to a negative value, the controller will use the default weight for that trajectory.
            * The priority of this trajectory is determined from the relative weight of this trajectory and the weight of the other objectives.
            */
   public void setWeight(double weight)
   {
      weight_ = weight;
   }
   /**
            * Weight used to encode the priority for achieving this trajectory:
            * - if too low, in the event the controller can't achieve all of the objectives it may lower the trajectory tracking quality.
            * - if too high, the controller will favor this trajectory over other objectives.
            * - if set to NaN or to a negative value, the controller will use the default weight for that trajectory.
            * The priority of this trajectory is determined from the relative weight of this trajectory and the weight of the other objectives.
            */
   public double getWeight()
   {
      return weight_;
   }


   public static Supplier<OneDoFJointTrajectoryMessagePubSubType> getPubSubType()
   {
      return OneDoFJointTrajectoryMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return OneDoFJointTrajectoryMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(OneDoFJointTrajectoryMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (this.trajectory_points_.size() != other.trajectory_points_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.trajectory_points_.size(); i++)
         {  if (!this.trajectory_points_.get(i).epsilonEquals(other.trajectory_points_.get(i), epsilon)) return false; }
      }


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.weight_, other.weight_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof OneDoFJointTrajectoryMessage)) return false;

      OneDoFJointTrajectoryMessage otherMyClass = (OneDoFJointTrajectoryMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if (!this.trajectory_points_.equals(otherMyClass.trajectory_points_)) return false;

      if(this.weight_ != otherMyClass.weight_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("OneDoFJointTrajectoryMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("trajectory_points=");
      builder.append(this.trajectory_points_);      builder.append(", ");

      builder.append("weight=");
      builder.append(this.weight_);
      builder.append("}");
      return builder.toString();
   }
}
