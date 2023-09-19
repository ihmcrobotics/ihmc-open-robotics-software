package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This is a message that holds data to describe the state and control trajectories.
       * 
       * The trajectory is defined by:
       * * the time interval,
       * * the state, and
       * * the control for each node.
       * 
       * The dimension of the intervals, state and control trajectories must be equal.
       * The header specifies the time at which the message is published.
       */
public class CrocoddylSolverTrajectoryMessage extends Packet<CrocoddylSolverTrajectoryMessage> implements Settable<CrocoddylSolverTrajectoryMessage>, EpsilonComparable<CrocoddylSolverTrajectoryMessage>
{
   /**
            * This represents the time and frame at which the message is published
            */
   public std_msgs.msg.dds.Header header_;
   /**
            * This represents the time and duration of each interval
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.CrocoddylTimeIntervalMessage>  intervals_;
   /**
            * This represents the state for each interval
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.CrocoddylStateMessage>  state_trajectory_;
   /**
            * This represents the control for each interval
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.CrocoddylControlMessage>  control_trajectory_;

   public CrocoddylSolverTrajectoryMessage()
   {
      header_ = new std_msgs.msg.dds.Header();
      intervals_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.CrocoddylTimeIntervalMessage> (100, new controller_msgs.msg.dds.CrocoddylTimeIntervalMessagePubSubType());
      state_trajectory_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.CrocoddylStateMessage> (100, new controller_msgs.msg.dds.CrocoddylStateMessagePubSubType());
      control_trajectory_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.CrocoddylControlMessage> (100, new controller_msgs.msg.dds.CrocoddylControlMessagePubSubType());

   }

   public CrocoddylSolverTrajectoryMessage(CrocoddylSolverTrajectoryMessage other)
   {
      this();
      set(other);
   }

   public void set(CrocoddylSolverTrajectoryMessage other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      intervals_.set(other.intervals_);
      state_trajectory_.set(other.state_trajectory_);
      control_trajectory_.set(other.control_trajectory_);
   }


   /**
            * This represents the time and frame at which the message is published
            */
   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }


   /**
            * This represents the time and duration of each interval
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.CrocoddylTimeIntervalMessage>  getIntervals()
   {
      return intervals_;
   }


   /**
            * This represents the state for each interval
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.CrocoddylStateMessage>  getStateTrajectory()
   {
      return state_trajectory_;
   }


   /**
            * This represents the control for each interval
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.CrocoddylControlMessage>  getControlTrajectory()
   {
      return control_trajectory_;
   }


   public static Supplier<CrocoddylSolverTrajectoryMessagePubSubType> getPubSubType()
   {
      return CrocoddylSolverTrajectoryMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return CrocoddylSolverTrajectoryMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(CrocoddylSolverTrajectoryMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon)) return false;
      if (this.intervals_.size() != other.intervals_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.intervals_.size(); i++)
         {  if (!this.intervals_.get(i).epsilonEquals(other.intervals_.get(i), epsilon)) return false; }
      }

      if (this.state_trajectory_.size() != other.state_trajectory_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.state_trajectory_.size(); i++)
         {  if (!this.state_trajectory_.get(i).epsilonEquals(other.state_trajectory_.get(i), epsilon)) return false; }
      }

      if (this.control_trajectory_.size() != other.control_trajectory_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.control_trajectory_.size(); i++)
         {  if (!this.control_trajectory_.get(i).epsilonEquals(other.control_trajectory_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof CrocoddylSolverTrajectoryMessage)) return false;

      CrocoddylSolverTrajectoryMessage otherMyClass = (CrocoddylSolverTrajectoryMessage) other;

      if (!this.header_.equals(otherMyClass.header_)) return false;
      if (!this.intervals_.equals(otherMyClass.intervals_)) return false;
      if (!this.state_trajectory_.equals(otherMyClass.state_trajectory_)) return false;
      if (!this.control_trajectory_.equals(otherMyClass.control_trajectory_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("CrocoddylSolverTrajectoryMessage {");
      builder.append("header=");
      builder.append(this.header_);      builder.append(", ");
      builder.append("intervals=");
      builder.append(this.intervals_);      builder.append(", ");
      builder.append("state_trajectory=");
      builder.append(this.state_trajectory_);      builder.append(", ");
      builder.append("control_trajectory=");
      builder.append(this.control_trajectory_);
      builder.append("}");
      return builder.toString();
   }
}
