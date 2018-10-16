package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class KinematicsPlanningToolboxOutputStatus extends Packet<KinematicsPlanningToolboxOutputStatus> implements Settable<KinematicsPlanningToolboxOutputStatus>, EpsilonComparable<KinematicsPlanningToolboxOutputStatus>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   public us.ihmc.idl.IDLSequence.Double  trajectory_times_;
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.KinematicsToolboxOutputStatus>  robot_configurations_;

   public KinematicsPlanningToolboxOutputStatus()
   {
      trajectory_times_ = new us.ihmc.idl.IDLSequence.Double (100, "type_6");

      robot_configurations_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.KinematicsToolboxOutputStatus> (100, new controller_msgs.msg.dds.KinematicsToolboxOutputStatusPubSubType());

   }

   public KinematicsPlanningToolboxOutputStatus(KinematicsPlanningToolboxOutputStatus other)
   {
      this();
      set(other);
   }

   public void set(KinematicsPlanningToolboxOutputStatus other)
   {
      sequence_id_ = other.sequence_id_;

      trajectory_times_.set(other.trajectory_times_);
      robot_configurations_.set(other.robot_configurations_);
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


   public us.ihmc.idl.IDLSequence.Double  getTrajectoryTimes()
   {
      return trajectory_times_;
   }


   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.KinematicsToolboxOutputStatus>  getRobotConfigurations()
   {
      return robot_configurations_;
   }


   public static Supplier<KinematicsPlanningToolboxOutputStatusPubSubType> getPubSubType()
   {
      return KinematicsPlanningToolboxOutputStatusPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return KinematicsPlanningToolboxOutputStatusPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(KinematicsPlanningToolboxOutputStatus other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsDoubleSequence(this.trajectory_times_, other.trajectory_times_, epsilon)) return false;

      if (this.robot_configurations_.size() != other.robot_configurations_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.robot_configurations_.size(); i++)
         {  if (!this.robot_configurations_.get(i).epsilonEquals(other.robot_configurations_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof KinematicsPlanningToolboxOutputStatus)) return false;

      KinematicsPlanningToolboxOutputStatus otherMyClass = (KinematicsPlanningToolboxOutputStatus) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if (!this.trajectory_times_.equals(otherMyClass.trajectory_times_)) return false;
      if (!this.robot_configurations_.equals(otherMyClass.robot_configurations_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("KinematicsPlanningToolboxOutputStatus {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("trajectory_times=");
      builder.append(this.trajectory_times_);      builder.append(", ");
      builder.append("robot_configurations=");
      builder.append(this.robot_configurations_);
      builder.append("}");
      return builder.toString();
   }
}
