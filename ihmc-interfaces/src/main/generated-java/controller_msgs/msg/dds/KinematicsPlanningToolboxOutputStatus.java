package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class KinematicsPlanningToolboxOutputStatus extends Packet<KinematicsPlanningToolboxOutputStatus> implements Settable<KinematicsPlanningToolboxOutputStatus>, EpsilonComparable<KinematicsPlanningToolboxOutputStatus>
{
   public static final byte KINEMATICS_PLANNING_RESULT_OPTIMAL_SOLUTION = (byte) 0;
   public static final byte KINEMATICS_PLANNING_RESULT_EXCEED_JOINT_VELOCITY_LIMIT = (byte) 1;
   public static final byte KINEMATICS_PLANNING_RESULT_UNREACHABLE_KEYFRAME = (byte) 2;
   /**
            * This message is part of the IHMC whole-body inverse kinematics module.
            * This output status will be converted into the WholeBodyTrajectoryMessage.
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   public int plan_id_ = -1;
   /**
            * List of times for each key frames.
            * The length of this should be same with the length of the configurations.
            */
   public us.ihmc.idl.IDLSequence.Double  key_frame_times_;
   /**
            * List of configurations for each key frames.
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.KinematicsToolboxOutputStatus>  robot_configurations_;
   /**
            * Solution quality.
            * The total summation of the all solution quality for each key frames.
            */
   public double solution_quality_ = -1.0;
   /**
            * Suggested message to send to the IHMC walking controller to request the robot to go through the key frames.
            */
   public controller_msgs.msg.dds.WholeBodyTrajectoryMessage suggested_controller_message_;

   public KinematicsPlanningToolboxOutputStatus()
   {
      key_frame_times_ = new us.ihmc.idl.IDLSequence.Double (100, "type_6");

      robot_configurations_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.KinematicsToolboxOutputStatus> (100, new controller_msgs.msg.dds.KinematicsToolboxOutputStatusPubSubType());
      suggested_controller_message_ = new controller_msgs.msg.dds.WholeBodyTrajectoryMessage();

   }

   public KinematicsPlanningToolboxOutputStatus(KinematicsPlanningToolboxOutputStatus other)
   {
      this();
      set(other);
   }

   public void set(KinematicsPlanningToolboxOutputStatus other)
   {
      sequence_id_ = other.sequence_id_;

      plan_id_ = other.plan_id_;

      key_frame_times_.set(other.key_frame_times_);
      robot_configurations_.set(other.robot_configurations_);
      solution_quality_ = other.solution_quality_;

      controller_msgs.msg.dds.WholeBodyTrajectoryMessagePubSubType.staticCopy(other.suggested_controller_message_, suggested_controller_message_);
   }

   /**
            * This message is part of the IHMC whole-body inverse kinematics module.
            * This output status will be converted into the WholeBodyTrajectoryMessage.
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public void setSequenceId(long sequence_id)
   {
      sequence_id_ = sequence_id;
   }
   /**
            * This message is part of the IHMC whole-body inverse kinematics module.
            * This output status will be converted into the WholeBodyTrajectoryMessage.
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long getSequenceId()
   {
      return sequence_id_;
   }

   public void setPlanId(int plan_id)
   {
      plan_id_ = plan_id;
   }
   public int getPlanId()
   {
      return plan_id_;
   }


   /**
            * List of times for each key frames.
            * The length of this should be same with the length of the configurations.
            */
   public us.ihmc.idl.IDLSequence.Double  getKeyFrameTimes()
   {
      return key_frame_times_;
   }


   /**
            * List of configurations for each key frames.
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.KinematicsToolboxOutputStatus>  getRobotConfigurations()
   {
      return robot_configurations_;
   }

   /**
            * Solution quality.
            * The total summation of the all solution quality for each key frames.
            */
   public void setSolutionQuality(double solution_quality)
   {
      solution_quality_ = solution_quality;
   }
   /**
            * Solution quality.
            * The total summation of the all solution quality for each key frames.
            */
   public double getSolutionQuality()
   {
      return solution_quality_;
   }


   /**
            * Suggested message to send to the IHMC walking controller to request the robot to go through the key frames.
            */
   public controller_msgs.msg.dds.WholeBodyTrajectoryMessage getSuggestedControllerMessage()
   {
      return suggested_controller_message_;
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

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.plan_id_, other.plan_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsDoubleSequence(this.key_frame_times_, other.key_frame_times_, epsilon)) return false;

      if (this.robot_configurations_.size() != other.robot_configurations_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.robot_configurations_.size(); i++)
         {  if (!this.robot_configurations_.get(i).epsilonEquals(other.robot_configurations_.get(i), epsilon)) return false; }
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.solution_quality_, other.solution_quality_, epsilon)) return false;

      if (!this.suggested_controller_message_.epsilonEquals(other.suggested_controller_message_, epsilon)) return false;

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

      if(this.plan_id_ != otherMyClass.plan_id_) return false;

      if (!this.key_frame_times_.equals(otherMyClass.key_frame_times_)) return false;
      if (!this.robot_configurations_.equals(otherMyClass.robot_configurations_)) return false;
      if(this.solution_quality_ != otherMyClass.solution_quality_) return false;

      if (!this.suggested_controller_message_.equals(otherMyClass.suggested_controller_message_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("KinematicsPlanningToolboxOutputStatus {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("plan_id=");
      builder.append(this.plan_id_);      builder.append(", ");
      builder.append("key_frame_times=");
      builder.append(this.key_frame_times_);      builder.append(", ");
      builder.append("robot_configurations=");
      builder.append(this.robot_configurations_);      builder.append(", ");
      builder.append("solution_quality=");
      builder.append(this.solution_quality_);      builder.append(", ");
      builder.append("suggested_controller_message=");
      builder.append(this.suggested_controller_message_);
      builder.append("}");
      return builder.toString();
   }
}
