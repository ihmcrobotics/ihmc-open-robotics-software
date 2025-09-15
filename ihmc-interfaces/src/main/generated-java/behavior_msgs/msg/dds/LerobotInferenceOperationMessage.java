package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * A message for remotely operationg the lerobot policies
       */
public class LerobotInferenceOperationMessage extends Packet<LerobotInferenceOperationMessage> implements Settable<LerobotInferenceOperationMessage>, EpsilonComparable<LerobotInferenceOperationMessage>
{
   /**
            * Allows the user to operate the inference of visuomotor policies
            */
   public ihmc_common_msgs.msg.dds.LatestModificationMessage latest_timestamp_modifiable_;
   /**
            * Monotonically increasing, number of updates of robot thread
            */
   public long sequence_id_;
   /**
            * Whether the inference and action output to IK streaming is active
            */
   public boolean running_;
   /**
            * Whether to enable controlling the robot instead of just previewing
            */
   public boolean control_robot_;
   /**
            * The latest action hand poses
            */
   public us.ihmc.euclid.geometry.Pose3D[] action_hand_poses_;
   /**
            * The latest action forearm poses
            */
   public us.ihmc.euclid.geometry.Pose3D[] action_forearm_poses_;
   /**
            * Frequency of status from python side
            */
   public double python_status_frequency_;
   /**
            * Message received from python
            */
   public java.lang.StringBuilder python_status_message_;
   /**
            * Number of output actions received from policy
            */
   public long received_actions_;

   public LerobotInferenceOperationMessage()
   {
      latest_timestamp_modifiable_ = new ihmc_common_msgs.msg.dds.LatestModificationMessage();
      action_hand_poses_ = new us.ihmc.euclid.geometry.Pose3D[2];

      for(int i1 = 0; i1 < action_hand_poses_.length; ++i1)
      {
          action_hand_poses_[i1] = new us.ihmc.euclid.geometry.Pose3D();
      }
      action_forearm_poses_ = new us.ihmc.euclid.geometry.Pose3D[2];

      for(int i3 = 0; i3 < action_forearm_poses_.length; ++i3)
      {
          action_forearm_poses_[i3] = new us.ihmc.euclid.geometry.Pose3D();
      }
      python_status_message_ = new java.lang.StringBuilder(255);
   }

   public LerobotInferenceOperationMessage(LerobotInferenceOperationMessage other)
   {
      this();
      set(other);
   }

   public void set(LerobotInferenceOperationMessage other)
   {
      ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType.staticCopy(other.latest_timestamp_modifiable_, latest_timestamp_modifiable_);
      sequence_id_ = other.sequence_id_;

      running_ = other.running_;

      control_robot_ = other.control_robot_;

      for(int i5 = 0; i5 < action_hand_poses_.length; ++i5)
      {
            geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.action_hand_poses_[i5], action_hand_poses_[i5]);}

      for(int i7 = 0; i7 < action_forearm_poses_.length; ++i7)
      {
            geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.action_forearm_poses_[i7], action_forearm_poses_[i7]);}

      python_status_frequency_ = other.python_status_frequency_;

      python_status_message_.setLength(0);
      python_status_message_.append(other.python_status_message_);

      received_actions_ = other.received_actions_;

   }


   /**
            * Allows the user to operate the inference of visuomotor policies
            */
   public ihmc_common_msgs.msg.dds.LatestModificationMessage getLatestTimestampModifiable()
   {
      return latest_timestamp_modifiable_;
   }

   /**
            * Monotonically increasing, number of updates of robot thread
            */
   public void setSequenceId(long sequence_id)
   {
      sequence_id_ = sequence_id;
   }
   /**
            * Monotonically increasing, number of updates of robot thread
            */
   public long getSequenceId()
   {
      return sequence_id_;
   }

   /**
            * Whether the inference and action output to IK streaming is active
            */
   public void setRunning(boolean running)
   {
      running_ = running;
   }
   /**
            * Whether the inference and action output to IK streaming is active
            */
   public boolean getRunning()
   {
      return running_;
   }

   /**
            * Whether to enable controlling the robot instead of just previewing
            */
   public void setControlRobot(boolean control_robot)
   {
      control_robot_ = control_robot;
   }
   /**
            * Whether to enable controlling the robot instead of just previewing
            */
   public boolean getControlRobot()
   {
      return control_robot_;
   }


   /**
            * The latest action hand poses
            */
   public us.ihmc.euclid.geometry.Pose3D[] getActionHandPoses()
   {
      return action_hand_poses_;
   }


   /**
            * The latest action forearm poses
            */
   public us.ihmc.euclid.geometry.Pose3D[] getActionForearmPoses()
   {
      return action_forearm_poses_;
   }

   /**
            * Frequency of status from python side
            */
   public void setPythonStatusFrequency(double python_status_frequency)
   {
      python_status_frequency_ = python_status_frequency;
   }
   /**
            * Frequency of status from python side
            */
   public double getPythonStatusFrequency()
   {
      return python_status_frequency_;
   }

   /**
            * Message received from python
            */
   public void setPythonStatusMessage(java.lang.String python_status_message)
   {
      python_status_message_.setLength(0);
      python_status_message_.append(python_status_message);
   }

   /**
            * Message received from python
            */
   public java.lang.String getPythonStatusMessageAsString()
   {
      return getPythonStatusMessage().toString();
   }
   /**
            * Message received from python
            */
   public java.lang.StringBuilder getPythonStatusMessage()
   {
      return python_status_message_;
   }

   /**
            * Number of output actions received from policy
            */
   public void setReceivedActions(long received_actions)
   {
      received_actions_ = received_actions;
   }
   /**
            * Number of output actions received from policy
            */
   public long getReceivedActions()
   {
      return received_actions_;
   }


   public static Supplier<LerobotInferenceOperationMessagePubSubType> getPubSubType()
   {
      return LerobotInferenceOperationMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return LerobotInferenceOperationMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(LerobotInferenceOperationMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.latest_timestamp_modifiable_.epsilonEquals(other.latest_timestamp_modifiable_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.running_, other.running_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.control_robot_, other.control_robot_, epsilon)) return false;

      for(int i9 = 0; i9 < action_hand_poses_.length; ++i9)
      {
              if (!this.action_hand_poses_[i9].epsilonEquals(other.action_hand_poses_[i9], epsilon)) return false;
      }

      for(int i11 = 0; i11 < action_forearm_poses_.length; ++i11)
      {
              if (!this.action_forearm_poses_[i11].epsilonEquals(other.action_forearm_poses_[i11], epsilon)) return false;
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.python_status_frequency_, other.python_status_frequency_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.python_status_message_, other.python_status_message_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.received_actions_, other.received_actions_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof LerobotInferenceOperationMessage)) return false;

      LerobotInferenceOperationMessage otherMyClass = (LerobotInferenceOperationMessage) other;

      if (!this.latest_timestamp_modifiable_.equals(otherMyClass.latest_timestamp_modifiable_)) return false;
      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.running_ != otherMyClass.running_) return false;

      if(this.control_robot_ != otherMyClass.control_robot_) return false;

      for(int i13 = 0; i13 < action_hand_poses_.length; ++i13)
      {
                if (!this.action_hand_poses_[i13].equals(otherMyClass.action_hand_poses_[i13])) return false;
      }
      for(int i15 = 0; i15 < action_forearm_poses_.length; ++i15)
      {
                if (!this.action_forearm_poses_[i15].equals(otherMyClass.action_forearm_poses_[i15])) return false;
      }
      if(this.python_status_frequency_ != otherMyClass.python_status_frequency_) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.python_status_message_, otherMyClass.python_status_message_)) return false;

      if(this.received_actions_ != otherMyClass.received_actions_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("LerobotInferenceOperationMessage {");
      builder.append("latest_timestamp_modifiable=");
      builder.append(this.latest_timestamp_modifiable_);      builder.append(", ");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("running=");
      builder.append(this.running_);      builder.append(", ");
      builder.append("control_robot=");
      builder.append(this.control_robot_);      builder.append(", ");
      builder.append("action_hand_poses=");
      builder.append(java.util.Arrays.toString(this.action_hand_poses_));      builder.append(", ");
      builder.append("action_forearm_poses=");
      builder.append(java.util.Arrays.toString(this.action_forearm_poses_));      builder.append(", ");
      builder.append("python_status_frequency=");
      builder.append(this.python_status_frequency_);      builder.append(", ");
      builder.append("python_status_message=");
      builder.append(this.python_status_message_);      builder.append(", ");
      builder.append("received_actions=");
      builder.append(this.received_actions_);
      builder.append("}");
      return builder.toString();
   }
}
