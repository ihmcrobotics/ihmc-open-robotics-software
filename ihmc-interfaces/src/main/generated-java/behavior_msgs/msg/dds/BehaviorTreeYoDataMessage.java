package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * A message used to send behavior tree data from the automomy process
       * to the controller process in order to be stored as YoVariables.
       * Try to keep this mesage as small as possible.
       */
public class BehaviorTreeYoDataMessage extends Packet<BehaviorTreeYoDataMessage> implements Settable<BehaviorTreeYoDataMessage>, EpsilonComparable<BehaviorTreeYoDataMessage>
{
   public byte number_of_persistent_detections_;
   public byte number_of_scene_objects_;
   public us.ihmc.euclid.geometry.Pose3D[] scene_object_pose_;
   public boolean automatic_execution_;
   public int execution_next_index_;
   public boolean concurrency_enabled_;
   public byte number_of_executing_actions_;
   public byte number_of_failed_actions_;
   public byte[] executing_action_type_;
   public short[] executing_action_id_;
   public float[] elapsed_execution_time_;
   public us.ihmc.euclid.geometry.Pose3D[] current_hand_pose_;
   public us.ihmc.euclid.geometry.Pose3D[] goal_hand_pose_;

   public BehaviorTreeYoDataMessage()
   {
      scene_object_pose_ = new us.ihmc.euclid.geometry.Pose3D[3];

      for(int i1 = 0; i1 < scene_object_pose_.length; ++i1)
      {
          scene_object_pose_[i1] = new us.ihmc.euclid.geometry.Pose3D();
      }
      executing_action_type_ = new byte[5];

      executing_action_id_ = new short[5];

      elapsed_execution_time_ = new float[5];

      current_hand_pose_ = new us.ihmc.euclid.geometry.Pose3D[2];

      for(int i3 = 0; i3 < current_hand_pose_.length; ++i3)
      {
          current_hand_pose_[i3] = new us.ihmc.euclid.geometry.Pose3D();
      }
      goal_hand_pose_ = new us.ihmc.euclid.geometry.Pose3D[2];

      for(int i5 = 0; i5 < goal_hand_pose_.length; ++i5)
      {
          goal_hand_pose_[i5] = new us.ihmc.euclid.geometry.Pose3D();
      }
   }

   public BehaviorTreeYoDataMessage(BehaviorTreeYoDataMessage other)
   {
      this();
      set(other);
   }

   public void set(BehaviorTreeYoDataMessage other)
   {
      number_of_persistent_detections_ = other.number_of_persistent_detections_;

      number_of_scene_objects_ = other.number_of_scene_objects_;

      for(int i7 = 0; i7 < scene_object_pose_.length; ++i7)
      {
            geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.scene_object_pose_[i7], scene_object_pose_[i7]);}

      automatic_execution_ = other.automatic_execution_;

      execution_next_index_ = other.execution_next_index_;

      concurrency_enabled_ = other.concurrency_enabled_;

      number_of_executing_actions_ = other.number_of_executing_actions_;

      number_of_failed_actions_ = other.number_of_failed_actions_;

      for(int i9 = 0; i9 < executing_action_type_.length; ++i9)
      {
            executing_action_type_[i9] = other.executing_action_type_[i9];

      }

      for(int i11 = 0; i11 < executing_action_id_.length; ++i11)
      {
            executing_action_id_[i11] = other.executing_action_id_[i11];

      }

      for(int i13 = 0; i13 < elapsed_execution_time_.length; ++i13)
      {
            elapsed_execution_time_[i13] = other.elapsed_execution_time_[i13];

      }

      for(int i15 = 0; i15 < current_hand_pose_.length; ++i15)
      {
            geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.current_hand_pose_[i15], current_hand_pose_[i15]);}

      for(int i17 = 0; i17 < goal_hand_pose_.length; ++i17)
      {
            geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.goal_hand_pose_[i17], goal_hand_pose_[i17]);}

   }

   public void setNumberOfPersistentDetections(byte number_of_persistent_detections)
   {
      number_of_persistent_detections_ = number_of_persistent_detections;
   }
   public byte getNumberOfPersistentDetections()
   {
      return number_of_persistent_detections_;
   }

   public void setNumberOfSceneObjects(byte number_of_scene_objects)
   {
      number_of_scene_objects_ = number_of_scene_objects;
   }
   public byte getNumberOfSceneObjects()
   {
      return number_of_scene_objects_;
   }


   public us.ihmc.euclid.geometry.Pose3D[] getSceneObjectPose()
   {
      return scene_object_pose_;
   }

   public void setAutomaticExecution(boolean automatic_execution)
   {
      automatic_execution_ = automatic_execution;
   }
   public boolean getAutomaticExecution()
   {
      return automatic_execution_;
   }

   public void setExecutionNextIndex(int execution_next_index)
   {
      execution_next_index_ = execution_next_index;
   }
   public int getExecutionNextIndex()
   {
      return execution_next_index_;
   }

   public void setConcurrencyEnabled(boolean concurrency_enabled)
   {
      concurrency_enabled_ = concurrency_enabled;
   }
   public boolean getConcurrencyEnabled()
   {
      return concurrency_enabled_;
   }

   public void setNumberOfExecutingActions(byte number_of_executing_actions)
   {
      number_of_executing_actions_ = number_of_executing_actions;
   }
   public byte getNumberOfExecutingActions()
   {
      return number_of_executing_actions_;
   }

   public void setNumberOfFailedActions(byte number_of_failed_actions)
   {
      number_of_failed_actions_ = number_of_failed_actions;
   }
   public byte getNumberOfFailedActions()
   {
      return number_of_failed_actions_;
   }


   public byte[] getExecutingActionType()
   {
      return executing_action_type_;
   }


   public short[] getExecutingActionId()
   {
      return executing_action_id_;
   }


   public float[] getElapsedExecutionTime()
   {
      return elapsed_execution_time_;
   }


   public us.ihmc.euclid.geometry.Pose3D[] getCurrentHandPose()
   {
      return current_hand_pose_;
   }


   public us.ihmc.euclid.geometry.Pose3D[] getGoalHandPose()
   {
      return goal_hand_pose_;
   }


   public static Supplier<BehaviorTreeYoDataMessagePubSubType> getPubSubType()
   {
      return BehaviorTreeYoDataMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return BehaviorTreeYoDataMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(BehaviorTreeYoDataMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.number_of_persistent_detections_, other.number_of_persistent_detections_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.number_of_scene_objects_, other.number_of_scene_objects_, epsilon)) return false;

      for(int i19 = 0; i19 < scene_object_pose_.length; ++i19)
      {
              if (!this.scene_object_pose_[i19].epsilonEquals(other.scene_object_pose_[i19], epsilon)) return false;
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.automatic_execution_, other.automatic_execution_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.execution_next_index_, other.execution_next_index_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.concurrency_enabled_, other.concurrency_enabled_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.number_of_executing_actions_, other.number_of_executing_actions_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.number_of_failed_actions_, other.number_of_failed_actions_, epsilon)) return false;

      for(int i21 = 0; i21 < executing_action_type_.length; ++i21)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.executing_action_type_[i21], other.executing_action_type_[i21], epsilon)) return false;
      }

      for(int i23 = 0; i23 < executing_action_id_.length; ++i23)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.executing_action_id_[i23], other.executing_action_id_[i23], epsilon)) return false;
      }

      for(int i25 = 0; i25 < elapsed_execution_time_.length; ++i25)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.elapsed_execution_time_[i25], other.elapsed_execution_time_[i25], epsilon)) return false;
      }

      for(int i27 = 0; i27 < current_hand_pose_.length; ++i27)
      {
              if (!this.current_hand_pose_[i27].epsilonEquals(other.current_hand_pose_[i27], epsilon)) return false;
      }

      for(int i29 = 0; i29 < goal_hand_pose_.length; ++i29)
      {
              if (!this.goal_hand_pose_[i29].epsilonEquals(other.goal_hand_pose_[i29], epsilon)) return false;
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof BehaviorTreeYoDataMessage)) return false;

      BehaviorTreeYoDataMessage otherMyClass = (BehaviorTreeYoDataMessage) other;

      if(this.number_of_persistent_detections_ != otherMyClass.number_of_persistent_detections_) return false;

      if(this.number_of_scene_objects_ != otherMyClass.number_of_scene_objects_) return false;

      for(int i31 = 0; i31 < scene_object_pose_.length; ++i31)
      {
                if (!this.scene_object_pose_[i31].equals(otherMyClass.scene_object_pose_[i31])) return false;
      }
      if(this.automatic_execution_ != otherMyClass.automatic_execution_) return false;

      if(this.execution_next_index_ != otherMyClass.execution_next_index_) return false;

      if(this.concurrency_enabled_ != otherMyClass.concurrency_enabled_) return false;

      if(this.number_of_executing_actions_ != otherMyClass.number_of_executing_actions_) return false;

      if(this.number_of_failed_actions_ != otherMyClass.number_of_failed_actions_) return false;

      for(int i33 = 0; i33 < executing_action_type_.length; ++i33)
      {
                if(this.executing_action_type_[i33] != otherMyClass.executing_action_type_[i33]) return false;

      }
      for(int i35 = 0; i35 < executing_action_id_.length; ++i35)
      {
                if(this.executing_action_id_[i35] != otherMyClass.executing_action_id_[i35]) return false;

      }
      for(int i37 = 0; i37 < elapsed_execution_time_.length; ++i37)
      {
                if(this.elapsed_execution_time_[i37] != otherMyClass.elapsed_execution_time_[i37]) return false;

      }
      for(int i39 = 0; i39 < current_hand_pose_.length; ++i39)
      {
                if (!this.current_hand_pose_[i39].equals(otherMyClass.current_hand_pose_[i39])) return false;
      }
      for(int i41 = 0; i41 < goal_hand_pose_.length; ++i41)
      {
                if (!this.goal_hand_pose_[i41].equals(otherMyClass.goal_hand_pose_[i41])) return false;
      }

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BehaviorTreeYoDataMessage {");
      builder.append("number_of_persistent_detections=");
      builder.append(this.number_of_persistent_detections_);      builder.append(", ");
      builder.append("number_of_scene_objects=");
      builder.append(this.number_of_scene_objects_);      builder.append(", ");
      builder.append("scene_object_pose=");
      builder.append(java.util.Arrays.toString(this.scene_object_pose_));      builder.append(", ");
      builder.append("automatic_execution=");
      builder.append(this.automatic_execution_);      builder.append(", ");
      builder.append("execution_next_index=");
      builder.append(this.execution_next_index_);      builder.append(", ");
      builder.append("concurrency_enabled=");
      builder.append(this.concurrency_enabled_);      builder.append(", ");
      builder.append("number_of_executing_actions=");
      builder.append(this.number_of_executing_actions_);      builder.append(", ");
      builder.append("number_of_failed_actions=");
      builder.append(this.number_of_failed_actions_);      builder.append(", ");
      builder.append("executing_action_type=");
      builder.append(java.util.Arrays.toString(this.executing_action_type_));      builder.append(", ");
      builder.append("executing_action_id=");
      builder.append(java.util.Arrays.toString(this.executing_action_id_));      builder.append(", ");
      builder.append("elapsed_execution_time=");
      builder.append(java.util.Arrays.toString(this.elapsed_execution_time_));      builder.append(", ");
      builder.append("current_hand_pose=");
      builder.append(java.util.Arrays.toString(this.current_hand_pose_));      builder.append(", ");
      builder.append("goal_hand_pose=");
      builder.append(java.util.Arrays.toString(this.goal_hand_pose_));
      builder.append("}");
      return builder.toString();
   }
}
