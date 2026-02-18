package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * A message used to send behavior tree data from the automomy process
       * to the controller process in order to be stored as YoVariables.
       * Currently 211 bytes, try to keep this under 250.
       */
public class BehaviorTreeYoDataMessage extends Packet<BehaviorTreeYoDataMessage> implements Settable<BehaviorTreeYoDataMessage>, EpsilonComparable<BehaviorTreeYoDataMessage>
{
   public byte number_of_persistent_detections_;
   public byte number_of_scene_objects_;
   public float[] scene_object_x_;
   public float[] scene_object_y_;
   public float[] scene_object_z_;
   public float[] scene_object_yaw_;
   public float[] scene_object_pitch_;
   public float[] scene_object_roll_;
   public boolean automatic_execution_;
   public int execution_next_index_;
   public boolean concurrency_enabled_;
   public byte number_of_executing_actions_;
   public byte number_of_failed_actions_;
   public byte[] executing_action_type_;
   public short[] executing_action_id_;
   public float[] elapsed_execution_time_;
   public float[] current_hand_x_;
   public float[] current_hand_y_;
   public float[] current_hand_z_;
   public float[] current_hand_yaw_;
   public float[] current_hand_pitch_;
   public float[] current_hand_roll_;
   public float[] goal_hand_x_;
   public float[] goal_hand_y_;
   public float[] goal_hand_z_;
   public float[] goal_hand_yaw_;
   public float[] goal_hand_pitch_;
   public float[] goal_hand_roll_;

   public BehaviorTreeYoDataMessage()
   {
      scene_object_x_ = new float[3];

      scene_object_y_ = new float[3];

      scene_object_z_ = new float[3];

      scene_object_yaw_ = new float[3];

      scene_object_pitch_ = new float[3];

      scene_object_roll_ = new float[3];

      executing_action_type_ = new byte[5];

      executing_action_id_ = new short[5];

      elapsed_execution_time_ = new float[5];

      current_hand_x_ = new float[2];

      current_hand_y_ = new float[2];

      current_hand_z_ = new float[2];

      current_hand_yaw_ = new float[2];

      current_hand_pitch_ = new float[2];

      current_hand_roll_ = new float[2];

      goal_hand_x_ = new float[2];

      goal_hand_y_ = new float[2];

      goal_hand_z_ = new float[2];

      goal_hand_yaw_ = new float[2];

      goal_hand_pitch_ = new float[2];

      goal_hand_roll_ = new float[2];

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

      for(int i1 = 0; i1 < scene_object_x_.length; ++i1)
      {
            scene_object_x_[i1] = other.scene_object_x_[i1];

      }

      for(int i3 = 0; i3 < scene_object_y_.length; ++i3)
      {
            scene_object_y_[i3] = other.scene_object_y_[i3];

      }

      for(int i5 = 0; i5 < scene_object_z_.length; ++i5)
      {
            scene_object_z_[i5] = other.scene_object_z_[i5];

      }

      for(int i7 = 0; i7 < scene_object_yaw_.length; ++i7)
      {
            scene_object_yaw_[i7] = other.scene_object_yaw_[i7];

      }

      for(int i9 = 0; i9 < scene_object_pitch_.length; ++i9)
      {
            scene_object_pitch_[i9] = other.scene_object_pitch_[i9];

      }

      for(int i11 = 0; i11 < scene_object_roll_.length; ++i11)
      {
            scene_object_roll_[i11] = other.scene_object_roll_[i11];

      }

      automatic_execution_ = other.automatic_execution_;

      execution_next_index_ = other.execution_next_index_;

      concurrency_enabled_ = other.concurrency_enabled_;

      number_of_executing_actions_ = other.number_of_executing_actions_;

      number_of_failed_actions_ = other.number_of_failed_actions_;

      for(int i13 = 0; i13 < executing_action_type_.length; ++i13)
      {
            executing_action_type_[i13] = other.executing_action_type_[i13];

      }

      for(int i15 = 0; i15 < executing_action_id_.length; ++i15)
      {
            executing_action_id_[i15] = other.executing_action_id_[i15];

      }

      for(int i17 = 0; i17 < elapsed_execution_time_.length; ++i17)
      {
            elapsed_execution_time_[i17] = other.elapsed_execution_time_[i17];

      }

      for(int i19 = 0; i19 < current_hand_x_.length; ++i19)
      {
            current_hand_x_[i19] = other.current_hand_x_[i19];

      }

      for(int i21 = 0; i21 < current_hand_y_.length; ++i21)
      {
            current_hand_y_[i21] = other.current_hand_y_[i21];

      }

      for(int i23 = 0; i23 < current_hand_z_.length; ++i23)
      {
            current_hand_z_[i23] = other.current_hand_z_[i23];

      }

      for(int i25 = 0; i25 < current_hand_yaw_.length; ++i25)
      {
            current_hand_yaw_[i25] = other.current_hand_yaw_[i25];

      }

      for(int i27 = 0; i27 < current_hand_pitch_.length; ++i27)
      {
            current_hand_pitch_[i27] = other.current_hand_pitch_[i27];

      }

      for(int i29 = 0; i29 < current_hand_roll_.length; ++i29)
      {
            current_hand_roll_[i29] = other.current_hand_roll_[i29];

      }

      for(int i31 = 0; i31 < goal_hand_x_.length; ++i31)
      {
            goal_hand_x_[i31] = other.goal_hand_x_[i31];

      }

      for(int i33 = 0; i33 < goal_hand_y_.length; ++i33)
      {
            goal_hand_y_[i33] = other.goal_hand_y_[i33];

      }

      for(int i35 = 0; i35 < goal_hand_z_.length; ++i35)
      {
            goal_hand_z_[i35] = other.goal_hand_z_[i35];

      }

      for(int i37 = 0; i37 < goal_hand_yaw_.length; ++i37)
      {
            goal_hand_yaw_[i37] = other.goal_hand_yaw_[i37];

      }

      for(int i39 = 0; i39 < goal_hand_pitch_.length; ++i39)
      {
            goal_hand_pitch_[i39] = other.goal_hand_pitch_[i39];

      }

      for(int i41 = 0; i41 < goal_hand_roll_.length; ++i41)
      {
            goal_hand_roll_[i41] = other.goal_hand_roll_[i41];

      }

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


   public float[] getSceneObjectX()
   {
      return scene_object_x_;
   }


   public float[] getSceneObjectY()
   {
      return scene_object_y_;
   }


   public float[] getSceneObjectZ()
   {
      return scene_object_z_;
   }


   public float[] getSceneObjectYaw()
   {
      return scene_object_yaw_;
   }


   public float[] getSceneObjectPitch()
   {
      return scene_object_pitch_;
   }


   public float[] getSceneObjectRoll()
   {
      return scene_object_roll_;
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


   public float[] getCurrentHandX()
   {
      return current_hand_x_;
   }


   public float[] getCurrentHandY()
   {
      return current_hand_y_;
   }


   public float[] getCurrentHandZ()
   {
      return current_hand_z_;
   }


   public float[] getCurrentHandYaw()
   {
      return current_hand_yaw_;
   }


   public float[] getCurrentHandPitch()
   {
      return current_hand_pitch_;
   }


   public float[] getCurrentHandRoll()
   {
      return current_hand_roll_;
   }


   public float[] getGoalHandX()
   {
      return goal_hand_x_;
   }


   public float[] getGoalHandY()
   {
      return goal_hand_y_;
   }


   public float[] getGoalHandZ()
   {
      return goal_hand_z_;
   }


   public float[] getGoalHandYaw()
   {
      return goal_hand_yaw_;
   }


   public float[] getGoalHandPitch()
   {
      return goal_hand_pitch_;
   }


   public float[] getGoalHandRoll()
   {
      return goal_hand_roll_;
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

      for(int i43 = 0; i43 < scene_object_x_.length; ++i43)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.scene_object_x_[i43], other.scene_object_x_[i43], epsilon)) return false;
      }

      for(int i45 = 0; i45 < scene_object_y_.length; ++i45)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.scene_object_y_[i45], other.scene_object_y_[i45], epsilon)) return false;
      }

      for(int i47 = 0; i47 < scene_object_z_.length; ++i47)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.scene_object_z_[i47], other.scene_object_z_[i47], epsilon)) return false;
      }

      for(int i49 = 0; i49 < scene_object_yaw_.length; ++i49)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.scene_object_yaw_[i49], other.scene_object_yaw_[i49], epsilon)) return false;
      }

      for(int i51 = 0; i51 < scene_object_pitch_.length; ++i51)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.scene_object_pitch_[i51], other.scene_object_pitch_[i51], epsilon)) return false;
      }

      for(int i53 = 0; i53 < scene_object_roll_.length; ++i53)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.scene_object_roll_[i53], other.scene_object_roll_[i53], epsilon)) return false;
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.automatic_execution_, other.automatic_execution_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.execution_next_index_, other.execution_next_index_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.concurrency_enabled_, other.concurrency_enabled_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.number_of_executing_actions_, other.number_of_executing_actions_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.number_of_failed_actions_, other.number_of_failed_actions_, epsilon)) return false;

      for(int i55 = 0; i55 < executing_action_type_.length; ++i55)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.executing_action_type_[i55], other.executing_action_type_[i55], epsilon)) return false;
      }

      for(int i57 = 0; i57 < executing_action_id_.length; ++i57)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.executing_action_id_[i57], other.executing_action_id_[i57], epsilon)) return false;
      }

      for(int i59 = 0; i59 < elapsed_execution_time_.length; ++i59)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.elapsed_execution_time_[i59], other.elapsed_execution_time_[i59], epsilon)) return false;
      }

      for(int i61 = 0; i61 < current_hand_x_.length; ++i61)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_hand_x_[i61], other.current_hand_x_[i61], epsilon)) return false;
      }

      for(int i63 = 0; i63 < current_hand_y_.length; ++i63)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_hand_y_[i63], other.current_hand_y_[i63], epsilon)) return false;
      }

      for(int i65 = 0; i65 < current_hand_z_.length; ++i65)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_hand_z_[i65], other.current_hand_z_[i65], epsilon)) return false;
      }

      for(int i67 = 0; i67 < current_hand_yaw_.length; ++i67)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_hand_yaw_[i67], other.current_hand_yaw_[i67], epsilon)) return false;
      }

      for(int i69 = 0; i69 < current_hand_pitch_.length; ++i69)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_hand_pitch_[i69], other.current_hand_pitch_[i69], epsilon)) return false;
      }

      for(int i71 = 0; i71 < current_hand_roll_.length; ++i71)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_hand_roll_[i71], other.current_hand_roll_[i71], epsilon)) return false;
      }

      for(int i73 = 0; i73 < goal_hand_x_.length; ++i73)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.goal_hand_x_[i73], other.goal_hand_x_[i73], epsilon)) return false;
      }

      for(int i75 = 0; i75 < goal_hand_y_.length; ++i75)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.goal_hand_y_[i75], other.goal_hand_y_[i75], epsilon)) return false;
      }

      for(int i77 = 0; i77 < goal_hand_z_.length; ++i77)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.goal_hand_z_[i77], other.goal_hand_z_[i77], epsilon)) return false;
      }

      for(int i79 = 0; i79 < goal_hand_yaw_.length; ++i79)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.goal_hand_yaw_[i79], other.goal_hand_yaw_[i79], epsilon)) return false;
      }

      for(int i81 = 0; i81 < goal_hand_pitch_.length; ++i81)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.goal_hand_pitch_[i81], other.goal_hand_pitch_[i81], epsilon)) return false;
      }

      for(int i83 = 0; i83 < goal_hand_roll_.length; ++i83)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.goal_hand_roll_[i83], other.goal_hand_roll_[i83], epsilon)) return false;
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

      for(int i85 = 0; i85 < scene_object_x_.length; ++i85)
      {
                if(this.scene_object_x_[i85] != otherMyClass.scene_object_x_[i85]) return false;

      }
      for(int i87 = 0; i87 < scene_object_y_.length; ++i87)
      {
                if(this.scene_object_y_[i87] != otherMyClass.scene_object_y_[i87]) return false;

      }
      for(int i89 = 0; i89 < scene_object_z_.length; ++i89)
      {
                if(this.scene_object_z_[i89] != otherMyClass.scene_object_z_[i89]) return false;

      }
      for(int i91 = 0; i91 < scene_object_yaw_.length; ++i91)
      {
                if(this.scene_object_yaw_[i91] != otherMyClass.scene_object_yaw_[i91]) return false;

      }
      for(int i93 = 0; i93 < scene_object_pitch_.length; ++i93)
      {
                if(this.scene_object_pitch_[i93] != otherMyClass.scene_object_pitch_[i93]) return false;

      }
      for(int i95 = 0; i95 < scene_object_roll_.length; ++i95)
      {
                if(this.scene_object_roll_[i95] != otherMyClass.scene_object_roll_[i95]) return false;

      }
      if(this.automatic_execution_ != otherMyClass.automatic_execution_) return false;

      if(this.execution_next_index_ != otherMyClass.execution_next_index_) return false;

      if(this.concurrency_enabled_ != otherMyClass.concurrency_enabled_) return false;

      if(this.number_of_executing_actions_ != otherMyClass.number_of_executing_actions_) return false;

      if(this.number_of_failed_actions_ != otherMyClass.number_of_failed_actions_) return false;

      for(int i97 = 0; i97 < executing_action_type_.length; ++i97)
      {
                if(this.executing_action_type_[i97] != otherMyClass.executing_action_type_[i97]) return false;

      }
      for(int i99 = 0; i99 < executing_action_id_.length; ++i99)
      {
                if(this.executing_action_id_[i99] != otherMyClass.executing_action_id_[i99]) return false;

      }
      for(int i101 = 0; i101 < elapsed_execution_time_.length; ++i101)
      {
                if(this.elapsed_execution_time_[i101] != otherMyClass.elapsed_execution_time_[i101]) return false;

      }
      for(int i103 = 0; i103 < current_hand_x_.length; ++i103)
      {
                if(this.current_hand_x_[i103] != otherMyClass.current_hand_x_[i103]) return false;

      }
      for(int i105 = 0; i105 < current_hand_y_.length; ++i105)
      {
                if(this.current_hand_y_[i105] != otherMyClass.current_hand_y_[i105]) return false;

      }
      for(int i107 = 0; i107 < current_hand_z_.length; ++i107)
      {
                if(this.current_hand_z_[i107] != otherMyClass.current_hand_z_[i107]) return false;

      }
      for(int i109 = 0; i109 < current_hand_yaw_.length; ++i109)
      {
                if(this.current_hand_yaw_[i109] != otherMyClass.current_hand_yaw_[i109]) return false;

      }
      for(int i111 = 0; i111 < current_hand_pitch_.length; ++i111)
      {
                if(this.current_hand_pitch_[i111] != otherMyClass.current_hand_pitch_[i111]) return false;

      }
      for(int i113 = 0; i113 < current_hand_roll_.length; ++i113)
      {
                if(this.current_hand_roll_[i113] != otherMyClass.current_hand_roll_[i113]) return false;

      }
      for(int i115 = 0; i115 < goal_hand_x_.length; ++i115)
      {
                if(this.goal_hand_x_[i115] != otherMyClass.goal_hand_x_[i115]) return false;

      }
      for(int i117 = 0; i117 < goal_hand_y_.length; ++i117)
      {
                if(this.goal_hand_y_[i117] != otherMyClass.goal_hand_y_[i117]) return false;

      }
      for(int i119 = 0; i119 < goal_hand_z_.length; ++i119)
      {
                if(this.goal_hand_z_[i119] != otherMyClass.goal_hand_z_[i119]) return false;

      }
      for(int i121 = 0; i121 < goal_hand_yaw_.length; ++i121)
      {
                if(this.goal_hand_yaw_[i121] != otherMyClass.goal_hand_yaw_[i121]) return false;

      }
      for(int i123 = 0; i123 < goal_hand_pitch_.length; ++i123)
      {
                if(this.goal_hand_pitch_[i123] != otherMyClass.goal_hand_pitch_[i123]) return false;

      }
      for(int i125 = 0; i125 < goal_hand_roll_.length; ++i125)
      {
                if(this.goal_hand_roll_[i125] != otherMyClass.goal_hand_roll_[i125]) return false;

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
      builder.append("scene_object_x=");
      builder.append(java.util.Arrays.toString(this.scene_object_x_));      builder.append(", ");
      builder.append("scene_object_y=");
      builder.append(java.util.Arrays.toString(this.scene_object_y_));      builder.append(", ");
      builder.append("scene_object_z=");
      builder.append(java.util.Arrays.toString(this.scene_object_z_));      builder.append(", ");
      builder.append("scene_object_yaw=");
      builder.append(java.util.Arrays.toString(this.scene_object_yaw_));      builder.append(", ");
      builder.append("scene_object_pitch=");
      builder.append(java.util.Arrays.toString(this.scene_object_pitch_));      builder.append(", ");
      builder.append("scene_object_roll=");
      builder.append(java.util.Arrays.toString(this.scene_object_roll_));      builder.append(", ");
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
      builder.append("current_hand_x=");
      builder.append(java.util.Arrays.toString(this.current_hand_x_));      builder.append(", ");
      builder.append("current_hand_y=");
      builder.append(java.util.Arrays.toString(this.current_hand_y_));      builder.append(", ");
      builder.append("current_hand_z=");
      builder.append(java.util.Arrays.toString(this.current_hand_z_));      builder.append(", ");
      builder.append("current_hand_yaw=");
      builder.append(java.util.Arrays.toString(this.current_hand_yaw_));      builder.append(", ");
      builder.append("current_hand_pitch=");
      builder.append(java.util.Arrays.toString(this.current_hand_pitch_));      builder.append(", ");
      builder.append("current_hand_roll=");
      builder.append(java.util.Arrays.toString(this.current_hand_roll_));      builder.append(", ");
      builder.append("goal_hand_x=");
      builder.append(java.util.Arrays.toString(this.goal_hand_x_));      builder.append(", ");
      builder.append("goal_hand_y=");
      builder.append(java.util.Arrays.toString(this.goal_hand_y_));      builder.append(", ");
      builder.append("goal_hand_z=");
      builder.append(java.util.Arrays.toString(this.goal_hand_z_));      builder.append(", ");
      builder.append("goal_hand_yaw=");
      builder.append(java.util.Arrays.toString(this.goal_hand_yaw_));      builder.append(", ");
      builder.append("goal_hand_pitch=");
      builder.append(java.util.Arrays.toString(this.goal_hand_pitch_));      builder.append(", ");
      builder.append("goal_hand_roll=");
      builder.append(java.util.Arrays.toString(this.goal_hand_roll_));
      builder.append("}");
      return builder.toString();
   }
}
