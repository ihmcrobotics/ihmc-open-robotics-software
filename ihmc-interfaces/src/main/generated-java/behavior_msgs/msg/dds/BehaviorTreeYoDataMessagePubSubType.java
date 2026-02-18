package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "BehaviorTreeYoDataMessage" defined in "BehaviorTreeYoDataMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from BehaviorTreeYoDataMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit BehaviorTreeYoDataMessage_.idl instead.
*
*/
public class BehaviorTreeYoDataMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.BehaviorTreeYoDataMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::BehaviorTreeYoDataMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "5e41e24d021796402ad1608a8e6b366631ff7d0b50d32eb62e0892c17e308cf7";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.BehaviorTreeYoDataMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.BehaviorTreeYoDataMessage data) throws java.io.IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      read(data, deserializeCDR);
      deserializeCDR.finishDeserialize();
   }

   public static int getMaxCdrSerializedSize()
   {
      return getMaxCdrSerializedSize(0);
   }

   public static int getMaxCdrSerializedSize(int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += ((3) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += ((3) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += ((3) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += ((3) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += ((3) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += ((3) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += ((5) * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += ((5) * 2) + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += ((5) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += ((2) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += ((2) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += ((2) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += ((2) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += ((2) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += ((2) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += ((2) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += ((2) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += ((2) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += ((2) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += ((2) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += ((2) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.BehaviorTreeYoDataMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.BehaviorTreeYoDataMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += ((3) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += ((3) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += ((3) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += ((3) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += ((3) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += ((3) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += ((5) * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);
      current_alignment += ((5) * 2) + us.ihmc.idl.CDR.alignment(current_alignment, 2);
      current_alignment += ((5) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += ((2) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += ((2) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += ((2) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += ((2) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += ((2) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += ((2) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += ((2) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += ((2) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += ((2) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += ((2) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += ((2) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += ((2) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.BehaviorTreeYoDataMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_9(data.getNumberOfPersistentDetections());

      cdr.write_type_9(data.getNumberOfSceneObjects());

      for(int i0 = 0; i0 < data.getSceneObjectX().length; ++i0)
      {
        	cdr.write_type_5(data.getSceneObjectX()[i0]);	
      }

      for(int i0 = 0; i0 < data.getSceneObjectY().length; ++i0)
      {
        	cdr.write_type_5(data.getSceneObjectY()[i0]);	
      }

      for(int i0 = 0; i0 < data.getSceneObjectZ().length; ++i0)
      {
        	cdr.write_type_5(data.getSceneObjectZ()[i0]);	
      }

      for(int i0 = 0; i0 < data.getSceneObjectYaw().length; ++i0)
      {
        	cdr.write_type_5(data.getSceneObjectYaw()[i0]);	
      }

      for(int i0 = 0; i0 < data.getSceneObjectPitch().length; ++i0)
      {
        	cdr.write_type_5(data.getSceneObjectPitch()[i0]);	
      }

      for(int i0 = 0; i0 < data.getSceneObjectRoll().length; ++i0)
      {
        	cdr.write_type_5(data.getSceneObjectRoll()[i0]);	
      }

      cdr.write_type_7(data.getAutomaticExecution());

      cdr.write_type_3(data.getExecutionNextIndex());

      cdr.write_type_7(data.getConcurrencyEnabled());

      cdr.write_type_9(data.getNumberOfExecutingActions());

      cdr.write_type_9(data.getNumberOfFailedActions());

      for(int i0 = 0; i0 < data.getExecutingActionType().length; ++i0)
      {
        	cdr.write_type_9(data.getExecutingActionType()[i0]);	
      }

      for(int i0 = 0; i0 < data.getExecutingActionId().length; ++i0)
      {
        	cdr.write_type_1(data.getExecutingActionId()[i0]);	
      }

      for(int i0 = 0; i0 < data.getElapsedExecutionTime().length; ++i0)
      {
        	cdr.write_type_5(data.getElapsedExecutionTime()[i0]);	
      }

      for(int i0 = 0; i0 < data.getCurrentHandX().length; ++i0)
      {
        	cdr.write_type_5(data.getCurrentHandX()[i0]);	
      }

      for(int i0 = 0; i0 < data.getCurrentHandY().length; ++i0)
      {
        	cdr.write_type_5(data.getCurrentHandY()[i0]);	
      }

      for(int i0 = 0; i0 < data.getCurrentHandZ().length; ++i0)
      {
        	cdr.write_type_5(data.getCurrentHandZ()[i0]);	
      }

      for(int i0 = 0; i0 < data.getCurrentHandYaw().length; ++i0)
      {
        	cdr.write_type_5(data.getCurrentHandYaw()[i0]);	
      }

      for(int i0 = 0; i0 < data.getCurrentHandPitch().length; ++i0)
      {
        	cdr.write_type_5(data.getCurrentHandPitch()[i0]);	
      }

      for(int i0 = 0; i0 < data.getCurrentHandRoll().length; ++i0)
      {
        	cdr.write_type_5(data.getCurrentHandRoll()[i0]);	
      }

      for(int i0 = 0; i0 < data.getGoalHandX().length; ++i0)
      {
        	cdr.write_type_5(data.getGoalHandX()[i0]);	
      }

      for(int i0 = 0; i0 < data.getGoalHandY().length; ++i0)
      {
        	cdr.write_type_5(data.getGoalHandY()[i0]);	
      }

      for(int i0 = 0; i0 < data.getGoalHandZ().length; ++i0)
      {
        	cdr.write_type_5(data.getGoalHandZ()[i0]);	
      }

      for(int i0 = 0; i0 < data.getGoalHandYaw().length; ++i0)
      {
        	cdr.write_type_5(data.getGoalHandYaw()[i0]);	
      }

      for(int i0 = 0; i0 < data.getGoalHandPitch().length; ++i0)
      {
        	cdr.write_type_5(data.getGoalHandPitch()[i0]);	
      }

      for(int i0 = 0; i0 < data.getGoalHandRoll().length; ++i0)
      {
        	cdr.write_type_5(data.getGoalHandRoll()[i0]);	
      }

   }

   public static void read(behavior_msgs.msg.dds.BehaviorTreeYoDataMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setNumberOfPersistentDetections(cdr.read_type_9());
      	
      data.setNumberOfSceneObjects(cdr.read_type_9());
      	
      for(int i0 = 0; i0 < data.getSceneObjectX().length; ++i0)
      {
        	data.getSceneObjectX()[i0] = cdr.read_type_5();
        	
      }
      	
      for(int i0 = 0; i0 < data.getSceneObjectY().length; ++i0)
      {
        	data.getSceneObjectY()[i0] = cdr.read_type_5();
        	
      }
      	
      for(int i0 = 0; i0 < data.getSceneObjectZ().length; ++i0)
      {
        	data.getSceneObjectZ()[i0] = cdr.read_type_5();
        	
      }
      	
      for(int i0 = 0; i0 < data.getSceneObjectYaw().length; ++i0)
      {
        	data.getSceneObjectYaw()[i0] = cdr.read_type_5();
        	
      }
      	
      for(int i0 = 0; i0 < data.getSceneObjectPitch().length; ++i0)
      {
        	data.getSceneObjectPitch()[i0] = cdr.read_type_5();
        	
      }
      	
      for(int i0 = 0; i0 < data.getSceneObjectRoll().length; ++i0)
      {
        	data.getSceneObjectRoll()[i0] = cdr.read_type_5();
        	
      }
      	
      data.setAutomaticExecution(cdr.read_type_7());
      	
      data.setExecutionNextIndex(cdr.read_type_3());
      	
      data.setConcurrencyEnabled(cdr.read_type_7());
      	
      data.setNumberOfExecutingActions(cdr.read_type_9());
      	
      data.setNumberOfFailedActions(cdr.read_type_9());
      	
      for(int i0 = 0; i0 < data.getExecutingActionType().length; ++i0)
      {
        	data.getExecutingActionType()[i0] = cdr.read_type_9();
        	
      }
      	
      for(int i0 = 0; i0 < data.getExecutingActionId().length; ++i0)
      {
        	data.getExecutingActionId()[i0] = cdr.read_type_1();
        	
      }
      	
      for(int i0 = 0; i0 < data.getElapsedExecutionTime().length; ++i0)
      {
        	data.getElapsedExecutionTime()[i0] = cdr.read_type_5();
        	
      }
      	
      for(int i0 = 0; i0 < data.getCurrentHandX().length; ++i0)
      {
        	data.getCurrentHandX()[i0] = cdr.read_type_5();
        	
      }
      	
      for(int i0 = 0; i0 < data.getCurrentHandY().length; ++i0)
      {
        	data.getCurrentHandY()[i0] = cdr.read_type_5();
        	
      }
      	
      for(int i0 = 0; i0 < data.getCurrentHandZ().length; ++i0)
      {
        	data.getCurrentHandZ()[i0] = cdr.read_type_5();
        	
      }
      	
      for(int i0 = 0; i0 < data.getCurrentHandYaw().length; ++i0)
      {
        	data.getCurrentHandYaw()[i0] = cdr.read_type_5();
        	
      }
      	
      for(int i0 = 0; i0 < data.getCurrentHandPitch().length; ++i0)
      {
        	data.getCurrentHandPitch()[i0] = cdr.read_type_5();
        	
      }
      	
      for(int i0 = 0; i0 < data.getCurrentHandRoll().length; ++i0)
      {
        	data.getCurrentHandRoll()[i0] = cdr.read_type_5();
        	
      }
      	
      for(int i0 = 0; i0 < data.getGoalHandX().length; ++i0)
      {
        	data.getGoalHandX()[i0] = cdr.read_type_5();
        	
      }
      	
      for(int i0 = 0; i0 < data.getGoalHandY().length; ++i0)
      {
        	data.getGoalHandY()[i0] = cdr.read_type_5();
        	
      }
      	
      for(int i0 = 0; i0 < data.getGoalHandZ().length; ++i0)
      {
        	data.getGoalHandZ()[i0] = cdr.read_type_5();
        	
      }
      	
      for(int i0 = 0; i0 < data.getGoalHandYaw().length; ++i0)
      {
        	data.getGoalHandYaw()[i0] = cdr.read_type_5();
        	
      }
      	
      for(int i0 = 0; i0 < data.getGoalHandPitch().length; ++i0)
      {
        	data.getGoalHandPitch()[i0] = cdr.read_type_5();
        	
      }
      	
      for(int i0 = 0; i0 < data.getGoalHandRoll().length; ++i0)
      {
        	data.getGoalHandRoll()[i0] = cdr.read_type_5();
        	
      }
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.BehaviorTreeYoDataMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_9("number_of_persistent_detections", data.getNumberOfPersistentDetections());
      ser.write_type_9("number_of_scene_objects", data.getNumberOfSceneObjects());
      ser.write_type_f("scene_object_x", data.getSceneObjectX());
      ser.write_type_f("scene_object_y", data.getSceneObjectY());
      ser.write_type_f("scene_object_z", data.getSceneObjectZ());
      ser.write_type_f("scene_object_yaw", data.getSceneObjectYaw());
      ser.write_type_f("scene_object_pitch", data.getSceneObjectPitch());
      ser.write_type_f("scene_object_roll", data.getSceneObjectRoll());
      ser.write_type_7("automatic_execution", data.getAutomaticExecution());
      ser.write_type_3("execution_next_index", data.getExecutionNextIndex());
      ser.write_type_7("concurrency_enabled", data.getConcurrencyEnabled());
      ser.write_type_9("number_of_executing_actions", data.getNumberOfExecutingActions());
      ser.write_type_9("number_of_failed_actions", data.getNumberOfFailedActions());
      ser.write_type_f("executing_action_type", data.getExecutingActionType());
      ser.write_type_f("executing_action_id", data.getExecutingActionId());
      ser.write_type_f("elapsed_execution_time", data.getElapsedExecutionTime());
      ser.write_type_f("current_hand_x", data.getCurrentHandX());
      ser.write_type_f("current_hand_y", data.getCurrentHandY());
      ser.write_type_f("current_hand_z", data.getCurrentHandZ());
      ser.write_type_f("current_hand_yaw", data.getCurrentHandYaw());
      ser.write_type_f("current_hand_pitch", data.getCurrentHandPitch());
      ser.write_type_f("current_hand_roll", data.getCurrentHandRoll());
      ser.write_type_f("goal_hand_x", data.getGoalHandX());
      ser.write_type_f("goal_hand_y", data.getGoalHandY());
      ser.write_type_f("goal_hand_z", data.getGoalHandZ());
      ser.write_type_f("goal_hand_yaw", data.getGoalHandYaw());
      ser.write_type_f("goal_hand_pitch", data.getGoalHandPitch());
      ser.write_type_f("goal_hand_roll", data.getGoalHandRoll());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.BehaviorTreeYoDataMessage data)
   {
      data.setNumberOfPersistentDetections(ser.read_type_9("number_of_persistent_detections"));
      data.setNumberOfSceneObjects(ser.read_type_9("number_of_scene_objects"));
      ser.read_type_f("scene_object_x", data.getSceneObjectX());
      ser.read_type_f("scene_object_y", data.getSceneObjectY());
      ser.read_type_f("scene_object_z", data.getSceneObjectZ());
      ser.read_type_f("scene_object_yaw", data.getSceneObjectYaw());
      ser.read_type_f("scene_object_pitch", data.getSceneObjectPitch());
      ser.read_type_f("scene_object_roll", data.getSceneObjectRoll());
      data.setAutomaticExecution(ser.read_type_7("automatic_execution"));
      data.setExecutionNextIndex(ser.read_type_3("execution_next_index"));
      data.setConcurrencyEnabled(ser.read_type_7("concurrency_enabled"));
      data.setNumberOfExecutingActions(ser.read_type_9("number_of_executing_actions"));
      data.setNumberOfFailedActions(ser.read_type_9("number_of_failed_actions"));
      ser.read_type_f("executing_action_type", data.getExecutingActionType());
      ser.read_type_f("executing_action_id", data.getExecutingActionId());
      ser.read_type_f("elapsed_execution_time", data.getElapsedExecutionTime());
      ser.read_type_f("current_hand_x", data.getCurrentHandX());
      ser.read_type_f("current_hand_y", data.getCurrentHandY());
      ser.read_type_f("current_hand_z", data.getCurrentHandZ());
      ser.read_type_f("current_hand_yaw", data.getCurrentHandYaw());
      ser.read_type_f("current_hand_pitch", data.getCurrentHandPitch());
      ser.read_type_f("current_hand_roll", data.getCurrentHandRoll());
      ser.read_type_f("goal_hand_x", data.getGoalHandX());
      ser.read_type_f("goal_hand_y", data.getGoalHandY());
      ser.read_type_f("goal_hand_z", data.getGoalHandZ());
      ser.read_type_f("goal_hand_yaw", data.getGoalHandYaw());
      ser.read_type_f("goal_hand_pitch", data.getGoalHandPitch());
      ser.read_type_f("goal_hand_roll", data.getGoalHandRoll());
   }

   public static void staticCopy(behavior_msgs.msg.dds.BehaviorTreeYoDataMessage src, behavior_msgs.msg.dds.BehaviorTreeYoDataMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.BehaviorTreeYoDataMessage createData()
   {
      return new behavior_msgs.msg.dds.BehaviorTreeYoDataMessage();
   }
   @Override
   public int getTypeSize()
   {
      return us.ihmc.idl.CDR.getTypeSize(getMaxCdrSerializedSize());
   }

   @Override
   public java.lang.String getName()
   {
      return name;
   }
   
   public void serialize(behavior_msgs.msg.dds.BehaviorTreeYoDataMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.BehaviorTreeYoDataMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.BehaviorTreeYoDataMessage src, behavior_msgs.msg.dds.BehaviorTreeYoDataMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public BehaviorTreeYoDataMessagePubSubType newInstance()
   {
      return new BehaviorTreeYoDataMessagePubSubType();
   }
}
