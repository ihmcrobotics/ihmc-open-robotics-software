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
   		return "094c3b7f7dabaf4523837623fe0a4e5e692b0a966162fc41462ea72c6f569415";
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

      for(int i0 = 0; i0 < (3); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += ((5) * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += ((5) * 2) + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += ((5) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      for(int i0 = 0; i0 < (2); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);}
      for(int i0 = 0; i0 < (2); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);}

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


      for(int i0 = 0; i0 < data.getSceneObjectPose().length; ++i0)
      {
              current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getSceneObjectPose()[i0], current_alignment);
      }
      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += ((5) * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);
      current_alignment += ((5) * 2) + us.ihmc.idl.CDR.alignment(current_alignment, 2);
      current_alignment += ((5) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getCurrentHandPose().length; ++i0)
      {
              current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getCurrentHandPose()[i0], current_alignment);
      }
      for(int i0 = 0; i0 < data.getGoalHandPose().length; ++i0)
      {
              current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getGoalHandPose()[i0], current_alignment);
      }

      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.BehaviorTreeYoDataMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_9(data.getNumberOfPersistentDetections());

      cdr.write_type_9(data.getNumberOfSceneObjects());

      for(int i0 = 0; i0 < data.getSceneObjectPose().length; ++i0)
      {
        	geometry_msgs.msg.dds.PosePubSubType.write(data.getSceneObjectPose()[i0], cdr);		
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

      for(int i0 = 0; i0 < data.getCurrentHandPose().length; ++i0)
      {
        	geometry_msgs.msg.dds.PosePubSubType.write(data.getCurrentHandPose()[i0], cdr);		
      }

      for(int i0 = 0; i0 < data.getGoalHandPose().length; ++i0)
      {
        	geometry_msgs.msg.dds.PosePubSubType.write(data.getGoalHandPose()[i0], cdr);		
      }

   }

   public static void read(behavior_msgs.msg.dds.BehaviorTreeYoDataMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setNumberOfPersistentDetections(cdr.read_type_9());
      	
      data.setNumberOfSceneObjects(cdr.read_type_9());
      	
      for(int i0 = 0; i0 < data.getSceneObjectPose().length; ++i0)
      {
        	geometry_msgs.msg.dds.PosePubSubType.read(data.getSceneObjectPose()[i0], cdr);	
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
      	
      for(int i0 = 0; i0 < data.getCurrentHandPose().length; ++i0)
      {
        	geometry_msgs.msg.dds.PosePubSubType.read(data.getCurrentHandPose()[i0], cdr);	
      }
      	
      for(int i0 = 0; i0 < data.getGoalHandPose().length; ++i0)
      {
        	geometry_msgs.msg.dds.PosePubSubType.read(data.getGoalHandPose()[i0], cdr);	
      }
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.BehaviorTreeYoDataMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_9("number_of_persistent_detections", data.getNumberOfPersistentDetections());
      ser.write_type_9("number_of_scene_objects", data.getNumberOfSceneObjects());
      ser.write_type_f("scene_object_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getSceneObjectPose());
      ser.write_type_7("automatic_execution", data.getAutomaticExecution());
      ser.write_type_3("execution_next_index", data.getExecutionNextIndex());
      ser.write_type_7("concurrency_enabled", data.getConcurrencyEnabled());
      ser.write_type_9("number_of_executing_actions", data.getNumberOfExecutingActions());
      ser.write_type_9("number_of_failed_actions", data.getNumberOfFailedActions());
      ser.write_type_f("executing_action_type", data.getExecutingActionType());
      ser.write_type_f("executing_action_id", data.getExecutingActionId());
      ser.write_type_f("elapsed_execution_time", data.getElapsedExecutionTime());
      ser.write_type_f("current_hand_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getCurrentHandPose());
      ser.write_type_f("goal_hand_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getGoalHandPose());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.BehaviorTreeYoDataMessage data)
   {
      data.setNumberOfPersistentDetections(ser.read_type_9("number_of_persistent_detections"));
      data.setNumberOfSceneObjects(ser.read_type_9("number_of_scene_objects"));
      ser.read_type_f("scene_object_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getSceneObjectPose());
      data.setAutomaticExecution(ser.read_type_7("automatic_execution"));
      data.setExecutionNextIndex(ser.read_type_3("execution_next_index"));
      data.setConcurrencyEnabled(ser.read_type_7("concurrency_enabled"));
      data.setNumberOfExecutingActions(ser.read_type_9("number_of_executing_actions"));
      data.setNumberOfFailedActions(ser.read_type_9("number_of_failed_actions"));
      ser.read_type_f("executing_action_type", data.getExecutingActionType());
      ser.read_type_f("executing_action_id", data.getExecutingActionId());
      ser.read_type_f("elapsed_execution_time", data.getElapsedExecutionTime());
      ser.read_type_f("current_hand_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getCurrentHandPose());
      ser.read_type_f("goal_hand_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getGoalHandPose());
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
