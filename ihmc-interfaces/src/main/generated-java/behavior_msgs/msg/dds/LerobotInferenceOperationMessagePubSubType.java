package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "LerobotInferenceOperationMessage" defined in "LerobotInferenceOperationMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from LerobotInferenceOperationMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit LerobotInferenceOperationMessage_.idl instead.
*
*/
public class LerobotInferenceOperationMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.LerobotInferenceOperationMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::LerobotInferenceOperationMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "5b21bef7345af3f80dbe8b3a7425c2fe6f028aef6971d319accd33df32f5abb5";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.LerobotInferenceOperationMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.LerobotInferenceOperationMessage data) throws java.io.IOException
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

      current_alignment += ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      for(int i0 = 0; i0 < (2); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);}
      for(int i0 = 0; i0 < (2); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.LerobotInferenceOperationMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.LerobotInferenceOperationMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType.getCdrSerializedSize(data.getLatestTimestampModifiable(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      for(int i0 = 0; i0 < data.getActionHandPoses().length; ++i0)
      {
              current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getActionHandPoses()[i0], current_alignment);
      }
      for(int i0 = 0; i0 < data.getActionForearmPoses().length; ++i0)
      {
              current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getActionForearmPoses()[i0], current_alignment);
      }
      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getPythonStatusMessage().length() + 1;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.LerobotInferenceOperationMessage data, us.ihmc.idl.CDR cdr)
   {
      ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType.write(data.getLatestTimestampModifiable(), cdr);
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_7(data.getRunning());

      cdr.write_type_7(data.getControlRobot());

      for(int i0 = 0; i0 < data.getActionHandPoses().length; ++i0)
      {
        	geometry_msgs.msg.dds.PosePubSubType.write(data.getActionHandPoses()[i0], cdr);		
      }

      for(int i0 = 0; i0 < data.getActionForearmPoses().length; ++i0)
      {
        	geometry_msgs.msg.dds.PosePubSubType.write(data.getActionForearmPoses()[i0], cdr);		
      }

      cdr.write_type_6(data.getPythonStatusFrequency());

      if(data.getPythonStatusMessage().length() <= 255)
      cdr.write_type_d(data.getPythonStatusMessage());else
          throw new RuntimeException("python_status_message field exceeds the maximum length: %d > %d".formatted(data.getPythonStatusMessage().length(), 255));

      cdr.write_type_4(data.getReceivedActions());

   }

   public static void read(behavior_msgs.msg.dds.LerobotInferenceOperationMessage data, us.ihmc.idl.CDR cdr)
   {
      ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType.read(data.getLatestTimestampModifiable(), cdr);	
      data.setSequenceId(cdr.read_type_4());
      	
      data.setRunning(cdr.read_type_7());
      	
      data.setControlRobot(cdr.read_type_7());
      	
      for(int i0 = 0; i0 < data.getActionHandPoses().length; ++i0)
      {
        	geometry_msgs.msg.dds.PosePubSubType.read(data.getActionHandPoses()[i0], cdr);	
      }
      	
      for(int i0 = 0; i0 < data.getActionForearmPoses().length; ++i0)
      {
        	geometry_msgs.msg.dds.PosePubSubType.read(data.getActionForearmPoses()[i0], cdr);	
      }
      	
      data.setPythonStatusFrequency(cdr.read_type_6());
      	
      cdr.read_type_d(data.getPythonStatusMessage());	
      data.setReceivedActions(cdr.read_type_4());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.LerobotInferenceOperationMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("latest_timestamp_modifiable", new ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType(), data.getLatestTimestampModifiable());

      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_7("running", data.getRunning());
      ser.write_type_7("control_robot", data.getControlRobot());
      ser.write_type_f("action_hand_poses", new geometry_msgs.msg.dds.PosePubSubType(), data.getActionHandPoses());
      ser.write_type_f("action_forearm_poses", new geometry_msgs.msg.dds.PosePubSubType(), data.getActionForearmPoses());
      ser.write_type_6("python_status_frequency", data.getPythonStatusFrequency());
      ser.write_type_d("python_status_message", data.getPythonStatusMessage());
      ser.write_type_4("received_actions", data.getReceivedActions());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.LerobotInferenceOperationMessage data)
   {
      ser.read_type_a("latest_timestamp_modifiable", new ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType(), data.getLatestTimestampModifiable());

      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setRunning(ser.read_type_7("running"));
      data.setControlRobot(ser.read_type_7("control_robot"));
      ser.read_type_f("action_hand_poses", new geometry_msgs.msg.dds.PosePubSubType(), data.getActionHandPoses());
      ser.read_type_f("action_forearm_poses", new geometry_msgs.msg.dds.PosePubSubType(), data.getActionForearmPoses());
      data.setPythonStatusFrequency(ser.read_type_6("python_status_frequency"));
      ser.read_type_d("python_status_message", data.getPythonStatusMessage());
      data.setReceivedActions(ser.read_type_4("received_actions"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.LerobotInferenceOperationMessage src, behavior_msgs.msg.dds.LerobotInferenceOperationMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.LerobotInferenceOperationMessage createData()
   {
      return new behavior_msgs.msg.dds.LerobotInferenceOperationMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.LerobotInferenceOperationMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.LerobotInferenceOperationMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.LerobotInferenceOperationMessage src, behavior_msgs.msg.dds.LerobotInferenceOperationMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public LerobotInferenceOperationMessagePubSubType newInstance()
   {
      return new LerobotInferenceOperationMessagePubSubType();
   }
}
