package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "VLAOperationMessage" defined in "VLAOperationMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from VLAOperationMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit VLAOperationMessage_.idl instead.
*
*/
public class VLAOperationMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.VLAOperationMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::VLAOperationMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "29bf149b6e1534f795f1ed5020b9966e318d866205e08af5a38a8c377e8a6676";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.VLAOperationMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.VLAOperationMessage data) throws java.io.IOException
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
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.VLAOperationMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.VLAOperationMessage data, int current_alignment)
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
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getStatusMessage().length() + 1;


      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.VLAOperationMessage data, us.ihmc.idl.CDR cdr)
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

      if(data.getStatusMessage().length() <= 255)
      cdr.write_type_d(data.getStatusMessage());else
          throw new RuntimeException("status_message field exceeds the maximum length: %d > %d".formatted(data.getStatusMessage().length(), 255));

   }

   public static void read(behavior_msgs.msg.dds.VLAOperationMessage data, us.ihmc.idl.CDR cdr)
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
      	
      cdr.read_type_d(data.getStatusMessage());	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.VLAOperationMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("latest_timestamp_modifiable", new ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType(), data.getLatestTimestampModifiable());

      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_7("running", data.getRunning());
      ser.write_type_7("control_robot", data.getControlRobot());
      ser.write_type_f("action_hand_poses", new geometry_msgs.msg.dds.PosePubSubType(), data.getActionHandPoses());
      ser.write_type_f("action_forearm_poses", new geometry_msgs.msg.dds.PosePubSubType(), data.getActionForearmPoses());
      ser.write_type_d("status_message", data.getStatusMessage());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.VLAOperationMessage data)
   {
      ser.read_type_a("latest_timestamp_modifiable", new ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType(), data.getLatestTimestampModifiable());

      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setRunning(ser.read_type_7("running"));
      data.setControlRobot(ser.read_type_7("control_robot"));
      ser.read_type_f("action_hand_poses", new geometry_msgs.msg.dds.PosePubSubType(), data.getActionHandPoses());
      ser.read_type_f("action_forearm_poses", new geometry_msgs.msg.dds.PosePubSubType(), data.getActionForearmPoses());
      ser.read_type_d("status_message", data.getStatusMessage());
   }

   public static void staticCopy(behavior_msgs.msg.dds.VLAOperationMessage src, behavior_msgs.msg.dds.VLAOperationMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.VLAOperationMessage createData()
   {
      return new behavior_msgs.msg.dds.VLAOperationMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.VLAOperationMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.VLAOperationMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.VLAOperationMessage src, behavior_msgs.msg.dds.VLAOperationMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public VLAOperationMessagePubSubType newInstance()
   {
      return new VLAOperationMessagePubSubType();
   }
}
