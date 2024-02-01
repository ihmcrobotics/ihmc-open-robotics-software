package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "KinematicsStreamingToolboxConfigurationMessage" defined in "KinematicsStreamingToolboxConfigurationMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from KinematicsStreamingToolboxConfigurationMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit KinematicsStreamingToolboxConfigurationMessage_.idl instead.
*
*/
public class KinematicsStreamingToolboxConfigurationMessagePubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.KinematicsStreamingToolboxConfigurationMessage>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::KinematicsStreamingToolboxConfigurationMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "adbb80bba9ebc6c0eaf0144915b2372c04de0e848973c03fde8e09a0dc1fe236";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.KinematicsStreamingToolboxConfigurationMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.KinematicsStreamingToolboxConfigurationMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.KinematicsStreamingToolboxConfigurationMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.KinematicsStreamingToolboxConfigurationMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.KinematicsStreamingToolboxConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_7(data.getLockPelvis());

      cdr.write_type_7(data.getLockChest());

      cdr.write_type_7(data.getEnableLeftArmJointspace());

      cdr.write_type_7(data.getEnableRightArmJointspace());

      cdr.write_type_7(data.getEnableNeckJointspace());

      cdr.write_type_7(data.getEnableLeftHandTaskspace());

      cdr.write_type_7(data.getEnableRightHandTaskspace());

      cdr.write_type_7(data.getEnableChestTaskspace());

      cdr.write_type_7(data.getEnablePelvisTaskspace());

      cdr.write_type_11(data.getLeftHandTrajectoryFrameId());

      cdr.write_type_11(data.getRightHandTrajectoryFrameId());

      cdr.write_type_11(data.getChestTrajectoryFrameId());

      cdr.write_type_11(data.getPelvisTrajectoryFrameId());

   }

   public static void read(toolbox_msgs.msg.dds.KinematicsStreamingToolboxConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setLockPelvis(cdr.read_type_7());
      	
      data.setLockChest(cdr.read_type_7());
      	
      data.setEnableLeftArmJointspace(cdr.read_type_7());
      	
      data.setEnableRightArmJointspace(cdr.read_type_7());
      	
      data.setEnableNeckJointspace(cdr.read_type_7());
      	
      data.setEnableLeftHandTaskspace(cdr.read_type_7());
      	
      data.setEnableRightHandTaskspace(cdr.read_type_7());
      	
      data.setEnableChestTaskspace(cdr.read_type_7());
      	
      data.setEnablePelvisTaskspace(cdr.read_type_7());
      	
      data.setLeftHandTrajectoryFrameId(cdr.read_type_11());
      	
      data.setRightHandTrajectoryFrameId(cdr.read_type_11());
      	
      data.setChestTrajectoryFrameId(cdr.read_type_11());
      	
      data.setPelvisTrajectoryFrameId(cdr.read_type_11());
      	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.KinematicsStreamingToolboxConfigurationMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_7("lock_pelvis", data.getLockPelvis());
      ser.write_type_7("lock_chest", data.getLockChest());
      ser.write_type_7("enable_left_arm_jointspace", data.getEnableLeftArmJointspace());
      ser.write_type_7("enable_right_arm_jointspace", data.getEnableRightArmJointspace());
      ser.write_type_7("enable_neck_jointspace", data.getEnableNeckJointspace());
      ser.write_type_7("enable_left_hand_taskspace", data.getEnableLeftHandTaskspace());
      ser.write_type_7("enable_right_hand_taskspace", data.getEnableRightHandTaskspace());
      ser.write_type_7("enable_chest_taskspace", data.getEnableChestTaskspace());
      ser.write_type_7("enable_pelvis_taskspace", data.getEnablePelvisTaskspace());
      ser.write_type_11("left_hand_trajectory_frame_id", data.getLeftHandTrajectoryFrameId());
      ser.write_type_11("right_hand_trajectory_frame_id", data.getRightHandTrajectoryFrameId());
      ser.write_type_11("chest_trajectory_frame_id", data.getChestTrajectoryFrameId());
      ser.write_type_11("pelvis_trajectory_frame_id", data.getPelvisTrajectoryFrameId());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.KinematicsStreamingToolboxConfigurationMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setLockPelvis(ser.read_type_7("lock_pelvis"));
      data.setLockChest(ser.read_type_7("lock_chest"));
      data.setEnableLeftArmJointspace(ser.read_type_7("enable_left_arm_jointspace"));
      data.setEnableRightArmJointspace(ser.read_type_7("enable_right_arm_jointspace"));
      data.setEnableNeckJointspace(ser.read_type_7("enable_neck_jointspace"));
      data.setEnableLeftHandTaskspace(ser.read_type_7("enable_left_hand_taskspace"));
      data.setEnableRightHandTaskspace(ser.read_type_7("enable_right_hand_taskspace"));
      data.setEnableChestTaskspace(ser.read_type_7("enable_chest_taskspace"));
      data.setEnablePelvisTaskspace(ser.read_type_7("enable_pelvis_taskspace"));
      data.setLeftHandTrajectoryFrameId(ser.read_type_11("left_hand_trajectory_frame_id"));
      data.setRightHandTrajectoryFrameId(ser.read_type_11("right_hand_trajectory_frame_id"));
      data.setChestTrajectoryFrameId(ser.read_type_11("chest_trajectory_frame_id"));
      data.setPelvisTrajectoryFrameId(ser.read_type_11("pelvis_trajectory_frame_id"));
   }

   public static void staticCopy(toolbox_msgs.msg.dds.KinematicsStreamingToolboxConfigurationMessage src, toolbox_msgs.msg.dds.KinematicsStreamingToolboxConfigurationMessage dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.KinematicsStreamingToolboxConfigurationMessage createData()
   {
      return new toolbox_msgs.msg.dds.KinematicsStreamingToolboxConfigurationMessage();
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
   
   public void serialize(toolbox_msgs.msg.dds.KinematicsStreamingToolboxConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.KinematicsStreamingToolboxConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.KinematicsStreamingToolboxConfigurationMessage src, toolbox_msgs.msg.dds.KinematicsStreamingToolboxConfigurationMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public KinematicsStreamingToolboxConfigurationMessagePubSubType newInstance()
   {
      return new KinematicsStreamingToolboxConfigurationMessagePubSubType();
   }
}
