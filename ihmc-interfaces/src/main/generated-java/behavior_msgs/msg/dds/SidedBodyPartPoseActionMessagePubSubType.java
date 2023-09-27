package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "SidedBodyPartPoseActionMessage" defined in "SidedBodyPartPoseActionMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from SidedBodyPartPoseActionMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit SidedBodyPartPoseActionMessage_.idl instead.
*
*/
public class SidedBodyPartPoseActionMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.SidedBodyPartPoseActionMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::SidedBodyPartPoseActionMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "897a1ce864084d2dfe87c832238a284de594330b4fc93d18dad9ffefc4a1d888";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.SidedBodyPartPoseActionMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.SidedBodyPartPoseActionMessage data) throws java.io.IOException
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

      current_alignment += behavior_msgs.msg.dds.ActionInformationMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 1000; ++i0)
      {
        current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      }
      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.SidedBodyPartPoseActionMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.SidedBodyPartPoseActionMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.ActionInformationMessagePubSubType.getCdrSerializedSize(data.getActionInformation(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getParentFrame().size(); ++i0)
      {
          current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getParentFrame().get(i0).length() + 1;
      }
      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getCdrSerializedSize(data.getTransformToParent(), current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.SidedBodyPartPoseActionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionInformationMessagePubSubType.write(data.getActionInformation(), cdr);
      cdr.write_type_9(data.getRobotSide());

      if(data.getParentFrame().size() <= 1000)
      cdr.write_type_e(data.getParentFrame());else
          throw new RuntimeException("parent_frame field exceeds the maximum length");

      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.write(data.getTransformToParent(), cdr);
      cdr.write_type_6(data.getTrajectoryDuration());

      cdr.write_type_7(data.getExecuteWithNextAction());

      cdr.write_type_7(data.getHoldPoseInWorld());

      cdr.write_type_7(data.getJointSpaceControl());

   }

   public static void read(behavior_msgs.msg.dds.SidedBodyPartPoseActionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionInformationMessagePubSubType.read(data.getActionInformation(), cdr);	
      data.setRobotSide(cdr.read_type_9());
      	
      cdr.read_type_e(data.getParentFrame());	
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.read(data.getTransformToParent(), cdr);	
      data.setTrajectoryDuration(cdr.read_type_6());
      	
      data.setExecuteWithNextAction(cdr.read_type_7());
      	
      data.setHoldPoseInWorld(cdr.read_type_7());
      	
      data.setJointSpaceControl(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.SidedBodyPartPoseActionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("action_information", new behavior_msgs.msg.dds.ActionInformationMessagePubSubType(), data.getActionInformation());

      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_e("parent_frame", data.getParentFrame());
      ser.write_type_a("transform_to_parent", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getTransformToParent());

      ser.write_type_6("trajectory_duration", data.getTrajectoryDuration());
      ser.write_type_7("execute_with_next_action", data.getExecuteWithNextAction());
      ser.write_type_7("hold_pose_in_world", data.getHoldPoseInWorld());
      ser.write_type_7("joint_space_control", data.getJointSpaceControl());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.SidedBodyPartPoseActionMessage data)
   {
      ser.read_type_a("action_information", new behavior_msgs.msg.dds.ActionInformationMessagePubSubType(), data.getActionInformation());

      data.setRobotSide(ser.read_type_9("robot_side"));
      ser.read_type_e("parent_frame", data.getParentFrame());
      ser.read_type_a("transform_to_parent", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getTransformToParent());

      data.setTrajectoryDuration(ser.read_type_6("trajectory_duration"));
      data.setExecuteWithNextAction(ser.read_type_7("execute_with_next_action"));
      data.setHoldPoseInWorld(ser.read_type_7("hold_pose_in_world"));
      data.setJointSpaceControl(ser.read_type_7("joint_space_control"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.SidedBodyPartPoseActionMessage src, behavior_msgs.msg.dds.SidedBodyPartPoseActionMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.SidedBodyPartPoseActionMessage createData()
   {
      return new behavior_msgs.msg.dds.SidedBodyPartPoseActionMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.SidedBodyPartPoseActionMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.SidedBodyPartPoseActionMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.SidedBodyPartPoseActionMessage src, behavior_msgs.msg.dds.SidedBodyPartPoseActionMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public SidedBodyPartPoseActionMessagePubSubType newInstance()
   {
      return new SidedBodyPartPoseActionMessagePubSubType();
   }
}
