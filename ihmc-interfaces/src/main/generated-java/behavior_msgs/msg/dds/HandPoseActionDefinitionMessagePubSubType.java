package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "HandPoseActionDefinitionMessage" defined in "HandPoseActionDefinitionMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from HandPoseActionDefinitionMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit HandPoseActionDefinitionMessage_.idl instead.
*
*/
public class HandPoseActionDefinitionMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.HandPoseActionDefinitionMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::HandPoseActionDefinitionMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "23eb3e2ef86916acee43dd5ed069ca71bdeaaea0ce8810960567d8cb1b52a7bd";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.HandPoseActionDefinitionMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.HandPoseActionDefinitionMessage data) throws java.io.IOException
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

      current_alignment += behavior_msgs.msg.dds.BehaviorActionDefinitionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.HandPoseActionDefinitionMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.HandPoseActionDefinitionMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.BehaviorActionDefinitionMessagePubSubType.getCdrSerializedSize(data.getActionDefinition(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getParentFrameName().length() + 1;

      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getCdrSerializedSize(data.getTransformToParent(), current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.HandPoseActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.BehaviorActionDefinitionMessagePubSubType.write(data.getActionDefinition(), cdr);
      cdr.write_type_9(data.getRobotSide());

      if(data.getParentFrameName().length() <= 255)
      cdr.write_type_d(data.getParentFrameName());else
          throw new RuntimeException("parent_frame_name field exceeds the maximum length");

      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.write(data.getTransformToParent(), cdr);
      cdr.write_type_6(data.getTrajectoryDuration());

      cdr.write_type_7(data.getExecuteWithNextAction());

      cdr.write_type_7(data.getHoldPoseInWorld());

      cdr.write_type_7(data.getJointSpaceControl());

   }

   public static void read(behavior_msgs.msg.dds.HandPoseActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.BehaviorActionDefinitionMessagePubSubType.read(data.getActionDefinition(), cdr);	
      data.setRobotSide(cdr.read_type_9());
      	
      cdr.read_type_d(data.getParentFrameName());	
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.read(data.getTransformToParent(), cdr);	
      data.setTrajectoryDuration(cdr.read_type_6());
      	
      data.setExecuteWithNextAction(cdr.read_type_7());
      	
      data.setHoldPoseInWorld(cdr.read_type_7());
      	
      data.setJointSpaceControl(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.HandPoseActionDefinitionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("action_definition", new behavior_msgs.msg.dds.BehaviorActionDefinitionMessagePubSubType(), data.getActionDefinition());

      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_d("parent_frame_name", data.getParentFrameName());
      ser.write_type_a("transform_to_parent", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getTransformToParent());

      ser.write_type_6("trajectory_duration", data.getTrajectoryDuration());
      ser.write_type_7("execute_with_next_action", data.getExecuteWithNextAction());
      ser.write_type_7("hold_pose_in_world", data.getHoldPoseInWorld());
      ser.write_type_7("joint_space_control", data.getJointSpaceControl());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.HandPoseActionDefinitionMessage data)
   {
      ser.read_type_a("action_definition", new behavior_msgs.msg.dds.BehaviorActionDefinitionMessagePubSubType(), data.getActionDefinition());

      data.setRobotSide(ser.read_type_9("robot_side"));
      ser.read_type_d("parent_frame_name", data.getParentFrameName());
      ser.read_type_a("transform_to_parent", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getTransformToParent());

      data.setTrajectoryDuration(ser.read_type_6("trajectory_duration"));
      data.setExecuteWithNextAction(ser.read_type_7("execute_with_next_action"));
      data.setHoldPoseInWorld(ser.read_type_7("hold_pose_in_world"));
      data.setJointSpaceControl(ser.read_type_7("joint_space_control"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.HandPoseActionDefinitionMessage src, behavior_msgs.msg.dds.HandPoseActionDefinitionMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.HandPoseActionDefinitionMessage createData()
   {
      return new behavior_msgs.msg.dds.HandPoseActionDefinitionMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.HandPoseActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.HandPoseActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.HandPoseActionDefinitionMessage src, behavior_msgs.msg.dds.HandPoseActionDefinitionMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public HandPoseActionDefinitionMessagePubSubType newInstance()
   {
      return new HandPoseActionDefinitionMessagePubSubType();
   }
}
