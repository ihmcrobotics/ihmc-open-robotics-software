package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "WalkActionDefinitionMessage" defined in "WalkActionDefinitionMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from WalkActionDefinitionMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit WalkActionDefinitionMessage_.idl instead.
*
*/
public class WalkActionDefinitionMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.WalkActionDefinitionMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::WalkActionDefinitionMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "280815d3ffb6b18c3e85d230511a673bd5b22275ab5a9add4cf8d2b11f8db3e5";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.WalkActionDefinitionMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.WalkActionDefinitionMessage data) throws java.io.IOException
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

      current_alignment += behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.WalkActionDefinitionMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.WalkActionDefinitionMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.getCdrSerializedSize(data.getDefinition(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getParentFrameName().length() + 1;

      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getCdrSerializedSize(data.getTransformToParent(), current_alignment);

      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getCdrSerializedSize(data.getLeftGoalFootTransformToGizmo(), current_alignment);

      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getCdrSerializedSize(data.getRightGoalFootTransformToGizmo(), current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.WalkActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.write(data.getDefinition(), cdr);
      if(data.getParentFrameName().length() <= 255)
      cdr.write_type_d(data.getParentFrameName());else
          throw new RuntimeException("parent_frame_name field exceeds the maximum length");

      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.write(data.getTransformToParent(), cdr);
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.write(data.getLeftGoalFootTransformToGizmo(), cdr);
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.write(data.getRightGoalFootTransformToGizmo(), cdr);
      cdr.write_type_6(data.getSwingDuration());

      cdr.write_type_6(data.getTransferDuration());

   }

   public static void read(behavior_msgs.msg.dds.WalkActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.read(data.getDefinition(), cdr);	
      cdr.read_type_d(data.getParentFrameName());	
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.read(data.getTransformToParent(), cdr);	
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.read(data.getLeftGoalFootTransformToGizmo(), cdr);	
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.read(data.getRightGoalFootTransformToGizmo(), cdr);	
      data.setSwingDuration(cdr.read_type_6());
      	
      data.setTransferDuration(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.WalkActionDefinitionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("definition", new behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType(), data.getDefinition());

      ser.write_type_d("parent_frame_name", data.getParentFrameName());
      ser.write_type_a("transform_to_parent", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getTransformToParent());

      ser.write_type_a("left_goal_foot_transform_to_gizmo", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getLeftGoalFootTransformToGizmo());

      ser.write_type_a("right_goal_foot_transform_to_gizmo", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getRightGoalFootTransformToGizmo());

      ser.write_type_6("swing_duration", data.getSwingDuration());
      ser.write_type_6("transfer_duration", data.getTransferDuration());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.WalkActionDefinitionMessage data)
   {
      ser.read_type_a("definition", new behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType(), data.getDefinition());

      ser.read_type_d("parent_frame_name", data.getParentFrameName());
      ser.read_type_a("transform_to_parent", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getTransformToParent());

      ser.read_type_a("left_goal_foot_transform_to_gizmo", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getLeftGoalFootTransformToGizmo());

      ser.read_type_a("right_goal_foot_transform_to_gizmo", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getRightGoalFootTransformToGizmo());

      data.setSwingDuration(ser.read_type_6("swing_duration"));
      data.setTransferDuration(ser.read_type_6("transfer_duration"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.WalkActionDefinitionMessage src, behavior_msgs.msg.dds.WalkActionDefinitionMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.WalkActionDefinitionMessage createData()
   {
      return new behavior_msgs.msg.dds.WalkActionDefinitionMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.WalkActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.WalkActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.WalkActionDefinitionMessage src, behavior_msgs.msg.dds.WalkActionDefinitionMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public WalkActionDefinitionMessagePubSubType newInstance()
   {
      return new WalkActionDefinitionMessagePubSubType();
   }
}
