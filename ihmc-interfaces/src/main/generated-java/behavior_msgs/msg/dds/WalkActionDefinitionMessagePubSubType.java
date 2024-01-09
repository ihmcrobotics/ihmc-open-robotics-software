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
   		return "3a945217f1fc7c0a0c6b102534d074cb006ff373548e526a085aa456e8c761e5";
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

      current_alignment += behavior_msgs.msg.dds.FootstepPlanActionDefinitionBasicsMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


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

      current_alignment += behavior_msgs.msg.dds.FootstepPlanActionDefinitionBasicsMessagePubSubType.getCdrSerializedSize(data.getDefinitionBasics(), current_alignment);

      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getCdrSerializedSize(data.getTransformToParent(), current_alignment);

      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getCdrSerializedSize(data.getLeftGoalFootTransformToGizmo(), current_alignment);

      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getCdrSerializedSize(data.getRightGoalFootTransformToGizmo(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.WalkActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.write(data.getDefinition(), cdr);
      behavior_msgs.msg.dds.FootstepPlanActionDefinitionBasicsMessagePubSubType.write(data.getDefinitionBasics(), cdr);
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.write(data.getTransformToParent(), cdr);
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.write(data.getLeftGoalFootTransformToGizmo(), cdr);
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.write(data.getRightGoalFootTransformToGizmo(), cdr);
   }

   public static void read(behavior_msgs.msg.dds.WalkActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.read(data.getDefinition(), cdr);	
      behavior_msgs.msg.dds.FootstepPlanActionDefinitionBasicsMessagePubSubType.read(data.getDefinitionBasics(), cdr);	
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.read(data.getTransformToParent(), cdr);	
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.read(data.getLeftGoalFootTransformToGizmo(), cdr);	
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.read(data.getRightGoalFootTransformToGizmo(), cdr);	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.WalkActionDefinitionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("definition", new behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType(), data.getDefinition());

      ser.write_type_a("definition_basics", new behavior_msgs.msg.dds.FootstepPlanActionDefinitionBasicsMessagePubSubType(), data.getDefinitionBasics());

      ser.write_type_a("transform_to_parent", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getTransformToParent());

      ser.write_type_a("left_goal_foot_transform_to_gizmo", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getLeftGoalFootTransformToGizmo());

      ser.write_type_a("right_goal_foot_transform_to_gizmo", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getRightGoalFootTransformToGizmo());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.WalkActionDefinitionMessage data)
   {
      ser.read_type_a("definition", new behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType(), data.getDefinition());

      ser.read_type_a("definition_basics", new behavior_msgs.msg.dds.FootstepPlanActionDefinitionBasicsMessagePubSubType(), data.getDefinitionBasics());

      ser.read_type_a("transform_to_parent", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getTransformToParent());

      ser.read_type_a("left_goal_foot_transform_to_gizmo", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getLeftGoalFootTransformToGizmo());

      ser.read_type_a("right_goal_foot_transform_to_gizmo", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getRightGoalFootTransformToGizmo());

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
