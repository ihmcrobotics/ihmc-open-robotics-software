package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ConditionNodeDefinitionMessage" defined in "ConditionNodeDefinitionMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ConditionNodeDefinitionMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ConditionNodeDefinitionMessage_.idl instead.
*
*/
public class ConditionNodeDefinitionMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.ConditionNodeDefinitionMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::ConditionNodeDefinitionMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "0ce56a844cee9c1b89456875b9e6b5b5184b9ccd4d0e6b2d0225e5654a8d5850";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.ConditionNodeDefinitionMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.ConditionNodeDefinitionMessage data) throws java.io.IOException
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

      current_alignment += behavior_msgs.msg.dds.LeafNodeDefinitionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 10000 + 1;
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 10000 + 1;
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 10000 + 1;
      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.ConditionNodeDefinitionMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.ConditionNodeDefinitionMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.LeafNodeDefinitionMessagePubSubType.getCdrSerializedSize(data.getDefinition(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getSystem().length() + 1;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getPrompt().length() + 1;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getResponseMatcher().length() + 1;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getFrameNameA().length() + 1;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getFrameNameB().length() + 1;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getShapeParentFrameName().length() + 1;

      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getCdrSerializedSize(data.getShapeTransformToParent(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getFrameName().length() + 1;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.ConditionNodeDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.LeafNodeDefinitionMessagePubSubType.write(data.getDefinition(), cdr);
      cdr.write_type_9(data.getType());

      cdr.write_type_4(data.getCountTo());

      cdr.write_type_7(data.getResetContextEachRun());

      cdr.write_type_7(data.getInjectBehaviorState());

      cdr.write_type_7(data.getInjectEnvironmentState());

      cdr.write_type_7(data.getMatchIsSuccess());

      if(data.getSystem().length() <= 10000)
      cdr.write_type_d(data.getSystem());else
          throw new RuntimeException("system field exceeds the maximum length: %d > %d".formatted(data.getSystem().length(), 10000));

      if(data.getPrompt().length() <= 10000)
      cdr.write_type_d(data.getPrompt());else
          throw new RuntimeException("prompt field exceeds the maximum length: %d > %d".formatted(data.getPrompt().length(), 10000));

      if(data.getResponseMatcher().length() <= 10000)
      cdr.write_type_d(data.getResponseMatcher());else
          throw new RuntimeException("response_matcher field exceeds the maximum length: %d > %d".formatted(data.getResponseMatcher().length(), 10000));

      cdr.write_type_9(data.getDistanceType());

      if(data.getFrameNameA().length() <= 255)
      cdr.write_type_d(data.getFrameNameA());else
          throw new RuntimeException("frame_name_a field exceeds the maximum length: %d > %d".formatted(data.getFrameNameA().length(), 255));

      if(data.getFrameNameB().length() <= 255)
      cdr.write_type_d(data.getFrameNameB());else
          throw new RuntimeException("frame_name_b field exceeds the maximum length: %d > %d".formatted(data.getFrameNameB().length(), 255));

      cdr.write_type_6(data.getMinDistance());

      cdr.write_type_6(data.getMaxDistance());

      cdr.write_type_6(data.getTimeout());

      cdr.write_type_9(data.getContainsType());

      if(data.getShapeParentFrameName().length() <= 255)
      cdr.write_type_d(data.getShapeParentFrameName());else
          throw new RuntimeException("shape_parent_frame_name field exceeds the maximum length: %d > %d".formatted(data.getShapeParentFrameName().length(), 255));

      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.write(data.getShapeTransformToParent(), cdr);
      cdr.write_type_5(data.getSphereRadius());

      if(data.getFrameName().length() <= 255)
      cdr.write_type_d(data.getFrameName());else
          throw new RuntimeException("frame_name field exceeds the maximum length: %d > %d".formatted(data.getFrameName().length(), 255));

      cdr.write_type_4(data.getMinPoints());

   }

   public static void read(behavior_msgs.msg.dds.ConditionNodeDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.LeafNodeDefinitionMessagePubSubType.read(data.getDefinition(), cdr);	
      data.setType(cdr.read_type_9());
      	
      data.setCountTo(cdr.read_type_4());
      	
      data.setResetContextEachRun(cdr.read_type_7());
      	
      data.setInjectBehaviorState(cdr.read_type_7());
      	
      data.setInjectEnvironmentState(cdr.read_type_7());
      	
      data.setMatchIsSuccess(cdr.read_type_7());
      	
      cdr.read_type_d(data.getSystem());	
      cdr.read_type_d(data.getPrompt());	
      cdr.read_type_d(data.getResponseMatcher());	
      data.setDistanceType(cdr.read_type_9());
      	
      cdr.read_type_d(data.getFrameNameA());	
      cdr.read_type_d(data.getFrameNameB());	
      data.setMinDistance(cdr.read_type_6());
      	
      data.setMaxDistance(cdr.read_type_6());
      	
      data.setTimeout(cdr.read_type_6());
      	
      data.setContainsType(cdr.read_type_9());
      	
      cdr.read_type_d(data.getShapeParentFrameName());	
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.read(data.getShapeTransformToParent(), cdr);	
      data.setSphereRadius(cdr.read_type_5());
      	
      cdr.read_type_d(data.getFrameName());	
      data.setMinPoints(cdr.read_type_4());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.ConditionNodeDefinitionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("definition", new behavior_msgs.msg.dds.LeafNodeDefinitionMessagePubSubType(), data.getDefinition());

      ser.write_type_9("type", data.getType());
      ser.write_type_4("count_to", data.getCountTo());
      ser.write_type_7("reset_context_each_run", data.getResetContextEachRun());
      ser.write_type_7("inject_behavior_state", data.getInjectBehaviorState());
      ser.write_type_7("inject_environment_state", data.getInjectEnvironmentState());
      ser.write_type_7("match_is_success", data.getMatchIsSuccess());
      ser.write_type_d("system", data.getSystem());
      ser.write_type_d("prompt", data.getPrompt());
      ser.write_type_d("response_matcher", data.getResponseMatcher());
      ser.write_type_9("distance_type", data.getDistanceType());
      ser.write_type_d("frame_name_a", data.getFrameNameA());
      ser.write_type_d("frame_name_b", data.getFrameNameB());
      ser.write_type_6("min_distance", data.getMinDistance());
      ser.write_type_6("max_distance", data.getMaxDistance());
      ser.write_type_6("timeout", data.getTimeout());
      ser.write_type_9("contains_type", data.getContainsType());
      ser.write_type_d("shape_parent_frame_name", data.getShapeParentFrameName());
      ser.write_type_a("shape_transform_to_parent", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getShapeTransformToParent());

      ser.write_type_5("sphere_radius", data.getSphereRadius());
      ser.write_type_d("frame_name", data.getFrameName());
      ser.write_type_4("min_points", data.getMinPoints());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.ConditionNodeDefinitionMessage data)
   {
      ser.read_type_a("definition", new behavior_msgs.msg.dds.LeafNodeDefinitionMessagePubSubType(), data.getDefinition());

      data.setType(ser.read_type_9("type"));
      data.setCountTo(ser.read_type_4("count_to"));
      data.setResetContextEachRun(ser.read_type_7("reset_context_each_run"));
      data.setInjectBehaviorState(ser.read_type_7("inject_behavior_state"));
      data.setInjectEnvironmentState(ser.read_type_7("inject_environment_state"));
      data.setMatchIsSuccess(ser.read_type_7("match_is_success"));
      ser.read_type_d("system", data.getSystem());
      ser.read_type_d("prompt", data.getPrompt());
      ser.read_type_d("response_matcher", data.getResponseMatcher());
      data.setDistanceType(ser.read_type_9("distance_type"));
      ser.read_type_d("frame_name_a", data.getFrameNameA());
      ser.read_type_d("frame_name_b", data.getFrameNameB());
      data.setMinDistance(ser.read_type_6("min_distance"));
      data.setMaxDistance(ser.read_type_6("max_distance"));
      data.setTimeout(ser.read_type_6("timeout"));
      data.setContainsType(ser.read_type_9("contains_type"));
      ser.read_type_d("shape_parent_frame_name", data.getShapeParentFrameName());
      ser.read_type_a("shape_transform_to_parent", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getShapeTransformToParent());

      data.setSphereRadius(ser.read_type_5("sphere_radius"));
      ser.read_type_d("frame_name", data.getFrameName());
      data.setMinPoints(ser.read_type_4("min_points"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.ConditionNodeDefinitionMessage src, behavior_msgs.msg.dds.ConditionNodeDefinitionMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.ConditionNodeDefinitionMessage createData()
   {
      return new behavior_msgs.msg.dds.ConditionNodeDefinitionMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.ConditionNodeDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.ConditionNodeDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.ConditionNodeDefinitionMessage src, behavior_msgs.msg.dds.ConditionNodeDefinitionMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ConditionNodeDefinitionMessagePubSubType newInstance()
   {
      return new ConditionNodeDefinitionMessagePubSubType();
   }
}
