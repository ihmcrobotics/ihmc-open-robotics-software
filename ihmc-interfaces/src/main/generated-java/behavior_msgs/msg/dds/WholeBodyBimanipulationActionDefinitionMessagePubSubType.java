package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "WholeBodyBimanipulationActionDefinitionMessage" defined in "WholeBodyBimanipulationActionDefinitionMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from WholeBodyBimanipulationActionDefinitionMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit WholeBodyBimanipulationActionDefinitionMessage_.idl instead.
*
*/
public class WholeBodyBimanipulationActionDefinitionMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.WholeBodyBimanipulationActionDefinitionMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::WholeBodyBimanipulationActionDefinitionMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "4abd4ac5494127410d2cec088f38c45dcd48053952742db1ec944be2962ea0ea";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.WholeBodyBimanipulationActionDefinitionMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.WholeBodyBimanipulationActionDefinitionMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.WholeBodyBimanipulationActionDefinitionMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.WholeBodyBimanipulationActionDefinitionMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.getCdrSerializedSize(data.getDefinition(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getParentFrameName().length() + 1;

      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getCdrSerializedSize(data.getRightHandTransformToParent(), current_alignment);

      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getCdrSerializedSize(data.getLeftHandTransformToParent(), current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.WholeBodyBimanipulationActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.write(data.getDefinition(), cdr);
      if(data.getParentFrameName().length() <= 255)
      cdr.write_type_d(data.getParentFrameName());else
          throw new RuntimeException("parent_frame_name field exceeds the maximum length");

      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.write(data.getRightHandTransformToParent(), cdr);
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.write(data.getLeftHandTransformToParent(), cdr);
      cdr.write_type_6(data.getTrajectoryDuration());

      cdr.write_type_7(data.getHoldPoseInWorld());

   }

   public static void read(behavior_msgs.msg.dds.WholeBodyBimanipulationActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.read(data.getDefinition(), cdr);	
      cdr.read_type_d(data.getParentFrameName());	
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.read(data.getRightHandTransformToParent(), cdr);	
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.read(data.getLeftHandTransformToParent(), cdr);	
      data.setTrajectoryDuration(cdr.read_type_6());
      	
      data.setHoldPoseInWorld(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.WholeBodyBimanipulationActionDefinitionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("definition", new behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType(), data.getDefinition());

      ser.write_type_d("parent_frame_name", data.getParentFrameName());
      ser.write_type_a("right_hand_transform_to_parent", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getRightHandTransformToParent());

      ser.write_type_a("left_hand_transform_to_parent", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getLeftHandTransformToParent());

      ser.write_type_6("trajectory_duration", data.getTrajectoryDuration());
      ser.write_type_7("hold_pose_in_world", data.getHoldPoseInWorld());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.WholeBodyBimanipulationActionDefinitionMessage data)
   {
      ser.read_type_a("definition", new behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType(), data.getDefinition());

      ser.read_type_d("parent_frame_name", data.getParentFrameName());
      ser.read_type_a("right_hand_transform_to_parent", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getRightHandTransformToParent());

      ser.read_type_a("left_hand_transform_to_parent", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getLeftHandTransformToParent());

      data.setTrajectoryDuration(ser.read_type_6("trajectory_duration"));
      data.setHoldPoseInWorld(ser.read_type_7("hold_pose_in_world"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.WholeBodyBimanipulationActionDefinitionMessage src, behavior_msgs.msg.dds.WholeBodyBimanipulationActionDefinitionMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.WholeBodyBimanipulationActionDefinitionMessage createData()
   {
      return new behavior_msgs.msg.dds.WholeBodyBimanipulationActionDefinitionMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.WholeBodyBimanipulationActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.WholeBodyBimanipulationActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.WholeBodyBimanipulationActionDefinitionMessage src, behavior_msgs.msg.dds.WholeBodyBimanipulationActionDefinitionMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public WholeBodyBimanipulationActionDefinitionMessagePubSubType newInstance()
   {
      return new WholeBodyBimanipulationActionDefinitionMessagePubSubType();
   }
}
