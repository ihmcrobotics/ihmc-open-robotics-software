package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ChestOrientationActionDefinitionMessage" defined in "ChestOrientationActionDefinitionMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ChestOrientationActionDefinitionMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ChestOrientationActionDefinitionMessage_.idl instead.
*
*/
public class ChestOrientationActionDefinitionMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.ChestOrientationActionDefinitionMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::ChestOrientationActionDefinitionMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "9ce9e9a78b3514b9e18ac50b10d85dfa5e02cbbeea5bf26c82764a0eb179b9f5";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.ChestOrientationActionDefinitionMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.ChestOrientationActionDefinitionMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.ChestOrientationActionDefinitionMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.ChestOrientationActionDefinitionMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.getCdrSerializedSize(data.getDefinition(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getParentFrameName().length() + 1;

      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getCdrSerializedSize(data.getChestTransformToParent(), current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.ChestOrientationActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.write(data.getDefinition(), cdr);
      if(data.getParentFrameName().length() <= 255)
      cdr.write_type_d(data.getParentFrameName());else
          throw new RuntimeException("parent_frame_name field exceeds the maximum length");

      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.write(data.getChestTransformToParent(), cdr);
      cdr.write_type_6(data.getTrajectoryDuration());

      cdr.write_type_7(data.getHoldPoseInWorld());

   }

   public static void read(behavior_msgs.msg.dds.ChestOrientationActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.read(data.getDefinition(), cdr);	
      cdr.read_type_d(data.getParentFrameName());	
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.read(data.getChestTransformToParent(), cdr);	
      data.setTrajectoryDuration(cdr.read_type_6());
      	
      data.setHoldPoseInWorld(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.ChestOrientationActionDefinitionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("definition", new behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType(), data.getDefinition());

      ser.write_type_d("parent_frame_name", data.getParentFrameName());
      ser.write_type_a("chest_transform_to_parent", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getChestTransformToParent());

      ser.write_type_6("trajectory_duration", data.getTrajectoryDuration());
      ser.write_type_7("hold_pose_in_world", data.getHoldPoseInWorld());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.ChestOrientationActionDefinitionMessage data)
   {
      ser.read_type_a("definition", new behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType(), data.getDefinition());

      ser.read_type_d("parent_frame_name", data.getParentFrameName());
      ser.read_type_a("chest_transform_to_parent", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getChestTransformToParent());

      data.setTrajectoryDuration(ser.read_type_6("trajectory_duration"));
      data.setHoldPoseInWorld(ser.read_type_7("hold_pose_in_world"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.ChestOrientationActionDefinitionMessage src, behavior_msgs.msg.dds.ChestOrientationActionDefinitionMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.ChestOrientationActionDefinitionMessage createData()
   {
      return new behavior_msgs.msg.dds.ChestOrientationActionDefinitionMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.ChestOrientationActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.ChestOrientationActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.ChestOrientationActionDefinitionMessage src, behavior_msgs.msg.dds.ChestOrientationActionDefinitionMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ChestOrientationActionDefinitionMessagePubSubType newInstance()
   {
      return new ChestOrientationActionDefinitionMessagePubSubType();
   }
}
