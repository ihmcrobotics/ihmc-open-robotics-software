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
   		return "edcdd24778cbcbf0ba3f08c03a670a7922f127532762cec256f03ff4dda4e5e6";
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

      current_alignment += behavior_msgs.msg.dds.BehaviorActionDefinitionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 1000; ++i0)
      {
        current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      }
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

      current_alignment += behavior_msgs.msg.dds.BehaviorActionDefinitionMessagePubSubType.getCdrSerializedSize(data.getActionDefinition(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getParentFrame().size(); ++i0)
      {
          current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getParentFrame().get(i0).length() + 1;
      }
      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getCdrSerializedSize(data.getChestTransformToParent(), current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.ChestOrientationActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.BehaviorActionDefinitionMessagePubSubType.write(data.getActionDefinition(), cdr);
      if(data.getParentFrame().size() <= 1000)
      cdr.write_type_e(data.getParentFrame());else
          throw new RuntimeException("parent_frame field exceeds the maximum length");

      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.write(data.getChestTransformToParent(), cdr);
      cdr.write_type_6(data.getTrajectoryDuration());

      cdr.write_type_7(data.getHoldPoseInWorld());

   }

   public static void read(behavior_msgs.msg.dds.ChestOrientationActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.BehaviorActionDefinitionMessagePubSubType.read(data.getActionDefinition(), cdr);	
      cdr.read_type_e(data.getParentFrame());	
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.read(data.getChestTransformToParent(), cdr);	
      data.setTrajectoryDuration(cdr.read_type_6());
      	
      data.setHoldPoseInWorld(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.ChestOrientationActionDefinitionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("action_definition", new behavior_msgs.msg.dds.BehaviorActionDefinitionMessagePubSubType(), data.getActionDefinition());

      ser.write_type_e("parent_frame", data.getParentFrame());
      ser.write_type_a("chest_transform_to_parent", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getChestTransformToParent());

      ser.write_type_6("trajectory_duration", data.getTrajectoryDuration());
      ser.write_type_7("hold_pose_in_world", data.getHoldPoseInWorld());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.ChestOrientationActionDefinitionMessage data)
   {
      ser.read_type_a("action_definition", new behavior_msgs.msg.dds.BehaviorActionDefinitionMessagePubSubType(), data.getActionDefinition());

      ser.read_type_e("parent_frame", data.getParentFrame());
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
