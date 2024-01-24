package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ChestOrientationActionStateMessage" defined in "ChestOrientationActionStateMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ChestOrientationActionStateMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ChestOrientationActionStateMessage_.idl instead.
*
*/
public class ChestOrientationActionStateMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.ChestOrientationActionStateMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::ChestOrientationActionStateMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "6bdb0d658a3ec62ad7e40396ea5af73ba897626073e85c5a7274edb1fb9d8c9c";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.ChestOrientationActionStateMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.ChestOrientationActionStateMessage data) throws java.io.IOException
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

      current_alignment += behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += behavior_msgs.msg.dds.ChestOrientationActionDefinitionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.ChestOrientationActionStateMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.ChestOrientationActionStateMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.getCdrSerializedSize(data.getState(), current_alignment);

      current_alignment += behavior_msgs.msg.dds.ChestOrientationActionDefinitionMessagePubSubType.getCdrSerializedSize(data.getDefinition(), current_alignment);

      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getCdrSerializedSize(data.getGoalPelvisTransformToWorld(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.ChestOrientationActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.write(data.getState(), cdr);
      behavior_msgs.msg.dds.ChestOrientationActionDefinitionMessagePubSubType.write(data.getDefinition(), cdr);
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.write(data.getGoalPelvisTransformToWorld(), cdr);
   }

   public static void read(behavior_msgs.msg.dds.ChestOrientationActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.read(data.getState(), cdr);	
      behavior_msgs.msg.dds.ChestOrientationActionDefinitionMessagePubSubType.read(data.getDefinition(), cdr);	
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.read(data.getGoalPelvisTransformToWorld(), cdr);	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.ChestOrientationActionStateMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("state", new behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType(), data.getState());

      ser.write_type_a("definition", new behavior_msgs.msg.dds.ChestOrientationActionDefinitionMessagePubSubType(), data.getDefinition());

      ser.write_type_a("goal_pelvis_transform_to_world", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getGoalPelvisTransformToWorld());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.ChestOrientationActionStateMessage data)
   {
      ser.read_type_a("state", new behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType(), data.getState());

      ser.read_type_a("definition", new behavior_msgs.msg.dds.ChestOrientationActionDefinitionMessagePubSubType(), data.getDefinition());

      ser.read_type_a("goal_pelvis_transform_to_world", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getGoalPelvisTransformToWorld());

   }

   public static void staticCopy(behavior_msgs.msg.dds.ChestOrientationActionStateMessage src, behavior_msgs.msg.dds.ChestOrientationActionStateMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.ChestOrientationActionStateMessage createData()
   {
      return new behavior_msgs.msg.dds.ChestOrientationActionStateMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.ChestOrientationActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.ChestOrientationActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.ChestOrientationActionStateMessage src, behavior_msgs.msg.dds.ChestOrientationActionStateMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ChestOrientationActionStateMessagePubSubType newInstance()
   {
      return new ChestOrientationActionStateMessagePubSubType();
   }
}
