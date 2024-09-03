package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "AI2RNodeStateMessage" defined in "AI2RNodeStateMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from AI2RNodeStateMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit AI2RNodeStateMessage_.idl instead.
*
*/
public class AI2RNodeStateMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.AI2RNodeStateMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::AI2RNodeStateMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "f04f9eb48576e6f2f43967189a524f4ba3a43c2e0998b4b6b5c83fb7cabd7b8e";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.AI2RNodeStateMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.AI2RNodeStateMessage data) throws java.io.IOException
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

      current_alignment += behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += behavior_msgs.msg.dds.AI2RNodeDefinitionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.AI2RNodeStateMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.AI2RNodeStateMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType.getCdrSerializedSize(data.getState(), current_alignment);

      current_alignment += behavior_msgs.msg.dds.AI2RNodeDefinitionMessagePubSubType.getCdrSerializedSize(data.getDefinition(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.AI2RNodeStateMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType.write(data.getState(), cdr);
      behavior_msgs.msg.dds.AI2RNodeDefinitionMessagePubSubType.write(data.getDefinition(), cdr);
   }

   public static void read(behavior_msgs.msg.dds.AI2RNodeStateMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType.read(data.getState(), cdr);	
      behavior_msgs.msg.dds.AI2RNodeDefinitionMessagePubSubType.read(data.getDefinition(), cdr);	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.AI2RNodeStateMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("state", new behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType(), data.getState());

      ser.write_type_a("definition", new behavior_msgs.msg.dds.AI2RNodeDefinitionMessagePubSubType(), data.getDefinition());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.AI2RNodeStateMessage data)
   {
      ser.read_type_a("state", new behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType(), data.getState());

      ser.read_type_a("definition", new behavior_msgs.msg.dds.AI2RNodeDefinitionMessagePubSubType(), data.getDefinition());

   }

   public static void staticCopy(behavior_msgs.msg.dds.AI2RNodeStateMessage src, behavior_msgs.msg.dds.AI2RNodeStateMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.AI2RNodeStateMessage createData()
   {
      return new behavior_msgs.msg.dds.AI2RNodeStateMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.AI2RNodeStateMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.AI2RNodeStateMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.AI2RNodeStateMessage src, behavior_msgs.msg.dds.AI2RNodeStateMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public AI2RNodeStateMessagePubSubType newInstance()
   {
      return new AI2RNodeStateMessagePubSubType();
   }
}
