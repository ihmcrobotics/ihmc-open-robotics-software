package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "AI2RNodeDefinitionMessage" defined in "AI2RNodeDefinitionMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from AI2RNodeDefinitionMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit AI2RNodeDefinitionMessage_.idl instead.
*
*/
public class AI2RNodeDefinitionMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.AI2RNodeDefinitionMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::AI2RNodeDefinitionMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "a40712f94ba6ad8aa2008fa5520ec5e073be4688f730e147acc18a5337b26ee3";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.AI2RNodeDefinitionMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.AI2RNodeDefinitionMessage data) throws java.io.IOException
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

      current_alignment += behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.AI2RNodeDefinitionMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.AI2RNodeDefinitionMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessagePubSubType.getCdrSerializedSize(data.getDefinition(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.AI2RNodeDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessagePubSubType.write(data.getDefinition(), cdr);   }

   public static void read(behavior_msgs.msg.dds.AI2RNodeDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessagePubSubType.read(data.getDefinition(), cdr);	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.AI2RNodeDefinitionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("definition", new behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessagePubSubType(), data.getDefinition());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.AI2RNodeDefinitionMessage data)
   {
      ser.read_type_a("definition", new behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessagePubSubType(), data.getDefinition());
   }

   public static void staticCopy(behavior_msgs.msg.dds.AI2RNodeDefinitionMessage src, behavior_msgs.msg.dds.AI2RNodeDefinitionMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.AI2RNodeDefinitionMessage createData()
   {
      return new behavior_msgs.msg.dds.AI2RNodeDefinitionMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.AI2RNodeDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.AI2RNodeDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.AI2RNodeDefinitionMessage src, behavior_msgs.msg.dds.AI2RNodeDefinitionMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public AI2RNodeDefinitionMessagePubSubType newInstance()
   {
      return new AI2RNodeDefinitionMessagePubSubType();
   }
}
