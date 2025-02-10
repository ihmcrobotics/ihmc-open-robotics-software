package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "GotoNodeDefinitionMessage" defined in "GotoNodeDefinitionMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from GotoNodeDefinitionMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit GotoNodeDefinitionMessage_.idl instead.
*
*/
public class GotoNodeDefinitionMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.GotoNodeDefinitionMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::GotoNodeDefinitionMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "a418e6d49ae3142953e73b6b9d9e23b53338078c82ae77e18e759d58ff6e69f2";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.GotoNodeDefinitionMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.GotoNodeDefinitionMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.GotoNodeDefinitionMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.GotoNodeDefinitionMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.LeafNodeDefinitionMessagePubSubType.getCdrSerializedSize(data.getDefinition(), current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.GotoNodeDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.LeafNodeDefinitionMessagePubSubType.write(data.getDefinition(), cdr);
      cdr.write_type_11(data.getNodeToGotoId());

   }

   public static void read(behavior_msgs.msg.dds.GotoNodeDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.LeafNodeDefinitionMessagePubSubType.read(data.getDefinition(), cdr);	
      data.setNodeToGotoId(cdr.read_type_11());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.GotoNodeDefinitionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("definition", new behavior_msgs.msg.dds.LeafNodeDefinitionMessagePubSubType(), data.getDefinition());

      ser.write_type_11("node_to_goto_id", data.getNodeToGotoId());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.GotoNodeDefinitionMessage data)
   {
      ser.read_type_a("definition", new behavior_msgs.msg.dds.LeafNodeDefinitionMessagePubSubType(), data.getDefinition());

      data.setNodeToGotoId(ser.read_type_11("node_to_goto_id"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.GotoNodeDefinitionMessage src, behavior_msgs.msg.dds.GotoNodeDefinitionMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.GotoNodeDefinitionMessage createData()
   {
      return new behavior_msgs.msg.dds.GotoNodeDefinitionMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.GotoNodeDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.GotoNodeDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.GotoNodeDefinitionMessage src, behavior_msgs.msg.dds.GotoNodeDefinitionMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public GotoNodeDefinitionMessagePubSubType newInstance()
   {
      return new GotoNodeDefinitionMessagePubSubType();
   }
}
