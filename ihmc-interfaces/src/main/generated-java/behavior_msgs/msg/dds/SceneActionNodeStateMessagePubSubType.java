package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "SceneActionNodeStateMessage" defined in "SceneActionNodeStateMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from SceneActionNodeStateMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit SceneActionNodeStateMessage_.idl instead.
*
*/
public class SceneActionNodeStateMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.SceneActionNodeStateMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::SceneActionNodeStateMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "37aa66fdd47363a95a82edff3316e7f52026849a8a5ca9125adf6f1f54fa0417";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.SceneActionNodeStateMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.SceneActionNodeStateMessage data) throws java.io.IOException
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

      current_alignment += behavior_msgs.msg.dds.SceneActionNodeDefinitionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.SceneActionNodeStateMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.SceneActionNodeStateMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.getCdrSerializedSize(data.getState(), current_alignment);

      current_alignment += behavior_msgs.msg.dds.SceneActionNodeDefinitionMessagePubSubType.getCdrSerializedSize(data.getDefinition(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.SceneActionNodeStateMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.write(data.getState(), cdr);
      behavior_msgs.msg.dds.SceneActionNodeDefinitionMessagePubSubType.write(data.getDefinition(), cdr);
   }

   public static void read(behavior_msgs.msg.dds.SceneActionNodeStateMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.read(data.getState(), cdr);	
      behavior_msgs.msg.dds.SceneActionNodeDefinitionMessagePubSubType.read(data.getDefinition(), cdr);	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.SceneActionNodeStateMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("state", new behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType(), data.getState());

      ser.write_type_a("definition", new behavior_msgs.msg.dds.SceneActionNodeDefinitionMessagePubSubType(), data.getDefinition());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.SceneActionNodeStateMessage data)
   {
      ser.read_type_a("state", new behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType(), data.getState());

      ser.read_type_a("definition", new behavior_msgs.msg.dds.SceneActionNodeDefinitionMessagePubSubType(), data.getDefinition());

   }

   public static void staticCopy(behavior_msgs.msg.dds.SceneActionNodeStateMessage src, behavior_msgs.msg.dds.SceneActionNodeStateMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.SceneActionNodeStateMessage createData()
   {
      return new behavior_msgs.msg.dds.SceneActionNodeStateMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.SceneActionNodeStateMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.SceneActionNodeStateMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.SceneActionNodeStateMessage src, behavior_msgs.msg.dds.SceneActionNodeStateMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public SceneActionNodeStateMessagePubSubType newInstance()
   {
      return new SceneActionNodeStateMessagePubSubType();
   }
}
