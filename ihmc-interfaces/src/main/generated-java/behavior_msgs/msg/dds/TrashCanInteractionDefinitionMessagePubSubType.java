package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "TrashCanInteractionDefinitionMessage" defined in "TrashCanInteractionDefinitionMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from TrashCanInteractionDefinitionMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit TrashCanInteractionDefinitionMessage_.idl instead.
*
*/
public class TrashCanInteractionDefinitionMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.TrashCanInteractionDefinitionMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::TrashCanInteractionDefinitionMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "375541de91116a9813ef2a33f0229306e37009db14580e5d157639f33d4d5428";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.TrashCanInteractionDefinitionMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.TrashCanInteractionDefinitionMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.TrashCanInteractionDefinitionMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.TrashCanInteractionDefinitionMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessagePubSubType.getCdrSerializedSize(data.getDefinition(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getObstructedNodeName().length() + 1;


      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.TrashCanInteractionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessagePubSubType.write(data.getDefinition(), cdr);
      if(data.getObstructedNodeName().length() <= 255)
      cdr.write_type_d(data.getObstructedNodeName());else
          throw new RuntimeException("obstructed_node_name field exceeds the maximum length");

   }

   public static void read(behavior_msgs.msg.dds.TrashCanInteractionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessagePubSubType.read(data.getDefinition(), cdr);	
      cdr.read_type_d(data.getObstructedNodeName());	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.TrashCanInteractionDefinitionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("definition", new behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessagePubSubType(), data.getDefinition());

      ser.write_type_d("obstructed_node_name", data.getObstructedNodeName());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.TrashCanInteractionDefinitionMessage data)
   {
      ser.read_type_a("definition", new behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessagePubSubType(), data.getDefinition());

      ser.read_type_d("obstructed_node_name", data.getObstructedNodeName());
   }

   public static void staticCopy(behavior_msgs.msg.dds.TrashCanInteractionDefinitionMessage src, behavior_msgs.msg.dds.TrashCanInteractionDefinitionMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.TrashCanInteractionDefinitionMessage createData()
   {
      return new behavior_msgs.msg.dds.TrashCanInteractionDefinitionMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.TrashCanInteractionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.TrashCanInteractionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.TrashCanInteractionDefinitionMessage src, behavior_msgs.msg.dds.TrashCanInteractionDefinitionMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public TrashCanInteractionDefinitionMessagePubSubType newInstance()
   {
      return new TrashCanInteractionDefinitionMessagePubSubType();
   }
}
