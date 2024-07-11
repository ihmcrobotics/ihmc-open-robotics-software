package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "TrashCanInteractionStateMessage" defined in "TrashCanInteractionStateMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from TrashCanInteractionStateMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit TrashCanInteractionStateMessage_.idl instead.
*
*/
public class TrashCanInteractionStateMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.TrashCanInteractionStateMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::TrashCanInteractionStateMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "c26c858d36b532d2e415f0acdf7089b57f72fc21d4a5542ec109922a5e9cb492";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.TrashCanInteractionStateMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.TrashCanInteractionStateMessage data) throws java.io.IOException
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

      current_alignment += behavior_msgs.msg.dds.TrashCanInteractionDefinitionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.TrashCanInteractionStateMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.TrashCanInteractionStateMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType.getCdrSerializedSize(data.getState(), current_alignment);

      current_alignment += behavior_msgs.msg.dds.TrashCanInteractionDefinitionMessagePubSubType.getCdrSerializedSize(data.getDefinition(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.TrashCanInteractionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType.write(data.getState(), cdr);
      behavior_msgs.msg.dds.TrashCanInteractionDefinitionMessagePubSubType.write(data.getDefinition(), cdr);
      cdr.write_type_2(data.getStance());

   }

   public static void read(behavior_msgs.msg.dds.TrashCanInteractionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType.read(data.getState(), cdr);	
      behavior_msgs.msg.dds.TrashCanInteractionDefinitionMessagePubSubType.read(data.getDefinition(), cdr);	
      data.setStance(cdr.read_type_2());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.TrashCanInteractionStateMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("state", new behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType(), data.getState());

      ser.write_type_a("definition", new behavior_msgs.msg.dds.TrashCanInteractionDefinitionMessagePubSubType(), data.getDefinition());

      ser.write_type_2("stance", data.getStance());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.TrashCanInteractionStateMessage data)
   {
      ser.read_type_a("state", new behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType(), data.getState());

      ser.read_type_a("definition", new behavior_msgs.msg.dds.TrashCanInteractionDefinitionMessagePubSubType(), data.getDefinition());

      data.setStance(ser.read_type_2("stance"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.TrashCanInteractionStateMessage src, behavior_msgs.msg.dds.TrashCanInteractionStateMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.TrashCanInteractionStateMessage createData()
   {
      return new behavior_msgs.msg.dds.TrashCanInteractionStateMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.TrashCanInteractionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.TrashCanInteractionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.TrashCanInteractionStateMessage src, behavior_msgs.msg.dds.TrashCanInteractionStateMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public TrashCanInteractionStateMessagePubSubType newInstance()
   {
      return new TrashCanInteractionStateMessagePubSubType();
   }
}
