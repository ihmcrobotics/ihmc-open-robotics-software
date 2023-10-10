package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "HandWrenchActionStateMessage" defined in "HandWrenchActionStateMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from HandWrenchActionStateMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit HandWrenchActionStateMessage_.idl instead.
*
*/
public class HandWrenchActionStateMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.HandWrenchActionStateMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::HandWrenchActionStateMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "405343dd4d33ae9bf8ab8795419f3377b86529dba0570b07c799741476acd68b";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.HandWrenchActionStateMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.HandWrenchActionStateMessage data) throws java.io.IOException
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

      current_alignment += behavior_msgs.msg.dds.BehaviorActionStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += behavior_msgs.msg.dds.HandWrenchActionDefinitionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.HandWrenchActionStateMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.HandWrenchActionStateMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.BehaviorActionStateMessagePubSubType.getCdrSerializedSize(data.getActionState(), current_alignment);

      current_alignment += behavior_msgs.msg.dds.HandWrenchActionDefinitionMessagePubSubType.getCdrSerializedSize(data.getDefinition(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.HandWrenchActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.BehaviorActionStateMessagePubSubType.write(data.getActionState(), cdr);
      behavior_msgs.msg.dds.HandWrenchActionDefinitionMessagePubSubType.write(data.getDefinition(), cdr);
   }

   public static void read(behavior_msgs.msg.dds.HandWrenchActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.BehaviorActionStateMessagePubSubType.read(data.getActionState(), cdr);	
      behavior_msgs.msg.dds.HandWrenchActionDefinitionMessagePubSubType.read(data.getDefinition(), cdr);	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.HandWrenchActionStateMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("action_state", new behavior_msgs.msg.dds.BehaviorActionStateMessagePubSubType(), data.getActionState());

      ser.write_type_a("definition", new behavior_msgs.msg.dds.HandWrenchActionDefinitionMessagePubSubType(), data.getDefinition());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.HandWrenchActionStateMessage data)
   {
      ser.read_type_a("action_state", new behavior_msgs.msg.dds.BehaviorActionStateMessagePubSubType(), data.getActionState());

      ser.read_type_a("definition", new behavior_msgs.msg.dds.HandWrenchActionDefinitionMessagePubSubType(), data.getDefinition());

   }

   public static void staticCopy(behavior_msgs.msg.dds.HandWrenchActionStateMessage src, behavior_msgs.msg.dds.HandWrenchActionStateMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.HandWrenchActionStateMessage createData()
   {
      return new behavior_msgs.msg.dds.HandWrenchActionStateMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.HandWrenchActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.HandWrenchActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.HandWrenchActionStateMessage src, behavior_msgs.msg.dds.HandWrenchActionStateMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public HandWrenchActionStateMessagePubSubType newInstance()
   {
      return new HandWrenchActionStateMessagePubSubType();
   }
}
