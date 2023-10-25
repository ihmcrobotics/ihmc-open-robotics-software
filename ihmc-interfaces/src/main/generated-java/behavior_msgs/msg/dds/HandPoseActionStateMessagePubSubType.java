package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "HandPoseActionStateMessage" defined in "HandPoseActionStateMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from HandPoseActionStateMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit HandPoseActionStateMessage_.idl instead.
*
*/
public class HandPoseActionStateMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.HandPoseActionStateMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::HandPoseActionStateMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "f72ec300a03ace38ee92703050cc7c0c7658c7d3377f066f68aadc61d98ee41e";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.HandPoseActionStateMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.HandPoseActionStateMessage data) throws java.io.IOException
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

      current_alignment += behavior_msgs.msg.dds.HandPoseActionDefinitionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.HandPoseActionStateMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.HandPoseActionStateMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.getCdrSerializedSize(data.getActionState(), current_alignment);

      current_alignment += behavior_msgs.msg.dds.HandPoseActionDefinitionMessagePubSubType.getCdrSerializedSize(data.getDefinition(), current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.HandPoseActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.write(data.getActionState(), cdr);
      behavior_msgs.msg.dds.HandPoseActionDefinitionMessagePubSubType.write(data.getDefinition(), cdr);
      cdr.write_type_6(data.getHandWrenchMagnitudeLinear());

   }

   public static void read(behavior_msgs.msg.dds.HandPoseActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.read(data.getActionState(), cdr);	
      behavior_msgs.msg.dds.HandPoseActionDefinitionMessagePubSubType.read(data.getDefinition(), cdr);	
      data.setHandWrenchMagnitudeLinear(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.HandPoseActionStateMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("action_state", new behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType(), data.getActionState());

      ser.write_type_a("definition", new behavior_msgs.msg.dds.HandPoseActionDefinitionMessagePubSubType(), data.getDefinition());

      ser.write_type_6("hand_wrench_magnitude_linear", data.getHandWrenchMagnitudeLinear());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.HandPoseActionStateMessage data)
   {
      ser.read_type_a("action_state", new behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType(), data.getActionState());

      ser.read_type_a("definition", new behavior_msgs.msg.dds.HandPoseActionDefinitionMessagePubSubType(), data.getDefinition());

      data.setHandWrenchMagnitudeLinear(ser.read_type_6("hand_wrench_magnitude_linear"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.HandPoseActionStateMessage src, behavior_msgs.msg.dds.HandPoseActionStateMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.HandPoseActionStateMessage createData()
   {
      return new behavior_msgs.msg.dds.HandPoseActionStateMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.HandPoseActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.HandPoseActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.HandPoseActionStateMessage src, behavior_msgs.msg.dds.HandPoseActionStateMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public HandPoseActionStateMessagePubSubType newInstance()
   {
      return new HandPoseActionStateMessagePubSubType();
   }
}
