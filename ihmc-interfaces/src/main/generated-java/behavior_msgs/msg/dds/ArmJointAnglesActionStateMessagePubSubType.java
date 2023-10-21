package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ArmJointAnglesActionStateMessage" defined in "ArmJointAnglesActionStateMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ArmJointAnglesActionStateMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ArmJointAnglesActionStateMessage_.idl instead.
*
*/
public class ArmJointAnglesActionStateMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.ArmJointAnglesActionStateMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::ArmJointAnglesActionStateMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "afc58748c67cad05b946718a72babdd30cdc91f9a37c4eb275ee39d0023a48bd";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.ArmJointAnglesActionStateMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.ArmJointAnglesActionStateMessage data) throws java.io.IOException
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

      current_alignment += behavior_msgs.msg.dds.ArmJointAnglesActionDefinitionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.ArmJointAnglesActionStateMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.ArmJointAnglesActionStateMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.BehaviorActionStateMessagePubSubType.getCdrSerializedSize(data.getActionState(), current_alignment);

      current_alignment += behavior_msgs.msg.dds.ArmJointAnglesActionDefinitionMessagePubSubType.getCdrSerializedSize(data.getDefinition(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.ArmJointAnglesActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.BehaviorActionStateMessagePubSubType.write(data.getActionState(), cdr);
      behavior_msgs.msg.dds.ArmJointAnglesActionDefinitionMessagePubSubType.write(data.getDefinition(), cdr);
   }

   public static void read(behavior_msgs.msg.dds.ArmJointAnglesActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.BehaviorActionStateMessagePubSubType.read(data.getActionState(), cdr);	
      behavior_msgs.msg.dds.ArmJointAnglesActionDefinitionMessagePubSubType.read(data.getDefinition(), cdr);	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.ArmJointAnglesActionStateMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("action_state", new behavior_msgs.msg.dds.BehaviorActionStateMessagePubSubType(), data.getActionState());

      ser.write_type_a("definition", new behavior_msgs.msg.dds.ArmJointAnglesActionDefinitionMessagePubSubType(), data.getDefinition());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.ArmJointAnglesActionStateMessage data)
   {
      ser.read_type_a("action_state", new behavior_msgs.msg.dds.BehaviorActionStateMessagePubSubType(), data.getActionState());

      ser.read_type_a("definition", new behavior_msgs.msg.dds.ArmJointAnglesActionDefinitionMessagePubSubType(), data.getDefinition());

   }

   public static void staticCopy(behavior_msgs.msg.dds.ArmJointAnglesActionStateMessage src, behavior_msgs.msg.dds.ArmJointAnglesActionStateMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.ArmJointAnglesActionStateMessage createData()
   {
      return new behavior_msgs.msg.dds.ArmJointAnglesActionStateMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.ArmJointAnglesActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.ArmJointAnglesActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.ArmJointAnglesActionStateMessage src, behavior_msgs.msg.dds.ArmJointAnglesActionStateMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ArmJointAnglesActionStateMessagePubSubType newInstance()
   {
      return new ArmJointAnglesActionStateMessagePubSubType();
   }
}
