package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "MimicActionDefinitionMessage" defined in "MimicActionDefinitionMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from MimicActionDefinitionMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit MimicActionDefinitionMessage_.idl instead.
*
*/
public class MimicActionDefinitionMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.MimicActionDefinitionMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::MimicActionDefinitionMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "2d99e403ee43ce8c6dc3aea95cca1366a85fea0c8e9cb6ee3ac4cd1d5d5f1b2c";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.MimicActionDefinitionMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.MimicActionDefinitionMessage data) throws java.io.IOException
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

      current_alignment += behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.MimicActionDefinitionMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.MimicActionDefinitionMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.getCdrSerializedSize(data.getDefinition(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getMimicFileName().length() + 1;


      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.MimicActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.write(data.getDefinition(), cdr);
      cdr.write_type_9(data.getMimicActionType());

      if(data.getMimicFileName().length() <= 255)
      cdr.write_type_d(data.getMimicFileName());else
          throw new RuntimeException("mimic_file_name field exceeds the maximum length: %d > %d".formatted(data.getMimicFileName().length(), 255));

   }

   public static void read(behavior_msgs.msg.dds.MimicActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.read(data.getDefinition(), cdr);	
      data.setMimicActionType(cdr.read_type_9());
      	
      cdr.read_type_d(data.getMimicFileName());	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.MimicActionDefinitionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("definition", new behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType(), data.getDefinition());

      ser.write_type_9("mimic_action_type", data.getMimicActionType());
      ser.write_type_d("mimic_file_name", data.getMimicFileName());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.MimicActionDefinitionMessage data)
   {
      ser.read_type_a("definition", new behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType(), data.getDefinition());

      data.setMimicActionType(ser.read_type_9("mimic_action_type"));
      ser.read_type_d("mimic_file_name", data.getMimicFileName());
   }

   public static void staticCopy(behavior_msgs.msg.dds.MimicActionDefinitionMessage src, behavior_msgs.msg.dds.MimicActionDefinitionMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.MimicActionDefinitionMessage createData()
   {
      return new behavior_msgs.msg.dds.MimicActionDefinitionMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.MimicActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.MimicActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.MimicActionDefinitionMessage src, behavior_msgs.msg.dds.MimicActionDefinitionMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public MimicActionDefinitionMessagePubSubType newInstance()
   {
      return new MimicActionDefinitionMessagePubSubType();
   }
}
