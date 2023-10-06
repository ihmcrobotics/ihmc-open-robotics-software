package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "BehaviorActionDefinitionMessage" defined in "BehaviorActionDefinitionMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from BehaviorActionDefinitionMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit BehaviorActionDefinitionMessage_.idl instead.
*
*/
public class BehaviorActionDefinitionMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.BehaviorActionDefinitionMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::BehaviorActionDefinitionMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "85852a4017c7a3779094995913a713cddd9abda407db51555b01232f901226cf";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.BehaviorActionDefinitionMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.BehaviorActionDefinitionMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.BehaviorActionDefinitionMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.BehaviorActionDefinitionMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getDescription().length() + 1;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.BehaviorActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      if(data.getDescription().length() <= 255)
      cdr.write_type_d(data.getDescription());else
          throw new RuntimeException("description field exceeds the maximum length");

      cdr.write_type_7(data.getExecuteWithNextAction());

   }

   public static void read(behavior_msgs.msg.dds.BehaviorActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_d(data.getDescription());	
      data.setExecuteWithNextAction(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.BehaviorActionDefinitionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_d("description", data.getDescription());
      ser.write_type_7("execute_with_next_action", data.getExecuteWithNextAction());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.BehaviorActionDefinitionMessage data)
   {
      ser.read_type_d("description", data.getDescription());
      data.setExecuteWithNextAction(ser.read_type_7("execute_with_next_action"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.BehaviorActionDefinitionMessage src, behavior_msgs.msg.dds.BehaviorActionDefinitionMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.BehaviorActionDefinitionMessage createData()
   {
      return new behavior_msgs.msg.dds.BehaviorActionDefinitionMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.BehaviorActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.BehaviorActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.BehaviorActionDefinitionMessage src, behavior_msgs.msg.dds.BehaviorActionDefinitionMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public BehaviorActionDefinitionMessagePubSubType newInstance()
   {
      return new BehaviorActionDefinitionMessagePubSubType();
   }
}
