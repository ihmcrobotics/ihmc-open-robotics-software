package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "AI2RScanMessage" defined in "AI2RScanMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from AI2RScanMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit AI2RScanMessage_.idl instead.
*
*/
public class AI2RScanMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.AI2RScanMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::AI2RScanMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "54361c9c28795ae5659fceb65002cafcd9908505c18d30199e8c0c10e5b35769";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.AI2RScanMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.AI2RScanMessage data) throws java.io.IOException
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
      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.AI2RScanMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.AI2RScanMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getTargetObject().length() + 1;

      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.AI2RScanMessage data, us.ihmc.idl.CDR cdr)
   {
      if(data.getTargetObject().length() <= 255)
      cdr.write_type_d(data.getTargetObject());else
          throw new RuntimeException("target_object field exceeds the maximum length: %d > %d".formatted(data.getTargetObject().length(), 255));

   }

   public static void read(behavior_msgs.msg.dds.AI2RScanMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_d(data.getTargetObject());	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.AI2RScanMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_d("target_object", data.getTargetObject());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.AI2RScanMessage data)
   {
      ser.read_type_d("target_object", data.getTargetObject());
   }

   public static void staticCopy(behavior_msgs.msg.dds.AI2RScanMessage src, behavior_msgs.msg.dds.AI2RScanMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.AI2RScanMessage createData()
   {
      return new behavior_msgs.msg.dds.AI2RScanMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.AI2RScanMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.AI2RScanMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.AI2RScanMessage src, behavior_msgs.msg.dds.AI2RScanMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public AI2RScanMessagePubSubType newInstance()
   {
      return new AI2RScanMessagePubSubType();
   }
}
