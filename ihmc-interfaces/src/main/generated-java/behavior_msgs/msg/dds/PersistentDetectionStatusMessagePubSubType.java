package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "PersistentDetectionStatusMessage" defined in "PersistentDetectionStatusMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from PersistentDetectionStatusMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit PersistentDetectionStatusMessage_.idl instead.
*
*/
public class PersistentDetectionStatusMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.PersistentDetectionStatusMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::PersistentDetectionStatusMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "f0bb22fbf0de69e3c7f727a3369a05b6160b694aac227e58b3be3194e85171b9";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.PersistentDetectionStatusMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.PersistentDetectionStatusMessage data) throws java.io.IOException
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
      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.PersistentDetectionStatusMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.PersistentDetectionStatusMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getObjectClass().length() + 1;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.PersistentDetectionStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      if(data.getObjectClass().length() <= 255)
      cdr.write_type_d(data.getObjectClass());else
          throw new RuntimeException("object_class field exceeds the maximum length: %d > %d".formatted(data.getObjectClass().length(), 255));

      cdr.write_type_6(data.getDecayingFrequency());

      cdr.write_type_2(data.getHistorySize());

   }

   public static void read(behavior_msgs.msg.dds.PersistentDetectionStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_d(data.getObjectClass());	
      data.setDecayingFrequency(cdr.read_type_6());
      	
      data.setHistorySize(cdr.read_type_2());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.PersistentDetectionStatusMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_d("object_class", data.getObjectClass());
      ser.write_type_6("decaying_frequency", data.getDecayingFrequency());
      ser.write_type_2("history_size", data.getHistorySize());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.PersistentDetectionStatusMessage data)
   {
      ser.read_type_d("object_class", data.getObjectClass());
      data.setDecayingFrequency(ser.read_type_6("decaying_frequency"));
      data.setHistorySize(ser.read_type_2("history_size"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.PersistentDetectionStatusMessage src, behavior_msgs.msg.dds.PersistentDetectionStatusMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.PersistentDetectionStatusMessage createData()
   {
      return new behavior_msgs.msg.dds.PersistentDetectionStatusMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.PersistentDetectionStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.PersistentDetectionStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.PersistentDetectionStatusMessage src, behavior_msgs.msg.dds.PersistentDetectionStatusMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public PersistentDetectionStatusMessagePubSubType newInstance()
   {
      return new PersistentDetectionStatusMessagePubSubType();
   }
}
