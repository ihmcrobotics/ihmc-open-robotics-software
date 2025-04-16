package unitree_go_msgs.msg.dds;

/**
* 
* Topic data type of the struct "AudioData" defined in "AudioData_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from AudioData_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit AudioData_.idl instead.
*
*/
public class AudioDataPubSubType implements us.ihmc.pubsub.TopicDataType<unitree_go_msgs.msg.dds.AudioData>
{
   public static final java.lang.String name = "unitree_go_msgs::msg::dds_::AudioData_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "8c273945baa97bdd38a5ccad6c1e7ac337247668b64879c861d804e29b75236c";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(unitree_go_msgs.msg.dds.AudioData data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, unitree_go_msgs.msg.dds.AudioData data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(unitree_go_msgs.msg.dds.AudioData data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(unitree_go_msgs.msg.dds.AudioData data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getData().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(unitree_go_msgs.msg.dds.AudioData data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_12(data.getTimeFrame());

      if(data.getData().size() <= 100)
      cdr.write_type_e(data.getData());else
          throw new RuntimeException("data field exceeds the maximum length: %d > %d".formatted(data.getData().size(), 100));

   }

   public static void read(unitree_go_msgs.msg.dds.AudioData data, us.ihmc.idl.CDR cdr)
   {
      data.setTimeFrame(cdr.read_type_12());
      	
      cdr.read_type_e(data.getData());	

   }

   @Override
   public final void serialize(unitree_go_msgs.msg.dds.AudioData data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_12("time_frame", data.getTimeFrame());
      ser.write_type_e("data", data.getData());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, unitree_go_msgs.msg.dds.AudioData data)
   {
      data.setTimeFrame(ser.read_type_12("time_frame"));
      ser.read_type_e("data", data.getData());
   }

   public static void staticCopy(unitree_go_msgs.msg.dds.AudioData src, unitree_go_msgs.msg.dds.AudioData dest)
   {
      dest.set(src);
   }

   @Override
   public unitree_go_msgs.msg.dds.AudioData createData()
   {
      return new unitree_go_msgs.msg.dds.AudioData();
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
   
   public void serialize(unitree_go_msgs.msg.dds.AudioData data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(unitree_go_msgs.msg.dds.AudioData data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(unitree_go_msgs.msg.dds.AudioData src, unitree_go_msgs.msg.dds.AudioData dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public AudioDataPubSubType newInstance()
   {
      return new AudioDataPubSubType();
   }
}
