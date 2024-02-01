package ihmc_common_msgs.msg.dds;

/**
* 
* Topic data type of the struct "TimeIntervalMessage" defined in "TimeIntervalMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from TimeIntervalMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit TimeIntervalMessage_.idl instead.
*
*/
public class TimeIntervalMessagePubSubType implements us.ihmc.pubsub.TopicDataType<ihmc_common_msgs.msg.dds.TimeIntervalMessage>
{
   public static final java.lang.String name = "ihmc_common_msgs::msg::dds_::TimeIntervalMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "a0653a5a82a206833f2d98e8452fcf1d28a94dfde1c0bbe446ce5dfbfa60a443";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(ihmc_common_msgs.msg.dds.TimeIntervalMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, ihmc_common_msgs.msg.dds.TimeIntervalMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.TimeIntervalMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.TimeIntervalMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(ihmc_common_msgs.msg.dds.TimeIntervalMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_6(data.getStartTime());

      cdr.write_type_6(data.getEndTime());

   }

   public static void read(ihmc_common_msgs.msg.dds.TimeIntervalMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setStartTime(cdr.read_type_6());
      	
      data.setEndTime(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(ihmc_common_msgs.msg.dds.TimeIntervalMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_6("start_time", data.getStartTime());
      ser.write_type_6("end_time", data.getEndTime());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, ihmc_common_msgs.msg.dds.TimeIntervalMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setStartTime(ser.read_type_6("start_time"));
      data.setEndTime(ser.read_type_6("end_time"));
   }

   public static void staticCopy(ihmc_common_msgs.msg.dds.TimeIntervalMessage src, ihmc_common_msgs.msg.dds.TimeIntervalMessage dest)
   {
      dest.set(src);
   }

   @Override
   public ihmc_common_msgs.msg.dds.TimeIntervalMessage createData()
   {
      return new ihmc_common_msgs.msg.dds.TimeIntervalMessage();
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
   
   public void serialize(ihmc_common_msgs.msg.dds.TimeIntervalMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(ihmc_common_msgs.msg.dds.TimeIntervalMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(ihmc_common_msgs.msg.dds.TimeIntervalMessage src, ihmc_common_msgs.msg.dds.TimeIntervalMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public TimeIntervalMessagePubSubType newInstance()
   {
      return new TimeIntervalMessagePubSubType();
   }
}
