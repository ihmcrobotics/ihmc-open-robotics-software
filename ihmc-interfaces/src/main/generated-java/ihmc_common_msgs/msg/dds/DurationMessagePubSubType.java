package ihmc_common_msgs.msg.dds;

/**
* 
* Topic data type of the struct "DurationMessage" defined in "DurationMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from DurationMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit DurationMessage_.idl instead.
*
*/
public class DurationMessagePubSubType implements us.ihmc.pubsub.TopicDataType<ihmc_common_msgs.msg.dds.DurationMessage>
{
   public static final java.lang.String name = "ihmc_common_msgs::msg::dds_::DurationMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "55e3f8f6af1ec3bcab2e6162583ea20513db3d2ffd88723eb39c77c94a7793a9";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(ihmc_common_msgs.msg.dds.DurationMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, ihmc_common_msgs.msg.dds.DurationMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.DurationMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.DurationMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(ihmc_common_msgs.msg.dds.DurationMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_11(data.getSeconds());

      cdr.write_type_2(data.getNanos());

   }

   public static void read(ihmc_common_msgs.msg.dds.DurationMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSeconds(cdr.read_type_11());
      	
      data.setNanos(cdr.read_type_2());
      	

   }

   @Override
   public final void serialize(ihmc_common_msgs.msg.dds.DurationMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_11("seconds", data.getSeconds());
      ser.write_type_2("nanos", data.getNanos());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, ihmc_common_msgs.msg.dds.DurationMessage data)
   {
      data.setSeconds(ser.read_type_11("seconds"));
      data.setNanos(ser.read_type_2("nanos"));
   }

   public static void staticCopy(ihmc_common_msgs.msg.dds.DurationMessage src, ihmc_common_msgs.msg.dds.DurationMessage dest)
   {
      dest.set(src);
   }

   @Override
   public ihmc_common_msgs.msg.dds.DurationMessage createData()
   {
      return new ihmc_common_msgs.msg.dds.DurationMessage();
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
   
   public void serialize(ihmc_common_msgs.msg.dds.DurationMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(ihmc_common_msgs.msg.dds.DurationMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(ihmc_common_msgs.msg.dds.DurationMessage src, ihmc_common_msgs.msg.dds.DurationMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public DurationMessagePubSubType newInstance()
   {
      return new DurationMessagePubSubType();
   }
}
