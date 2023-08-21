package ihmc_common_msgs.msg.dds;

/**
* 
* Topic data type of the struct "InstantMessage" defined in "InstantMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from InstantMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit InstantMessage_.idl instead.
*
*/
public class InstantMessagePubSubType implements us.ihmc.pubsub.TopicDataType<ihmc_common_msgs.msg.dds.InstantMessage>
{
   public static final java.lang.String name = "ihmc_common_msgs::msg::dds_::InstantMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "3e624c67fb0f0f47f43355f36f18ef596dc450369cd0f412a66907c7032ae924";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(ihmc_common_msgs.msg.dds.InstantMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, ihmc_common_msgs.msg.dds.InstantMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.InstantMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.InstantMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(ihmc_common_msgs.msg.dds.InstantMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_11(data.getSecondsSinceEpoch());

      cdr.write_type_11(data.getAdditionalNanos());

   }

   public static void read(ihmc_common_msgs.msg.dds.InstantMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSecondsSinceEpoch(cdr.read_type_11());
      	
      data.setAdditionalNanos(cdr.read_type_11());
      	

   }

   @Override
   public final void serialize(ihmc_common_msgs.msg.dds.InstantMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_11("seconds_since_epoch", data.getSecondsSinceEpoch());
      ser.write_type_11("additional_nanos", data.getAdditionalNanos());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, ihmc_common_msgs.msg.dds.InstantMessage data)
   {
      data.setSecondsSinceEpoch(ser.read_type_11("seconds_since_epoch"));
      data.setAdditionalNanos(ser.read_type_11("additional_nanos"));
   }

   public static void staticCopy(ihmc_common_msgs.msg.dds.InstantMessage src, ihmc_common_msgs.msg.dds.InstantMessage dest)
   {
      dest.set(src);
   }

   @Override
   public ihmc_common_msgs.msg.dds.InstantMessage createData()
   {
      return new ihmc_common_msgs.msg.dds.InstantMessage();
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
   
   public void serialize(ihmc_common_msgs.msg.dds.InstantMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(ihmc_common_msgs.msg.dds.InstantMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(ihmc_common_msgs.msg.dds.InstantMessage src, ihmc_common_msgs.msg.dds.InstantMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public InstantMessagePubSubType newInstance()
   {
      return new InstantMessagePubSubType();
   }
}
