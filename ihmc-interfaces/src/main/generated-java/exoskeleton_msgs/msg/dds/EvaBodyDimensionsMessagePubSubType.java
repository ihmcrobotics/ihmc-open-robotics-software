package exoskeleton_msgs.msg.dds;

/**
* 
* Topic data type of the struct "EvaBodyDimensionsMessage" defined in "EvaBodyDimensionsMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from EvaBodyDimensionsMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit EvaBodyDimensionsMessage_.idl instead.
*
*/
public class EvaBodyDimensionsMessagePubSubType implements us.ihmc.pubsub.TopicDataType<exoskeleton_msgs.msg.dds.EvaBodyDimensionsMessage>
{
   public static final java.lang.String name = "exoskeleton_msgs::msg::dds_::EvaBodyDimensionsMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "360ac6a3caf40584ac16c6c17a4fc488ac29cfd098a9c3e88346c4eb0c151a20";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(exoskeleton_msgs.msg.dds.EvaBodyDimensionsMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, exoskeleton_msgs.msg.dds.EvaBodyDimensionsMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (2048 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(exoskeleton_msgs.msg.dds.EvaBodyDimensionsMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(exoskeleton_msgs.msg.dds.EvaBodyDimensionsMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getFileName().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(exoskeleton_msgs.msg.dds.EvaBodyDimensionsMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      if(data.getFileName().size() <= 2048)
      cdr.write_type_e(data.getFileName());else
          throw new RuntimeException("file_name field exceeds the maximum length");

      cdr.write_type_6(data.getThighLength());

      cdr.write_type_6(data.getShankLength());

      cdr.write_type_4(data.getShoeSizeOridinal());

   }

   public static void read(exoskeleton_msgs.msg.dds.EvaBodyDimensionsMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      cdr.read_type_e(data.getFileName());	
      data.setThighLength(cdr.read_type_6());
      	
      data.setShankLength(cdr.read_type_6());
      	
      data.setShoeSizeOridinal(cdr.read_type_4());
      	

   }

   @Override
   public final void serialize(exoskeleton_msgs.msg.dds.EvaBodyDimensionsMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_e("file_name", data.getFileName());
      ser.write_type_6("thigh_length", data.getThighLength());
      ser.write_type_6("shank_length", data.getShankLength());
      ser.write_type_4("shoe_size_oridinal", data.getShoeSizeOridinal());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, exoskeleton_msgs.msg.dds.EvaBodyDimensionsMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_e("file_name", data.getFileName());
      data.setThighLength(ser.read_type_6("thigh_length"));
      data.setShankLength(ser.read_type_6("shank_length"));
      data.setShoeSizeOridinal(ser.read_type_4("shoe_size_oridinal"));
   }

   public static void staticCopy(exoskeleton_msgs.msg.dds.EvaBodyDimensionsMessage src, exoskeleton_msgs.msg.dds.EvaBodyDimensionsMessage dest)
   {
      dest.set(src);
   }

   @Override
   public exoskeleton_msgs.msg.dds.EvaBodyDimensionsMessage createData()
   {
      return new exoskeleton_msgs.msg.dds.EvaBodyDimensionsMessage();
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
   
   public void serialize(exoskeleton_msgs.msg.dds.EvaBodyDimensionsMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(exoskeleton_msgs.msg.dds.EvaBodyDimensionsMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(exoskeleton_msgs.msg.dds.EvaBodyDimensionsMessage src, exoskeleton_msgs.msg.dds.EvaBodyDimensionsMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public EvaBodyDimensionsMessagePubSubType newInstance()
   {
      return new EvaBodyDimensionsMessagePubSubType();
   }
}
