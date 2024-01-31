package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "BigVideoPacket" defined in "BigVideoPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from BigVideoPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit BigVideoPacket_.idl instead.
*
*/
public class BigVideoPacketPubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.BigVideoPacket>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::BigVideoPacket_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "2e35cabc2007a999a9e3abf536e1af29a0b2697cfa9c4814e14336db85aeb998";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.BigVideoPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.BigVideoPacket data) throws java.io.IOException
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

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (25000000 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.BigVideoPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.BigVideoPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getData().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.BigVideoPacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_11(data.getAcquisitionTimeSecondsSinceEpoch());

      cdr.write_type_11(data.getAcquisitionTimeAdditionalNanos());

      cdr.write_type_3(data.getImageWidth());

      cdr.write_type_3(data.getImageHeight());

      if(data.getData().size() <= 25000000)
      cdr.write_type_e(data.getData());else
          throw new RuntimeException("data field exceeds the maximum length");

   }

   public static void read(perception_msgs.msg.dds.BigVideoPacket data, us.ihmc.idl.CDR cdr)
   {
      data.setAcquisitionTimeSecondsSinceEpoch(cdr.read_type_11());
      	
      data.setAcquisitionTimeAdditionalNanos(cdr.read_type_11());
      	
      data.setImageWidth(cdr.read_type_3());
      	
      data.setImageHeight(cdr.read_type_3());
      	
      cdr.read_type_e(data.getData());	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.BigVideoPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_11("acquisition_time_seconds_since_epoch", data.getAcquisitionTimeSecondsSinceEpoch());
      ser.write_type_11("acquisition_time_additional_nanos", data.getAcquisitionTimeAdditionalNanos());
      ser.write_type_3("image_width", data.getImageWidth());
      ser.write_type_3("image_height", data.getImageHeight());
      ser.write_type_e("data", data.getData());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.BigVideoPacket data)
   {
      data.setAcquisitionTimeSecondsSinceEpoch(ser.read_type_11("acquisition_time_seconds_since_epoch"));
      data.setAcquisitionTimeAdditionalNanos(ser.read_type_11("acquisition_time_additional_nanos"));
      data.setImageWidth(ser.read_type_3("image_width"));
      data.setImageHeight(ser.read_type_3("image_height"));
      ser.read_type_e("data", data.getData());
   }

   public static void staticCopy(perception_msgs.msg.dds.BigVideoPacket src, perception_msgs.msg.dds.BigVideoPacket dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.BigVideoPacket createData()
   {
      return new perception_msgs.msg.dds.BigVideoPacket();
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
   
   public void serialize(perception_msgs.msg.dds.BigVideoPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.BigVideoPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.BigVideoPacket src, perception_msgs.msg.dds.BigVideoPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public BigVideoPacketPubSubType newInstance()
   {
      return new BigVideoPacketPubSubType();
   }
}
