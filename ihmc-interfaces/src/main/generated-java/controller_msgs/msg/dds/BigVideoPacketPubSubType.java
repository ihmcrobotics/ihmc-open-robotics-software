package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "BigVideoPacket" defined in "BigVideoPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from BigVideoPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit BigVideoPacket_.idl instead.
*
*/
public class BigVideoPacketPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.BigVideoPacket>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::BigVideoPacket_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.BigVideoPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.BigVideoPacket data) throws java.io.IOException
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

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (25000000 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.BigVideoPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.BigVideoPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getData().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.BigVideoPacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_11(data.getAcquisitionTimeNanos());

      cdr.write_type_3(data.getImageWidth());

      cdr.write_type_3(data.getImageHeight());

      if(data.getData().size() <= 25000000)
      cdr.write_type_e(data.getData());else
          throw new RuntimeException("data field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.BigVideoPacket data, us.ihmc.idl.CDR cdr)
   {
      data.setAcquisitionTimeNanos(cdr.read_type_11());
      	
      data.setImageWidth(cdr.read_type_3());
      	
      data.setImageHeight(cdr.read_type_3());
      	
      cdr.read_type_e(data.getData());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.BigVideoPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_11("acquisition_time_nanos", data.getAcquisitionTimeNanos());
      ser.write_type_3("image_width", data.getImageWidth());
      ser.write_type_3("image_height", data.getImageHeight());
      ser.write_type_e("data", data.getData());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.BigVideoPacket data)
   {
      data.setAcquisitionTimeNanos(ser.read_type_11("acquisition_time_nanos"));
      data.setImageWidth(ser.read_type_3("image_width"));
      data.setImageHeight(ser.read_type_3("image_height"));
      ser.read_type_e("data", data.getData());
   }

   public static void staticCopy(controller_msgs.msg.dds.BigVideoPacket src, controller_msgs.msg.dds.BigVideoPacket dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.BigVideoPacket createData()
   {
      return new controller_msgs.msg.dds.BigVideoPacket();
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
   
   public void serialize(controller_msgs.msg.dds.BigVideoPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.BigVideoPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.BigVideoPacket src, controller_msgs.msg.dds.BigVideoPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public BigVideoPacketPubSubType newInstance()
   {
      return new BigVideoPacketPubSubType();
   }
}
