package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ImageMessage" defined in "ImageMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ImageMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ImageMessage_.idl instead.
*
*/
public class ImageMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.ImageMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::ImageMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.ImageMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.ImageMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (4000000 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ImageMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ImageMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getRgbdata().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.ImageMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_2(data.getWidth());

      cdr.write_type_2(data.getHeight());

      if(data.getRgbdata().size() <= 4000000)
      cdr.write_type_e(data.getRgbdata());else
          throw new RuntimeException("rgbdata field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.ImageMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setWidth(cdr.read_type_2());
      	
      data.setHeight(cdr.read_type_2());
      	
      cdr.read_type_e(data.getRgbdata());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.ImageMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_2("width", data.getWidth());
      ser.write_type_2("height", data.getHeight());
      ser.write_type_e("rgbdata", data.getRgbdata());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.ImageMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setWidth(ser.read_type_2("width"));
      data.setHeight(ser.read_type_2("height"));
      ser.read_type_e("rgbdata", data.getRgbdata());
   }

   public static void staticCopy(controller_msgs.msg.dds.ImageMessage src, controller_msgs.msg.dds.ImageMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.ImageMessage createData()
   {
      return new controller_msgs.msg.dds.ImageMessage();
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
   
   public void serialize(controller_msgs.msg.dds.ImageMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.ImageMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.ImageMessage src, controller_msgs.msg.dds.ImageMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ImageMessagePubSubType newInstance()
   {
      return new ImageMessagePubSubType();
   }
}
