package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "BoundingBoxesPacket" defined in "BoundingBoxesPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from BoundingBoxesPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit BoundingBoxesPacket_.idl instead.
*
*/
public class BoundingBoxesPacketPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.BoundingBoxesPacket>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::BoundingBoxesPacket_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "0cd676e84e192f81e02d1d8da8e2d1d24ede0f452717401e834829c5d4396f0a";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.BoundingBoxesPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.BoundingBoxesPacket data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
        current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      }

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.BoundingBoxesPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.BoundingBoxesPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getBoundingBoxesXCoordinates().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getBoundingBoxesYCoordinates().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getBoundingBoxesWidths().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getBoundingBoxesHeights().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getLabels().size(); ++i0)
      {
          current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getLabels().get(i0).length() + 1;
      }

      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.BoundingBoxesPacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      if(data.getBoundingBoxesXCoordinates().size() <= 100)
      cdr.write_type_e(data.getBoundingBoxesXCoordinates());else
          throw new RuntimeException("bounding_boxes_x_coordinates field exceeds the maximum length");

      if(data.getBoundingBoxesYCoordinates().size() <= 100)
      cdr.write_type_e(data.getBoundingBoxesYCoordinates());else
          throw new RuntimeException("bounding_boxes_y_coordinates field exceeds the maximum length");

      if(data.getBoundingBoxesWidths().size() <= 100)
      cdr.write_type_e(data.getBoundingBoxesWidths());else
          throw new RuntimeException("bounding_boxes_widths field exceeds the maximum length");

      if(data.getBoundingBoxesHeights().size() <= 100)
      cdr.write_type_e(data.getBoundingBoxesHeights());else
          throw new RuntimeException("bounding_boxes_heights field exceeds the maximum length");

      if(data.getLabels().size() <= 100)
      cdr.write_type_e(data.getLabels());else
          throw new RuntimeException("labels field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.BoundingBoxesPacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      cdr.read_type_e(data.getBoundingBoxesXCoordinates());	
      cdr.read_type_e(data.getBoundingBoxesYCoordinates());	
      cdr.read_type_e(data.getBoundingBoxesWidths());	
      cdr.read_type_e(data.getBoundingBoxesHeights());	
      cdr.read_type_e(data.getLabels());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.BoundingBoxesPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_e("bounding_boxes_x_coordinates", data.getBoundingBoxesXCoordinates());
      ser.write_type_e("bounding_boxes_y_coordinates", data.getBoundingBoxesYCoordinates());
      ser.write_type_e("bounding_boxes_widths", data.getBoundingBoxesWidths());
      ser.write_type_e("bounding_boxes_heights", data.getBoundingBoxesHeights());
      ser.write_type_e("labels", data.getLabels());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.BoundingBoxesPacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_e("bounding_boxes_x_coordinates", data.getBoundingBoxesXCoordinates());
      ser.read_type_e("bounding_boxes_y_coordinates", data.getBoundingBoxesYCoordinates());
      ser.read_type_e("bounding_boxes_widths", data.getBoundingBoxesWidths());
      ser.read_type_e("bounding_boxes_heights", data.getBoundingBoxesHeights());
      ser.read_type_e("labels", data.getLabels());
   }

   public static void staticCopy(controller_msgs.msg.dds.BoundingBoxesPacket src, controller_msgs.msg.dds.BoundingBoxesPacket dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.BoundingBoxesPacket createData()
   {
      return new controller_msgs.msg.dds.BoundingBoxesPacket();
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
   
   public void serialize(controller_msgs.msg.dds.BoundingBoxesPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.BoundingBoxesPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.BoundingBoxesPacket src, controller_msgs.msg.dds.BoundingBoxesPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public BoundingBoxesPacketPubSubType newInstance()
   {
      return new BoundingBoxesPacketPubSubType();
   }
}
