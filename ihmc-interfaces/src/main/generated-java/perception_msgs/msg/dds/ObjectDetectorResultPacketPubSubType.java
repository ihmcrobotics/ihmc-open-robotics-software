package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ObjectDetectorResultPacket" defined in "ObjectDetectorResultPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ObjectDetectorResultPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ObjectDetectorResultPacket_.idl instead.
*
*/
public class ObjectDetectorResultPacketPubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.ObjectDetectorResultPacket>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::ObjectDetectorResultPacket_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "d2a69bf3012907d2df4e409d54800f020bbcd34fd1af913048aafc6afd07ec6e";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.ObjectDetectorResultPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.ObjectDetectorResultPacket data) throws java.io.IOException
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

      current_alignment += perception_msgs.msg.dds.HeatMapPacketPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.BoundingBoxesPacketPubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.ObjectDetectorResultPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.ObjectDetectorResultPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += perception_msgs.msg.dds.HeatMapPacketPubSubType.getCdrSerializedSize(data.getHeatMap(), current_alignment);

      current_alignment += controller_msgs.msg.dds.BoundingBoxesPacketPubSubType.getCdrSerializedSize(data.getBoundingBoxes(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.ObjectDetectorResultPacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      perception_msgs.msg.dds.HeatMapPacketPubSubType.write(data.getHeatMap(), cdr);
      controller_msgs.msg.dds.BoundingBoxesPacketPubSubType.write(data.getBoundingBoxes(), cdr);
   }

   public static void read(perception_msgs.msg.dds.ObjectDetectorResultPacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      perception_msgs.msg.dds.HeatMapPacketPubSubType.read(data.getHeatMap(), cdr);	
      controller_msgs.msg.dds.BoundingBoxesPacketPubSubType.read(data.getBoundingBoxes(), cdr);	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.ObjectDetectorResultPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_a("heat_map", new perception_msgs.msg.dds.HeatMapPacketPubSubType(), data.getHeatMap());

      ser.write_type_a("bounding_boxes", new controller_msgs.msg.dds.BoundingBoxesPacketPubSubType(), data.getBoundingBoxes());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.ObjectDetectorResultPacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_a("heat_map", new perception_msgs.msg.dds.HeatMapPacketPubSubType(), data.getHeatMap());

      ser.read_type_a("bounding_boxes", new controller_msgs.msg.dds.BoundingBoxesPacketPubSubType(), data.getBoundingBoxes());

   }

   public static void staticCopy(perception_msgs.msg.dds.ObjectDetectorResultPacket src, perception_msgs.msg.dds.ObjectDetectorResultPacket dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.ObjectDetectorResultPacket createData()
   {
      return new perception_msgs.msg.dds.ObjectDetectorResultPacket();
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
   
   public void serialize(perception_msgs.msg.dds.ObjectDetectorResultPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.ObjectDetectorResultPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.ObjectDetectorResultPacket src, perception_msgs.msg.dds.ObjectDetectorResultPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ObjectDetectorResultPacketPubSubType newInstance()
   {
      return new ObjectDetectorResultPacketPubSubType();
   }
}
