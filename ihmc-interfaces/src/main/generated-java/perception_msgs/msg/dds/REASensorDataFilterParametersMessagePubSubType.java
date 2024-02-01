package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "REASensorDataFilterParametersMessage" defined in "REASensorDataFilterParametersMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from REASensorDataFilterParametersMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit REASensorDataFilterParametersMessage_.idl instead.
*
*/
public class REASensorDataFilterParametersMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.REASensorDataFilterParametersMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::REASensorDataFilterParametersMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "71e1ea6be90c363c53506399978b23e1a2ec80becddee848f0839165c11a055e";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.REASensorDataFilterParametersMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.REASensorDataFilterParametersMessage data) throws java.io.IOException
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

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.REASensorDataFilterParametersMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.REASensorDataFilterParametersMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getBoundingBoxMin(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getBoundingBoxMax(), current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.REASensorDataFilterParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      geometry_msgs.msg.dds.PointPubSubType.write(data.getBoundingBoxMin(), cdr);
      geometry_msgs.msg.dds.PointPubSubType.write(data.getBoundingBoxMax(), cdr);
      cdr.write_type_6(data.getSensorMaxRange());

      cdr.write_type_6(data.getSensorMinRange());

   }

   public static void read(perception_msgs.msg.dds.REASensorDataFilterParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      geometry_msgs.msg.dds.PointPubSubType.read(data.getBoundingBoxMin(), cdr);	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getBoundingBoxMax(), cdr);	
      data.setSensorMaxRange(cdr.read_type_6());
      	
      data.setSensorMinRange(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.REASensorDataFilterParametersMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("bounding_box_min", new geometry_msgs.msg.dds.PointPubSubType(), data.getBoundingBoxMin());

      ser.write_type_a("bounding_box_max", new geometry_msgs.msg.dds.PointPubSubType(), data.getBoundingBoxMax());

      ser.write_type_6("sensor_max_range", data.getSensorMaxRange());
      ser.write_type_6("sensor_min_range", data.getSensorMinRange());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.REASensorDataFilterParametersMessage data)
   {
      ser.read_type_a("bounding_box_min", new geometry_msgs.msg.dds.PointPubSubType(), data.getBoundingBoxMin());

      ser.read_type_a("bounding_box_max", new geometry_msgs.msg.dds.PointPubSubType(), data.getBoundingBoxMax());

      data.setSensorMaxRange(ser.read_type_6("sensor_max_range"));
      data.setSensorMinRange(ser.read_type_6("sensor_min_range"));
   }

   public static void staticCopy(perception_msgs.msg.dds.REASensorDataFilterParametersMessage src, perception_msgs.msg.dds.REASensorDataFilterParametersMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.REASensorDataFilterParametersMessage createData()
   {
      return new perception_msgs.msg.dds.REASensorDataFilterParametersMessage();
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
   
   public void serialize(perception_msgs.msg.dds.REASensorDataFilterParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.REASensorDataFilterParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.REASensorDataFilterParametersMessage src, perception_msgs.msg.dds.REASensorDataFilterParametersMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public REASensorDataFilterParametersMessagePubSubType newInstance()
   {
      return new REASensorDataFilterParametersMessagePubSubType();
   }
}
