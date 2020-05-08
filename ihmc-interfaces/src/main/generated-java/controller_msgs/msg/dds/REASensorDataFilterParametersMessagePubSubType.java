package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "REASensorDataFilterParametersMessage" defined in "REASensorDataFilterParametersMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from REASensorDataFilterParametersMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit REASensorDataFilterParametersMessage_.idl instead.
*
*/
public class REASensorDataFilterParametersMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.REASensorDataFilterParametersMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::REASensorDataFilterParametersMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.REASensorDataFilterParametersMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.REASensorDataFilterParametersMessage data) throws java.io.IOException
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

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.REASensorDataFilterParametersMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.REASensorDataFilterParametersMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getBoundingBoxMin(), current_alignment);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getBoundingBoxMax(), current_alignment);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.REASensorDataFilterParametersMessage data, us.ihmc.idl.CDR cdr)
   {

      geometry_msgs.msg.dds.PointPubSubType.write(data.getBoundingBoxMin(), cdr);

      geometry_msgs.msg.dds.PointPubSubType.write(data.getBoundingBoxMax(), cdr);

      cdr.write_type_6(data.getSensorMaxRange());


      cdr.write_type_6(data.getSensorMinRange());

   }

   public static void read(controller_msgs.msg.dds.REASensorDataFilterParametersMessage data, us.ihmc.idl.CDR cdr)
   {

      geometry_msgs.msg.dds.PointPubSubType.read(data.getBoundingBoxMin(), cdr);	

      geometry_msgs.msg.dds.PointPubSubType.read(data.getBoundingBoxMax(), cdr);	

      data.setSensorMaxRange(cdr.read_type_6());
      	

      data.setSensorMinRange(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.REASensorDataFilterParametersMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {

      ser.write_type_a("bounding_box_min", new geometry_msgs.msg.dds.PointPubSubType(), data.getBoundingBoxMin());


      ser.write_type_a("bounding_box_max", new geometry_msgs.msg.dds.PointPubSubType(), data.getBoundingBoxMax());


      ser.write_type_6("sensor_max_range", data.getSensorMaxRange());

      ser.write_type_6("sensor_min_range", data.getSensorMinRange());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.REASensorDataFilterParametersMessage data)
   {

      ser.read_type_a("bounding_box_min", new geometry_msgs.msg.dds.PointPubSubType(), data.getBoundingBoxMin());


      ser.read_type_a("bounding_box_max", new geometry_msgs.msg.dds.PointPubSubType(), data.getBoundingBoxMax());


      data.setSensorMaxRange(ser.read_type_6("sensor_max_range"));

      data.setSensorMinRange(ser.read_type_6("sensor_min_range"));
   }

   public static void staticCopy(controller_msgs.msg.dds.REASensorDataFilterParametersMessage src, controller_msgs.msg.dds.REASensorDataFilterParametersMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.REASensorDataFilterParametersMessage createData()
   {
      return new controller_msgs.msg.dds.REASensorDataFilterParametersMessage();
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
   
   public void serialize(controller_msgs.msg.dds.REASensorDataFilterParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.REASensorDataFilterParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.REASensorDataFilterParametersMessage src, controller_msgs.msg.dds.REASensorDataFilterParametersMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public REASensorDataFilterParametersMessagePubSubType newInstance()
   {
      return new REASensorDataFilterParametersMessagePubSubType();
   }
}
