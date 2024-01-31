package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FramePlanarRegionsListMessage" defined in "FramePlanarRegionsListMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FramePlanarRegionsListMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FramePlanarRegionsListMessage_.idl instead.
*
*/
public class FramePlanarRegionsListMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.FramePlanarRegionsListMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::FramePlanarRegionsListMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "2d5436a211b398c63f08cb5c52aba1e39a84b4b88f2ea527d822be9c452d942e";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.FramePlanarRegionsListMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.FramePlanarRegionsListMessage data) throws java.io.IOException
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

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += perception_msgs.msg.dds.PlanarRegionsListMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.FramePlanarRegionsListMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.FramePlanarRegionsListMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getSensorPosition(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getSensorOrientation(), current_alignment);

      current_alignment += perception_msgs.msg.dds.PlanarRegionsListMessagePubSubType.getCdrSerializedSize(data.getPlanarRegions(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.FramePlanarRegionsListMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      geometry_msgs.msg.dds.PointPubSubType.write(data.getSensorPosition(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getSensorOrientation(), cdr);
      perception_msgs.msg.dds.PlanarRegionsListMessagePubSubType.write(data.getPlanarRegions(), cdr);
   }

   public static void read(perception_msgs.msg.dds.FramePlanarRegionsListMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getSensorPosition(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getSensorOrientation(), cdr);	
      perception_msgs.msg.dds.PlanarRegionsListMessagePubSubType.read(data.getPlanarRegions(), cdr);	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.FramePlanarRegionsListMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_a("sensor_position", new geometry_msgs.msg.dds.PointPubSubType(), data.getSensorPosition());

      ser.write_type_a("sensor_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getSensorOrientation());

      ser.write_type_a("planar_regions", new perception_msgs.msg.dds.PlanarRegionsListMessagePubSubType(), data.getPlanarRegions());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.FramePlanarRegionsListMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_a("sensor_position", new geometry_msgs.msg.dds.PointPubSubType(), data.getSensorPosition());

      ser.read_type_a("sensor_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getSensorOrientation());

      ser.read_type_a("planar_regions", new perception_msgs.msg.dds.PlanarRegionsListMessagePubSubType(), data.getPlanarRegions());

   }

   public static void staticCopy(perception_msgs.msg.dds.FramePlanarRegionsListMessage src, perception_msgs.msg.dds.FramePlanarRegionsListMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.FramePlanarRegionsListMessage createData()
   {
      return new perception_msgs.msg.dds.FramePlanarRegionsListMessage();
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
   
   public void serialize(perception_msgs.msg.dds.FramePlanarRegionsListMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.FramePlanarRegionsListMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.FramePlanarRegionsListMessage src, perception_msgs.msg.dds.FramePlanarRegionsListMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FramePlanarRegionsListMessagePubSubType newInstance()
   {
      return new FramePlanarRegionsListMessagePubSubType();
   }
}
