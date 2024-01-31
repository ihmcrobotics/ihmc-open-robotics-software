package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "RequestPlanarRegionsListMessage" defined in "RequestPlanarRegionsListMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from RequestPlanarRegionsListMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit RequestPlanarRegionsListMessage_.idl instead.
*
*/
public class RequestPlanarRegionsListMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.RequestPlanarRegionsListMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::RequestPlanarRegionsListMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "4edcd9d8270fe46ce11c978b80445efb12803dcec0993b3dcc4d6d1d878e5dea";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.RequestPlanarRegionsListMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.RequestPlanarRegionsListMessage data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += ihmc_common_msgs.msg.dds.BoundingBox3DMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.RequestPlanarRegionsListMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.RequestPlanarRegionsListMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += ihmc_common_msgs.msg.dds.BoundingBox3DMessagePubSubType.getCdrSerializedSize(data.getBoundingBoxInWorldForRequest(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.RequestPlanarRegionsListMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getPlanarRegionsRequestType());

      ihmc_common_msgs.msg.dds.BoundingBox3DMessagePubSubType.write(data.getBoundingBoxInWorldForRequest(), cdr);
   }

   public static void read(perception_msgs.msg.dds.RequestPlanarRegionsListMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setPlanarRegionsRequestType(cdr.read_type_9());
      	
      ihmc_common_msgs.msg.dds.BoundingBox3DMessagePubSubType.read(data.getBoundingBoxInWorldForRequest(), cdr);	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.RequestPlanarRegionsListMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("planar_regions_request_type", data.getPlanarRegionsRequestType());
      ser.write_type_a("bounding_box_in_world_for_request", new ihmc_common_msgs.msg.dds.BoundingBox3DMessagePubSubType(), data.getBoundingBoxInWorldForRequest());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.RequestPlanarRegionsListMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setPlanarRegionsRequestType(ser.read_type_9("planar_regions_request_type"));
      ser.read_type_a("bounding_box_in_world_for_request", new ihmc_common_msgs.msg.dds.BoundingBox3DMessagePubSubType(), data.getBoundingBoxInWorldForRequest());

   }

   public static void staticCopy(perception_msgs.msg.dds.RequestPlanarRegionsListMessage src, perception_msgs.msg.dds.RequestPlanarRegionsListMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.RequestPlanarRegionsListMessage createData()
   {
      return new perception_msgs.msg.dds.RequestPlanarRegionsListMessage();
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
   
   public void serialize(perception_msgs.msg.dds.RequestPlanarRegionsListMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.RequestPlanarRegionsListMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.RequestPlanarRegionsListMessage src, perception_msgs.msg.dds.RequestPlanarRegionsListMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public RequestPlanarRegionsListMessagePubSubType newInstance()
   {
      return new RequestPlanarRegionsListMessagePubSubType();
   }
}
