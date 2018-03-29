package controller_msgs.msg.dds;

/**
 * Topic data type of the struct "RequestPlanarRegionsListMessage" defined in "RequestPlanarRegionsListMessage_.idl". Use this class to provide the TopicDataType to a Participant.
 *
 * This file was automatically generated from RequestPlanarRegionsListMessage_.idl by us.ihmc.idl.generator.IDLGenerator.
 * Do not update this file directly, edit RequestPlanarRegionsListMessage_.idl instead.
 */
public class RequestPlanarRegionsListMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.RequestPlanarRegionsListMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::RequestPlanarRegionsListMessage_";
   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   public RequestPlanarRegionsListMessagePubSubType()
   {

   }

   public static int getMaxCdrSerializedSize()
   {
      return getMaxCdrSerializedSize(0);
   }

   public static int getMaxCdrSerializedSize(int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += controller_msgs.msg.dds.BoundingBox3DMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.RequestPlanarRegionsListMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.RequestPlanarRegionsListMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += controller_msgs.msg.dds.BoundingBox3DMessagePubSubType
            .getCdrSerializedSize(data.getBoundingBoxInWorldForRequest(), current_alignment);

      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.RequestPlanarRegionsListMessage data, us.ihmc.idl.CDR cdr)
   {

      cdr.write_type_9(data.getPlanarRegionsRequestType());

      controller_msgs.msg.dds.BoundingBox3DMessagePubSubType.write(data.getBoundingBoxInWorldForRequest(), cdr);
   }

   public static void read(controller_msgs.msg.dds.RequestPlanarRegionsListMessage data, us.ihmc.idl.CDR cdr)
   {

      data.setPlanarRegionsRequestType(cdr.read_type_9());

      controller_msgs.msg.dds.BoundingBox3DMessagePubSubType.read(data.getBoundingBoxInWorldForRequest(), cdr);
   }

   public static void staticCopy(controller_msgs.msg.dds.RequestPlanarRegionsListMessage src, controller_msgs.msg.dds.RequestPlanarRegionsListMessage dest)
   {
      dest.set(src);
   }

   @Override
   public void serialize(controller_msgs.msg.dds.RequestPlanarRegionsListMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload)
         throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.RequestPlanarRegionsListMessage data)
         throws java.io.IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      read(data, deserializeCDR);
      deserializeCDR.finishDeserialize();
   }

   @Override
   public final void serialize(controller_msgs.msg.dds.RequestPlanarRegionsListMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_9("planar_regions_request_type", data.getPlanarRegionsRequestType());

      ser.write_type_a("bounding_box_in_world_for_request", new controller_msgs.msg.dds.BoundingBox3DMessagePubSubType(),
                       data.getBoundingBoxInWorldForRequest());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.RequestPlanarRegionsListMessage data)
   {
      data.setPlanarRegionsRequestType(ser.read_type_9("planar_regions_request_type"));

      ser.read_type_a("bounding_box_in_world_for_request", new controller_msgs.msg.dds.BoundingBox3DMessagePubSubType(),
                      data.getBoundingBoxInWorldForRequest());
   }

   @Override
   public controller_msgs.msg.dds.RequestPlanarRegionsListMessage createData()
   {
      return new controller_msgs.msg.dds.RequestPlanarRegionsListMessage();
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

   public void serialize(controller_msgs.msg.dds.RequestPlanarRegionsListMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.RequestPlanarRegionsListMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }

   public void copy(controller_msgs.msg.dds.RequestPlanarRegionsListMessage src, controller_msgs.msg.dds.RequestPlanarRegionsListMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public RequestPlanarRegionsListMessagePubSubType newInstance()
   {
      return new RequestPlanarRegionsListMessagePubSubType();
   }
}
