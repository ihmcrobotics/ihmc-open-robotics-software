package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "DetectedDoorMessage" defined in "DetectedDoorMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from DetectedDoorMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit DetectedDoorMessage_.idl instead.
*
*/
public class DetectedDoorMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.DetectedDoorMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::DetectedDoorMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "1f28bf94e8ea9d2e71e803e690eacabed2392c8e7b9c7923e3067ff0c11ca2ea";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.DetectedDoorMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.DetectedDoorMessage data) throws java.io.IOException
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

      current_alignment += ihmc_common_msgs.msg.dds.UUIDMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += perception_msgs.msg.dds.DetectedDoorOpeningMechanismMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += perception_msgs.msg.dds.PlanarRegionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += ihmc_common_msgs.msg.dds.InstantMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += ihmc_common_msgs.msg.dds.InstantMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += ihmc_common_msgs.msg.dds.InstantMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.DetectedDoorMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.DetectedDoorMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += ihmc_common_msgs.msg.dds.UUIDMessagePubSubType.getCdrSerializedSize(data.getDetectionUuid(), current_alignment);

      current_alignment += perception_msgs.msg.dds.DetectedDoorOpeningMechanismMessagePubSubType.getCdrSerializedSize(data.getOpeningMechanism(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getPanelPose(), current_alignment);

      current_alignment += perception_msgs.msg.dds.PlanarRegionMessagePubSubType.getCdrSerializedSize(data.getPanelPlanarRegion(), current_alignment);

      current_alignment += ihmc_common_msgs.msg.dds.InstantMessagePubSubType.getCdrSerializedSize(data.getLastDetectionTime(), current_alignment);

      current_alignment += ihmc_common_msgs.msg.dds.InstantMessagePubSubType.getCdrSerializedSize(data.getOpeningMechanismFirstDetectionTime(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += ihmc_common_msgs.msg.dds.InstantMessagePubSubType.getCdrSerializedSize(data.getPanelFirstDetectionTime(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.DetectedDoorMessage data, us.ihmc.idl.CDR cdr)
   {
      ihmc_common_msgs.msg.dds.UUIDMessagePubSubType.write(data.getDetectionUuid(), cdr);
      perception_msgs.msg.dds.DetectedDoorOpeningMechanismMessagePubSubType.write(data.getOpeningMechanism(), cdr);
      geometry_msgs.msg.dds.PosePubSubType.write(data.getPanelPose(), cdr);
      perception_msgs.msg.dds.PlanarRegionMessagePubSubType.write(data.getPanelPlanarRegion(), cdr);
      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.write(data.getLastDetectionTime(), cdr);
      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.write(data.getOpeningMechanismFirstDetectionTime(), cdr);
      cdr.write_type_2(data.getOpeningMechanismDetectionCount());

      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.write(data.getPanelFirstDetectionTime(), cdr);
      cdr.write_type_2(data.getPanelDetectionCount());

   }

   public static void read(perception_msgs.msg.dds.DetectedDoorMessage data, us.ihmc.idl.CDR cdr)
   {
      ihmc_common_msgs.msg.dds.UUIDMessagePubSubType.read(data.getDetectionUuid(), cdr);	
      perception_msgs.msg.dds.DetectedDoorOpeningMechanismMessagePubSubType.read(data.getOpeningMechanism(), cdr);	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getPanelPose(), cdr);	
      perception_msgs.msg.dds.PlanarRegionMessagePubSubType.read(data.getPanelPlanarRegion(), cdr);	
      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.read(data.getLastDetectionTime(), cdr);	
      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.read(data.getOpeningMechanismFirstDetectionTime(), cdr);	
      data.setOpeningMechanismDetectionCount(cdr.read_type_2());
      	
      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.read(data.getPanelFirstDetectionTime(), cdr);	
      data.setPanelDetectionCount(cdr.read_type_2());
      	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.DetectedDoorMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("detection_uuid", new ihmc_common_msgs.msg.dds.UUIDMessagePubSubType(), data.getDetectionUuid());

      ser.write_type_a("opening_mechanism", new perception_msgs.msg.dds.DetectedDoorOpeningMechanismMessagePubSubType(), data.getOpeningMechanism());

      ser.write_type_a("panel_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getPanelPose());

      ser.write_type_a("panel_planar_region", new perception_msgs.msg.dds.PlanarRegionMessagePubSubType(), data.getPanelPlanarRegion());

      ser.write_type_a("last_detection_time", new ihmc_common_msgs.msg.dds.InstantMessagePubSubType(), data.getLastDetectionTime());

      ser.write_type_a("opening_mechanism_first_detection_time", new ihmc_common_msgs.msg.dds.InstantMessagePubSubType(), data.getOpeningMechanismFirstDetectionTime());

      ser.write_type_2("opening_mechanism_detection_count", data.getOpeningMechanismDetectionCount());
      ser.write_type_a("panel_first_detection_time", new ihmc_common_msgs.msg.dds.InstantMessagePubSubType(), data.getPanelFirstDetectionTime());

      ser.write_type_2("panel_detection_count", data.getPanelDetectionCount());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.DetectedDoorMessage data)
   {
      ser.read_type_a("detection_uuid", new ihmc_common_msgs.msg.dds.UUIDMessagePubSubType(), data.getDetectionUuid());

      ser.read_type_a("opening_mechanism", new perception_msgs.msg.dds.DetectedDoorOpeningMechanismMessagePubSubType(), data.getOpeningMechanism());

      ser.read_type_a("panel_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getPanelPose());

      ser.read_type_a("panel_planar_region", new perception_msgs.msg.dds.PlanarRegionMessagePubSubType(), data.getPanelPlanarRegion());

      ser.read_type_a("last_detection_time", new ihmc_common_msgs.msg.dds.InstantMessagePubSubType(), data.getLastDetectionTime());

      ser.read_type_a("opening_mechanism_first_detection_time", new ihmc_common_msgs.msg.dds.InstantMessagePubSubType(), data.getOpeningMechanismFirstDetectionTime());

      data.setOpeningMechanismDetectionCount(ser.read_type_2("opening_mechanism_detection_count"));
      ser.read_type_a("panel_first_detection_time", new ihmc_common_msgs.msg.dds.InstantMessagePubSubType(), data.getPanelFirstDetectionTime());

      data.setPanelDetectionCount(ser.read_type_2("panel_detection_count"));
   }

   public static void staticCopy(perception_msgs.msg.dds.DetectedDoorMessage src, perception_msgs.msg.dds.DetectedDoorMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.DetectedDoorMessage createData()
   {
      return new perception_msgs.msg.dds.DetectedDoorMessage();
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
   
   public void serialize(perception_msgs.msg.dds.DetectedDoorMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.DetectedDoorMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.DetectedDoorMessage src, perception_msgs.msg.dds.DetectedDoorMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public DetectedDoorMessagePubSubType newInstance()
   {
      return new DetectedDoorMessagePubSubType();
   }
}
