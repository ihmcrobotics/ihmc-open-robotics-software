package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FootstepPostProcessingPacket" defined in "FootstepPostProcessingPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FootstepPostProcessingPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FootstepPostProcessingPacket_.idl instead.
*
*/
public class FootstepPostProcessingPacketPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.FootstepPostProcessingPacket>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::FootstepPostProcessingPacket_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.FootstepPostProcessingPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.FootstepPostProcessingPacket data) throws java.io.IOException
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

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 10; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 10; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += controller_msgs.msg.dds.FootstepDataListMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FootstepPostProcessingPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FootstepPostProcessingPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getLeftFootPositionInWorld(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getRightFootPositionInWorld(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getLeftFootOrientationInWorld(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getRightFootOrientationInWorld(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getLeftFootContactPoints2d().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getLeftFootContactPoints2d().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getRightFootContactPoints2d().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getRightFootContactPoints2d().get(i0), current_alignment);}

      current_alignment += controller_msgs.msg.dds.FootstepDataListMessagePubSubType.getCdrSerializedSize(data.getFootstepDataList(), current_alignment);

      current_alignment += controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType.getCdrSerializedSize(data.getPlanarRegionsList(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.FootstepPostProcessingPacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      geometry_msgs.msg.dds.PointPubSubType.write(data.getLeftFootPositionInWorld(), cdr);
      geometry_msgs.msg.dds.PointPubSubType.write(data.getRightFootPositionInWorld(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getLeftFootOrientationInWorld(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getRightFootOrientationInWorld(), cdr);
      if(data.getLeftFootContactPoints2d().size() <= 10)
      cdr.write_type_e(data.getLeftFootContactPoints2d());else
          throw new RuntimeException("left_foot_contact_points_2d field exceeds the maximum length");

      if(data.getRightFootContactPoints2d().size() <= 10)
      cdr.write_type_e(data.getRightFootContactPoints2d());else
          throw new RuntimeException("right_foot_contact_points_2d field exceeds the maximum length");

      controller_msgs.msg.dds.FootstepDataListMessagePubSubType.write(data.getFootstepDataList(), cdr);
      controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType.write(data.getPlanarRegionsList(), cdr);
   }

   public static void read(controller_msgs.msg.dds.FootstepPostProcessingPacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getLeftFootPositionInWorld(), cdr);	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getRightFootPositionInWorld(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getLeftFootOrientationInWorld(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getRightFootOrientationInWorld(), cdr);	
      cdr.read_type_e(data.getLeftFootContactPoints2d());	
      cdr.read_type_e(data.getRightFootContactPoints2d());	
      controller_msgs.msg.dds.FootstepDataListMessagePubSubType.read(data.getFootstepDataList(), cdr);	
      controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType.read(data.getPlanarRegionsList(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.FootstepPostProcessingPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_a("left_foot_position_in_world", new geometry_msgs.msg.dds.PointPubSubType(), data.getLeftFootPositionInWorld());

      ser.write_type_a("right_foot_position_in_world", new geometry_msgs.msg.dds.PointPubSubType(), data.getRightFootPositionInWorld());

      ser.write_type_a("left_foot_orientation_in_world", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getLeftFootOrientationInWorld());

      ser.write_type_a("right_foot_orientation_in_world", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getRightFootOrientationInWorld());

      ser.write_type_e("left_foot_contact_points_2d", data.getLeftFootContactPoints2d());
      ser.write_type_e("right_foot_contact_points_2d", data.getRightFootContactPoints2d());
      ser.write_type_a("footstep_data_list", new controller_msgs.msg.dds.FootstepDataListMessagePubSubType(), data.getFootstepDataList());

      ser.write_type_a("planar_regions_list", new controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType(), data.getPlanarRegionsList());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.FootstepPostProcessingPacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_a("left_foot_position_in_world", new geometry_msgs.msg.dds.PointPubSubType(), data.getLeftFootPositionInWorld());

      ser.read_type_a("right_foot_position_in_world", new geometry_msgs.msg.dds.PointPubSubType(), data.getRightFootPositionInWorld());

      ser.read_type_a("left_foot_orientation_in_world", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getLeftFootOrientationInWorld());

      ser.read_type_a("right_foot_orientation_in_world", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getRightFootOrientationInWorld());

      ser.read_type_e("left_foot_contact_points_2d", data.getLeftFootContactPoints2d());
      ser.read_type_e("right_foot_contact_points_2d", data.getRightFootContactPoints2d());
      ser.read_type_a("footstep_data_list", new controller_msgs.msg.dds.FootstepDataListMessagePubSubType(), data.getFootstepDataList());

      ser.read_type_a("planar_regions_list", new controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType(), data.getPlanarRegionsList());

   }

   public static void staticCopy(controller_msgs.msg.dds.FootstepPostProcessingPacket src, controller_msgs.msg.dds.FootstepPostProcessingPacket dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.FootstepPostProcessingPacket createData()
   {
      return new controller_msgs.msg.dds.FootstepPostProcessingPacket();
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
   
   public void serialize(controller_msgs.msg.dds.FootstepPostProcessingPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.FootstepPostProcessingPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.FootstepPostProcessingPacket src, controller_msgs.msg.dds.FootstepPostProcessingPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FootstepPostProcessingPacketPubSubType newInstance()
   {
      return new FootstepPostProcessingPacketPubSubType();
   }
}
