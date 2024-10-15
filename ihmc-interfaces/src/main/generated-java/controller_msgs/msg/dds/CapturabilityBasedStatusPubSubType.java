package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "CapturabilityBasedStatus" defined in "CapturabilityBasedStatus_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from CapturabilityBasedStatus_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit CapturabilityBasedStatus_.idl instead.
*
*/
public class CapturabilityBasedStatusPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.CapturabilityBasedStatus>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::CapturabilityBasedStatus_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "89bd668d0399ecc46454d943374f50831cba27092687a60eb59eb194d287920a";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.CapturabilityBasedStatus data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.CapturabilityBasedStatus data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 8; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 8; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 1; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 1; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.SpatialVectorMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.SpatialVectorMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.CapturabilityBasedStatus data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.CapturabilityBasedStatus data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getCapturePoint2d(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getDesiredCapturePoint2d(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getCenterOfMass3d(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getLeftFootSupportPolygon3d().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getLeftFootSupportPolygon3d().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getRightFootSupportPolygon3d().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getRightFootSupportPolygon3d().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getLeftHandContactPoints().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getLeftHandContactPoints().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getRightHandContactPoints().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getRightHandContactPoints().get(i0), current_alignment);}

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getLeftHandContactNormal(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getRightHandContactNormal(), current_alignment);

      current_alignment += controller_msgs.msg.dds.SpatialVectorMessagePubSubType.getCdrSerializedSize(data.getRightHandWrench(), current_alignment);

      current_alignment += controller_msgs.msg.dds.SpatialVectorMessagePubSubType.getCdrSerializedSize(data.getLeftHandWrench(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.CapturabilityBasedStatus data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_6(data.getOmega());

      geometry_msgs.msg.dds.PointPubSubType.write(data.getCapturePoint2d(), cdr);
      geometry_msgs.msg.dds.PointPubSubType.write(data.getDesiredCapturePoint2d(), cdr);
      geometry_msgs.msg.dds.PointPubSubType.write(data.getCenterOfMass3d(), cdr);
      if(data.getLeftFootSupportPolygon3d().size() <= 8)
      cdr.write_type_e(data.getLeftFootSupportPolygon3d());else
          throw new RuntimeException("left_foot_support_polygon_3d field exceeds the maximum length");

      if(data.getRightFootSupportPolygon3d().size() <= 8)
      cdr.write_type_e(data.getRightFootSupportPolygon3d());else
          throw new RuntimeException("right_foot_support_polygon_3d field exceeds the maximum length");

      if(data.getLeftHandContactPoints().size() <= 1)
      cdr.write_type_e(data.getLeftHandContactPoints());else
          throw new RuntimeException("left_hand_contact_points field exceeds the maximum length");

      if(data.getRightHandContactPoints().size() <= 1)
      cdr.write_type_e(data.getRightHandContactPoints());else
          throw new RuntimeException("right_hand_contact_points field exceeds the maximum length");

      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getLeftHandContactNormal(), cdr);
      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getRightHandContactNormal(), cdr);
      controller_msgs.msg.dds.SpatialVectorMessagePubSubType.write(data.getRightHandWrench(), cdr);
      controller_msgs.msg.dds.SpatialVectorMessagePubSubType.write(data.getLeftHandWrench(), cdr);
   }

   public static void read(controller_msgs.msg.dds.CapturabilityBasedStatus data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setOmega(cdr.read_type_6());
      	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getCapturePoint2d(), cdr);	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getDesiredCapturePoint2d(), cdr);	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getCenterOfMass3d(), cdr);	
      cdr.read_type_e(data.getLeftFootSupportPolygon3d());	
      cdr.read_type_e(data.getRightFootSupportPolygon3d());	
      cdr.read_type_e(data.getLeftHandContactPoints());	
      cdr.read_type_e(data.getRightHandContactPoints());	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getLeftHandContactNormal(), cdr);	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getRightHandContactNormal(), cdr);	
      controller_msgs.msg.dds.SpatialVectorMessagePubSubType.read(data.getRightHandWrench(), cdr);	
      controller_msgs.msg.dds.SpatialVectorMessagePubSubType.read(data.getLeftHandWrench(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.CapturabilityBasedStatus data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_6("omega", data.getOmega());
      ser.write_type_a("capture_point_2d", new geometry_msgs.msg.dds.PointPubSubType(), data.getCapturePoint2d());

      ser.write_type_a("desired_capture_point_2d", new geometry_msgs.msg.dds.PointPubSubType(), data.getDesiredCapturePoint2d());

      ser.write_type_a("center_of_mass_3d", new geometry_msgs.msg.dds.PointPubSubType(), data.getCenterOfMass3d());

      ser.write_type_e("left_foot_support_polygon_3d", data.getLeftFootSupportPolygon3d());
      ser.write_type_e("right_foot_support_polygon_3d", data.getRightFootSupportPolygon3d());
      ser.write_type_e("left_hand_contact_points", data.getLeftHandContactPoints());
      ser.write_type_e("right_hand_contact_points", data.getRightHandContactPoints());
      ser.write_type_a("left_hand_contact_normal", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getLeftHandContactNormal());

      ser.write_type_a("right_hand_contact_normal", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getRightHandContactNormal());

      ser.write_type_a("right_hand_wrench", new controller_msgs.msg.dds.SpatialVectorMessagePubSubType(), data.getRightHandWrench());

      ser.write_type_a("left_hand_wrench", new controller_msgs.msg.dds.SpatialVectorMessagePubSubType(), data.getLeftHandWrench());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.CapturabilityBasedStatus data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setOmega(ser.read_type_6("omega"));
      ser.read_type_a("capture_point_2d", new geometry_msgs.msg.dds.PointPubSubType(), data.getCapturePoint2d());

      ser.read_type_a("desired_capture_point_2d", new geometry_msgs.msg.dds.PointPubSubType(), data.getDesiredCapturePoint2d());

      ser.read_type_a("center_of_mass_3d", new geometry_msgs.msg.dds.PointPubSubType(), data.getCenterOfMass3d());

      ser.read_type_e("left_foot_support_polygon_3d", data.getLeftFootSupportPolygon3d());
      ser.read_type_e("right_foot_support_polygon_3d", data.getRightFootSupportPolygon3d());
      ser.read_type_e("left_hand_contact_points", data.getLeftHandContactPoints());
      ser.read_type_e("right_hand_contact_points", data.getRightHandContactPoints());
      ser.read_type_a("left_hand_contact_normal", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getLeftHandContactNormal());

      ser.read_type_a("right_hand_contact_normal", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getRightHandContactNormal());

      ser.read_type_a("right_hand_wrench", new controller_msgs.msg.dds.SpatialVectorMessagePubSubType(), data.getRightHandWrench());

      ser.read_type_a("left_hand_wrench", new controller_msgs.msg.dds.SpatialVectorMessagePubSubType(), data.getLeftHandWrench());

   }

   public static void staticCopy(controller_msgs.msg.dds.CapturabilityBasedStatus src, controller_msgs.msg.dds.CapturabilityBasedStatus dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.CapturabilityBasedStatus createData()
   {
      return new controller_msgs.msg.dds.CapturabilityBasedStatus();
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
   
   public void serialize(controller_msgs.msg.dds.CapturabilityBasedStatus data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.CapturabilityBasedStatus data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.CapturabilityBasedStatus src, controller_msgs.msg.dds.CapturabilityBasedStatus dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public CapturabilityBasedStatusPubSubType newInstance()
   {
      return new CapturabilityBasedStatusPubSubType();
   }
}
