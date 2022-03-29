package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "MultiContactBalanceStatus" defined in "MultiContactBalanceStatus_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from MultiContactBalanceStatus_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit MultiContactBalanceStatus_.idl instead.
*
*/
public class MultiContactBalanceStatusPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.MultiContactBalanceStatus>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::MultiContactBalanceStatus_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.MultiContactBalanceStatus data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.MultiContactBalanceStatus data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 16; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 16; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (16 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.MultiContactBalanceStatus data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.MultiContactBalanceStatus data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getCapturePoint2d(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getCenterOfMass3d(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getContactPointsInWorld().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getContactPointsInWorld().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getSurfaceNormalsInWorld().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getSurfaceNormalsInWorld().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getSupportRigidBodyIds().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.MultiContactBalanceStatus data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      geometry_msgs.msg.dds.PointPubSubType.write(data.getCapturePoint2d(), cdr);
      geometry_msgs.msg.dds.PointPubSubType.write(data.getCenterOfMass3d(), cdr);
      if(data.getContactPointsInWorld().size() <= 16)
      cdr.write_type_e(data.getContactPointsInWorld());else
          throw new RuntimeException("contact_points_in_world field exceeds the maximum length");

      if(data.getSurfaceNormalsInWorld().size() <= 16)
      cdr.write_type_e(data.getSurfaceNormalsInWorld());else
          throw new RuntimeException("surface_normals_in_world field exceeds the maximum length");

      if(data.getSupportRigidBodyIds().size() <= 16)
      cdr.write_type_e(data.getSupportRigidBodyIds());else
          throw new RuntimeException("support_rigid_body_ids field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.MultiContactBalanceStatus data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getCapturePoint2d(), cdr);	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getCenterOfMass3d(), cdr);	
      cdr.read_type_e(data.getContactPointsInWorld());	
      cdr.read_type_e(data.getSurfaceNormalsInWorld());	
      cdr.read_type_e(data.getSupportRigidBodyIds());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.MultiContactBalanceStatus data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_a("capture_point_2d", new geometry_msgs.msg.dds.PointPubSubType(), data.getCapturePoint2d());

      ser.write_type_a("center_of_mass_3d", new geometry_msgs.msg.dds.PointPubSubType(), data.getCenterOfMass3d());

      ser.write_type_e("contact_points_in_world", data.getContactPointsInWorld());
      ser.write_type_e("surface_normals_in_world", data.getSurfaceNormalsInWorld());
      ser.write_type_e("support_rigid_body_ids", data.getSupportRigidBodyIds());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.MultiContactBalanceStatus data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_a("capture_point_2d", new geometry_msgs.msg.dds.PointPubSubType(), data.getCapturePoint2d());

      ser.read_type_a("center_of_mass_3d", new geometry_msgs.msg.dds.PointPubSubType(), data.getCenterOfMass3d());

      ser.read_type_e("contact_points_in_world", data.getContactPointsInWorld());
      ser.read_type_e("surface_normals_in_world", data.getSurfaceNormalsInWorld());
      ser.read_type_e("support_rigid_body_ids", data.getSupportRigidBodyIds());
   }

   public static void staticCopy(controller_msgs.msg.dds.MultiContactBalanceStatus src, controller_msgs.msg.dds.MultiContactBalanceStatus dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.MultiContactBalanceStatus createData()
   {
      return new controller_msgs.msg.dds.MultiContactBalanceStatus();
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
   
   public void serialize(controller_msgs.msg.dds.MultiContactBalanceStatus data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.MultiContactBalanceStatus data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.MultiContactBalanceStatus src, controller_msgs.msg.dds.MultiContactBalanceStatus dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public MultiContactBalanceStatusPubSubType newInstance()
   {
      return new MultiContactBalanceStatusPubSubType();
   }
}
