package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ExternalForceEstimationOutputStatus" defined in "ExternalForceEstimationOutputStatus_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ExternalForceEstimationOutputStatus_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ExternalForceEstimationOutputStatus_.idl instead.
*
*/
public class ExternalForceEstimationOutputStatusPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.ExternalForceEstimationOutputStatus>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::ExternalForceEstimationOutputStatus_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.ExternalForceEstimationOutputStatus data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.ExternalForceEstimationOutputStatus data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 10; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += geometry_msgs.msg.dds.WrenchPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ExternalForceEstimationOutputStatus data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ExternalForceEstimationOutputStatus data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getEstimatedExternalForces().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getEstimatedExternalForces().get(i0), current_alignment);}

      current_alignment += geometry_msgs.msg.dds.WrenchPubSubType.getCdrSerializedSize(data.getEstimatedRootJointWrench(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getContactPoint(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.ExternalForceEstimationOutputStatus data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      if(data.getEstimatedExternalForces().size() <= 10)
      cdr.write_type_e(data.getEstimatedExternalForces());else
          throw new RuntimeException("estimated_external_forces field exceeds the maximum length");

      geometry_msgs.msg.dds.WrenchPubSubType.write(data.getEstimatedRootJointWrench(), cdr);
      cdr.write_type_2(data.getRigidBodyHashCode());

      geometry_msgs.msg.dds.PointPubSubType.write(data.getContactPoint(), cdr);
   }

   public static void read(controller_msgs.msg.dds.ExternalForceEstimationOutputStatus data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      cdr.read_type_e(data.getEstimatedExternalForces());	
      geometry_msgs.msg.dds.WrenchPubSubType.read(data.getEstimatedRootJointWrench(), cdr);	
      data.setRigidBodyHashCode(cdr.read_type_2());
      	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getContactPoint(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.ExternalForceEstimationOutputStatus data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_e("estimated_external_forces", data.getEstimatedExternalForces());
      ser.write_type_a("estimated_root_joint_wrench", new geometry_msgs.msg.dds.WrenchPubSubType(), data.getEstimatedRootJointWrench());

      ser.write_type_2("rigid_body_hash_code", data.getRigidBodyHashCode());
      ser.write_type_a("contact_point", new geometry_msgs.msg.dds.PointPubSubType(), data.getContactPoint());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.ExternalForceEstimationOutputStatus data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_e("estimated_external_forces", data.getEstimatedExternalForces());
      ser.read_type_a("estimated_root_joint_wrench", new geometry_msgs.msg.dds.WrenchPubSubType(), data.getEstimatedRootJointWrench());

      data.setRigidBodyHashCode(ser.read_type_2("rigid_body_hash_code"));
      ser.read_type_a("contact_point", new geometry_msgs.msg.dds.PointPubSubType(), data.getContactPoint());

   }

   public static void staticCopy(controller_msgs.msg.dds.ExternalForceEstimationOutputStatus src, controller_msgs.msg.dds.ExternalForceEstimationOutputStatus dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.ExternalForceEstimationOutputStatus createData()
   {
      return new controller_msgs.msg.dds.ExternalForceEstimationOutputStatus();
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
   
   public void serialize(controller_msgs.msg.dds.ExternalForceEstimationOutputStatus data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.ExternalForceEstimationOutputStatus data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.ExternalForceEstimationOutputStatus src, controller_msgs.msg.dds.ExternalForceEstimationOutputStatus dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ExternalForceEstimationOutputStatusPubSubType newInstance()
   {
      return new ExternalForceEstimationOutputStatusPubSubType();
   }
}
