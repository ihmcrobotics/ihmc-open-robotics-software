package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ExternalForceEstimationConfigurationMessage" defined in "ExternalForceEstimationConfigurationMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ExternalForceEstimationConfigurationMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ExternalForceEstimationConfigurationMessage_.idl instead.
*
*/
public class ExternalForceEstimationConfigurationMessagePubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.ExternalForceEstimationConfigurationMessage>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::ExternalForceEstimationConfigurationMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "1cf66818d79ba2698e9270cc6ca538bcf7223057dcd605d4acdafe57eaa75a24";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.ExternalForceEstimationConfigurationMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.ExternalForceEstimationConfigurationMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (10 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 10; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.ExternalForceEstimationConfigurationMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.ExternalForceEstimationConfigurationMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getRigidBodyHashCodes().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getContactPointPositions().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getContactPointPositions().get(i0), current_alignment);}

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.ExternalForceEstimationConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_6(data.getEstimatorGain());

      cdr.write_type_6(data.getSolverAlpha());

      cdr.write_type_7(data.getCalculateRootJointWrench());

      if(data.getRigidBodyHashCodes().size() <= 10)
      cdr.write_type_e(data.getRigidBodyHashCodes());else
          throw new RuntimeException("rigid_body_hash_codes field exceeds the maximum length");

      if(data.getContactPointPositions().size() <= 10)
      cdr.write_type_e(data.getContactPointPositions());else
          throw new RuntimeException("contact_point_positions field exceeds the maximum length");

      cdr.write_type_7(data.getEstimateContactLocation());

   }

   public static void read(toolbox_msgs.msg.dds.ExternalForceEstimationConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setEstimatorGain(cdr.read_type_6());
      	
      data.setSolverAlpha(cdr.read_type_6());
      	
      data.setCalculateRootJointWrench(cdr.read_type_7());
      	
      cdr.read_type_e(data.getRigidBodyHashCodes());	
      cdr.read_type_e(data.getContactPointPositions());	
      data.setEstimateContactLocation(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.ExternalForceEstimationConfigurationMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_6("estimator_gain", data.getEstimatorGain());
      ser.write_type_6("solver_alpha", data.getSolverAlpha());
      ser.write_type_7("calculate_root_joint_wrench", data.getCalculateRootJointWrench());
      ser.write_type_e("rigid_body_hash_codes", data.getRigidBodyHashCodes());
      ser.write_type_e("contact_point_positions", data.getContactPointPositions());
      ser.write_type_7("estimate_contact_location", data.getEstimateContactLocation());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.ExternalForceEstimationConfigurationMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setEstimatorGain(ser.read_type_6("estimator_gain"));
      data.setSolverAlpha(ser.read_type_6("solver_alpha"));
      data.setCalculateRootJointWrench(ser.read_type_7("calculate_root_joint_wrench"));
      ser.read_type_e("rigid_body_hash_codes", data.getRigidBodyHashCodes());
      ser.read_type_e("contact_point_positions", data.getContactPointPositions());
      data.setEstimateContactLocation(ser.read_type_7("estimate_contact_location"));
   }

   public static void staticCopy(toolbox_msgs.msg.dds.ExternalForceEstimationConfigurationMessage src, toolbox_msgs.msg.dds.ExternalForceEstimationConfigurationMessage dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.ExternalForceEstimationConfigurationMessage createData()
   {
      return new toolbox_msgs.msg.dds.ExternalForceEstimationConfigurationMessage();
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
   
   public void serialize(toolbox_msgs.msg.dds.ExternalForceEstimationConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.ExternalForceEstimationConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.ExternalForceEstimationConfigurationMessage src, toolbox_msgs.msg.dds.ExternalForceEstimationConfigurationMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ExternalForceEstimationConfigurationMessagePubSubType newInstance()
   {
      return new ExternalForceEstimationConfigurationMessagePubSubType();
   }
}
