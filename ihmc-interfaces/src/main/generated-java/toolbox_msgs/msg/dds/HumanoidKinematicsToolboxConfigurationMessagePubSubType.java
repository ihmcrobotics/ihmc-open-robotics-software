package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "HumanoidKinematicsToolboxConfigurationMessage" defined in "HumanoidKinematicsToolboxConfigurationMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from HumanoidKinematicsToolboxConfigurationMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit HumanoidKinematicsToolboxConfigurationMessage_.idl instead.
*
*/
public class HumanoidKinematicsToolboxConfigurationMessagePubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.HumanoidKinematicsToolboxConfigurationMessage>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::HumanoidKinematicsToolboxConfigurationMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "512c136f0cf0055d0bdd81435509cb1b8c4f12139314e1899c6868499bcb9269";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.HumanoidKinematicsToolboxConfigurationMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.HumanoidKinematicsToolboxConfigurationMessage data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (20 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (20 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.HumanoidKinematicsToolboxConfigurationMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.HumanoidKinematicsToolboxConfigurationMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getJointLimitReductionFactors().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getJointLimitReductionHashCodes().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.HumanoidKinematicsToolboxConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_7(data.getHoldCurrentCenterOfMassXyPosition());

      cdr.write_type_7(data.getEnableAutoSupportPolygon());

      cdr.write_type_7(data.getHoldSupportRigidBodies());

      cdr.write_type_7(data.getEnableMultiContactSupportRegionSolver());

      cdr.write_type_7(data.getEnableJointLimitReduction());

      if(data.getJointLimitReductionFactors().size() <= 20)
      cdr.write_type_e(data.getJointLimitReductionFactors());else
          throw new RuntimeException("joint_limit_reduction_factors field exceeds the maximum length");

      if(data.getJointLimitReductionHashCodes().size() <= 20)
      cdr.write_type_e(data.getJointLimitReductionHashCodes());else
          throw new RuntimeException("joint_limit_reduction_hash_codes field exceeds the maximum length");

   }

   public static void read(toolbox_msgs.msg.dds.HumanoidKinematicsToolboxConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setHoldCurrentCenterOfMassXyPosition(cdr.read_type_7());
      	
      data.setEnableAutoSupportPolygon(cdr.read_type_7());
      	
      data.setHoldSupportRigidBodies(cdr.read_type_7());
      	
      data.setEnableMultiContactSupportRegionSolver(cdr.read_type_7());
      	
      data.setEnableJointLimitReduction(cdr.read_type_7());
      	
      cdr.read_type_e(data.getJointLimitReductionFactors());	
      cdr.read_type_e(data.getJointLimitReductionHashCodes());	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.HumanoidKinematicsToolboxConfigurationMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_7("hold_current_center_of_mass_xy_position", data.getHoldCurrentCenterOfMassXyPosition());
      ser.write_type_7("enable_auto_support_polygon", data.getEnableAutoSupportPolygon());
      ser.write_type_7("hold_support_rigid_bodies", data.getHoldSupportRigidBodies());
      ser.write_type_7("enable_multi_contact_support_region_solver", data.getEnableMultiContactSupportRegionSolver());
      ser.write_type_7("enable_joint_limit_reduction", data.getEnableJointLimitReduction());
      ser.write_type_e("joint_limit_reduction_factors", data.getJointLimitReductionFactors());
      ser.write_type_e("joint_limit_reduction_hash_codes", data.getJointLimitReductionHashCodes());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.HumanoidKinematicsToolboxConfigurationMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setHoldCurrentCenterOfMassXyPosition(ser.read_type_7("hold_current_center_of_mass_xy_position"));
      data.setEnableAutoSupportPolygon(ser.read_type_7("enable_auto_support_polygon"));
      data.setHoldSupportRigidBodies(ser.read_type_7("hold_support_rigid_bodies"));
      data.setEnableMultiContactSupportRegionSolver(ser.read_type_7("enable_multi_contact_support_region_solver"));
      data.setEnableJointLimitReduction(ser.read_type_7("enable_joint_limit_reduction"));
      ser.read_type_e("joint_limit_reduction_factors", data.getJointLimitReductionFactors());
      ser.read_type_e("joint_limit_reduction_hash_codes", data.getJointLimitReductionHashCodes());
   }

   public static void staticCopy(toolbox_msgs.msg.dds.HumanoidKinematicsToolboxConfigurationMessage src, toolbox_msgs.msg.dds.HumanoidKinematicsToolboxConfigurationMessage dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.HumanoidKinematicsToolboxConfigurationMessage createData()
   {
      return new toolbox_msgs.msg.dds.HumanoidKinematicsToolboxConfigurationMessage();
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
   
   public void serialize(toolbox_msgs.msg.dds.HumanoidKinematicsToolboxConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.HumanoidKinematicsToolboxConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.HumanoidKinematicsToolboxConfigurationMessage src, toolbox_msgs.msg.dds.HumanoidKinematicsToolboxConfigurationMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public HumanoidKinematicsToolboxConfigurationMessagePubSubType newInstance()
   {
      return new HumanoidKinematicsToolboxConfigurationMessagePubSubType();
   }
}
