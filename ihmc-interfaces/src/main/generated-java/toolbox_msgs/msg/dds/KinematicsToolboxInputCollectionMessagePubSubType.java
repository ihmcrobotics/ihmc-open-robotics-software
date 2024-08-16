package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "KinematicsToolboxInputCollectionMessage" defined in "KinematicsToolboxInputCollectionMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from KinematicsToolboxInputCollectionMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit KinematicsToolboxInputCollectionMessage_.idl instead.
*
*/
public class KinematicsToolboxInputCollectionMessagePubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.KinematicsToolboxInputCollectionMessage>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::KinematicsToolboxInputCollectionMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "c769e55d2ae133bfca9f1ae369e9e6e0145ed274fce8aba2a65ab8345b23e8f5";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.KinematicsToolboxInputCollectionMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.KinematicsToolboxInputCollectionMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 3; ++i0)
      {
          current_alignment += toolbox_msgs.msg.dds.KinematicsToolboxCenterOfMassMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 20; ++i0)
      {
          current_alignment += toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 20; ++i0)
      {
          current_alignment += toolbox_msgs.msg.dds.KinematicsToolboxOneDoFJointMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 1; ++i0)
      {
          current_alignment += toolbox_msgs.msg.dds.KinematicsToolboxSupportRegionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.KinematicsToolboxInputCollectionMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.KinematicsToolboxInputCollectionMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getCenterOfMassInputs().size(); ++i0)
      {
          current_alignment += toolbox_msgs.msg.dds.KinematicsToolboxCenterOfMassMessagePubSubType.getCdrSerializedSize(data.getCenterOfMassInputs().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getRigidBodyInputs().size(); ++i0)
      {
          current_alignment += toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessagePubSubType.getCdrSerializedSize(data.getRigidBodyInputs().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getJointInputs().size(); ++i0)
      {
          current_alignment += toolbox_msgs.msg.dds.KinematicsToolboxOneDoFJointMessagePubSubType.getCdrSerializedSize(data.getJointInputs().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getContactStateInput().size(); ++i0)
      {
          current_alignment += toolbox_msgs.msg.dds.KinematicsToolboxSupportRegionMessagePubSubType.getCdrSerializedSize(data.getContactStateInput().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.KinematicsToolboxInputCollectionMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      if(data.getCenterOfMassInputs().size() <= 3)
      cdr.write_type_e(data.getCenterOfMassInputs());else
          throw new RuntimeException("center_of_mass_inputs field exceeds the maximum length");

      if(data.getRigidBodyInputs().size() <= 20)
      cdr.write_type_e(data.getRigidBodyInputs());else
          throw new RuntimeException("rigid_body_inputs field exceeds the maximum length");

      if(data.getJointInputs().size() <= 20)
      cdr.write_type_e(data.getJointInputs());else
          throw new RuntimeException("joint_inputs field exceeds the maximum length");

      if(data.getContactStateInput().size() <= 1)
      cdr.write_type_e(data.getContactStateInput());else
          throw new RuntimeException("contact_state_input field exceeds the maximum length");

   }

   public static void read(toolbox_msgs.msg.dds.KinematicsToolboxInputCollectionMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      cdr.read_type_e(data.getCenterOfMassInputs());	
      cdr.read_type_e(data.getRigidBodyInputs());	
      cdr.read_type_e(data.getJointInputs());	
      cdr.read_type_e(data.getContactStateInput());	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.KinematicsToolboxInputCollectionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_e("center_of_mass_inputs", data.getCenterOfMassInputs());
      ser.write_type_e("rigid_body_inputs", data.getRigidBodyInputs());
      ser.write_type_e("joint_inputs", data.getJointInputs());
      ser.write_type_e("contact_state_input", data.getContactStateInput());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.KinematicsToolboxInputCollectionMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_e("center_of_mass_inputs", data.getCenterOfMassInputs());
      ser.read_type_e("rigid_body_inputs", data.getRigidBodyInputs());
      ser.read_type_e("joint_inputs", data.getJointInputs());
      ser.read_type_e("contact_state_input", data.getContactStateInput());
   }

   public static void staticCopy(toolbox_msgs.msg.dds.KinematicsToolboxInputCollectionMessage src, toolbox_msgs.msg.dds.KinematicsToolboxInputCollectionMessage dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.KinematicsToolboxInputCollectionMessage createData()
   {
      return new toolbox_msgs.msg.dds.KinematicsToolboxInputCollectionMessage();
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
   
   public void serialize(toolbox_msgs.msg.dds.KinematicsToolboxInputCollectionMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.KinematicsToolboxInputCollectionMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.KinematicsToolboxInputCollectionMessage src, toolbox_msgs.msg.dds.KinematicsToolboxInputCollectionMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public KinematicsToolboxInputCollectionMessagePubSubType newInstance()
   {
      return new KinematicsToolboxInputCollectionMessagePubSubType();
   }
}
