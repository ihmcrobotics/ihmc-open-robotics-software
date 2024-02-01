package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "KinematicsToolboxPrivilegedConfigurationMessage" defined in "KinematicsToolboxPrivilegedConfigurationMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from KinematicsToolboxPrivilegedConfigurationMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit KinematicsToolboxPrivilegedConfigurationMessage_.idl instead.
*
*/
public class KinematicsToolboxPrivilegedConfigurationMessagePubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.KinematicsToolboxPrivilegedConfigurationMessage>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::KinematicsToolboxPrivilegedConfigurationMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "0911e08f380e577807757343785ed18699965563fcf46a870a259013e85f4b42";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.KinematicsToolboxPrivilegedConfigurationMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.KinematicsToolboxPrivilegedConfigurationMessage data) throws java.io.IOException
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

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.KinematicsToolboxPrivilegedConfigurationMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.KinematicsToolboxPrivilegedConfigurationMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getPrivilegedRootJointPosition(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getPrivilegedRootJointOrientation(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getPrivilegedJointHashCodes().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getPrivilegedJointAngles().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.KinematicsToolboxPrivilegedConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_7(data.getUsePrivilegedRootJointPosition());

      cdr.write_type_7(data.getUsePrivilegedRootJointOrientation());

      geometry_msgs.msg.dds.PointPubSubType.write(data.getPrivilegedRootJointPosition(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getPrivilegedRootJointOrientation(), cdr);
      if(data.getPrivilegedJointHashCodes().size() <= 100)
      cdr.write_type_e(data.getPrivilegedJointHashCodes());else
          throw new RuntimeException("privileged_joint_hash_codes field exceeds the maximum length");

      if(data.getPrivilegedJointAngles().size() <= 100)
      cdr.write_type_e(data.getPrivilegedJointAngles());else
          throw new RuntimeException("privileged_joint_angles field exceeds the maximum length");

      cdr.write_type_6(data.getPrivilegedWeight());

      cdr.write_type_6(data.getPrivilegedGain());

   }

   public static void read(toolbox_msgs.msg.dds.KinematicsToolboxPrivilegedConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setUsePrivilegedRootJointPosition(cdr.read_type_7());
      	
      data.setUsePrivilegedRootJointOrientation(cdr.read_type_7());
      	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getPrivilegedRootJointPosition(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getPrivilegedRootJointOrientation(), cdr);	
      cdr.read_type_e(data.getPrivilegedJointHashCodes());	
      cdr.read_type_e(data.getPrivilegedJointAngles());	
      data.setPrivilegedWeight(cdr.read_type_6());
      	
      data.setPrivilegedGain(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.KinematicsToolboxPrivilegedConfigurationMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_7("use_privileged_root_joint_position", data.getUsePrivilegedRootJointPosition());
      ser.write_type_7("use_privileged_root_joint_orientation", data.getUsePrivilegedRootJointOrientation());
      ser.write_type_a("privileged_root_joint_position", new geometry_msgs.msg.dds.PointPubSubType(), data.getPrivilegedRootJointPosition());

      ser.write_type_a("privileged_root_joint_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getPrivilegedRootJointOrientation());

      ser.write_type_e("privileged_joint_hash_codes", data.getPrivilegedJointHashCodes());
      ser.write_type_e("privileged_joint_angles", data.getPrivilegedJointAngles());
      ser.write_type_6("privileged_weight", data.getPrivilegedWeight());
      ser.write_type_6("privileged_gain", data.getPrivilegedGain());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.KinematicsToolboxPrivilegedConfigurationMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setUsePrivilegedRootJointPosition(ser.read_type_7("use_privileged_root_joint_position"));
      data.setUsePrivilegedRootJointOrientation(ser.read_type_7("use_privileged_root_joint_orientation"));
      ser.read_type_a("privileged_root_joint_position", new geometry_msgs.msg.dds.PointPubSubType(), data.getPrivilegedRootJointPosition());

      ser.read_type_a("privileged_root_joint_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getPrivilegedRootJointOrientation());

      ser.read_type_e("privileged_joint_hash_codes", data.getPrivilegedJointHashCodes());
      ser.read_type_e("privileged_joint_angles", data.getPrivilegedJointAngles());
      data.setPrivilegedWeight(ser.read_type_6("privileged_weight"));
      data.setPrivilegedGain(ser.read_type_6("privileged_gain"));
   }

   public static void staticCopy(toolbox_msgs.msg.dds.KinematicsToolboxPrivilegedConfigurationMessage src, toolbox_msgs.msg.dds.KinematicsToolboxPrivilegedConfigurationMessage dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.KinematicsToolboxPrivilegedConfigurationMessage createData()
   {
      return new toolbox_msgs.msg.dds.KinematicsToolboxPrivilegedConfigurationMessage();
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
   
   public void serialize(toolbox_msgs.msg.dds.KinematicsToolboxPrivilegedConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.KinematicsToolboxPrivilegedConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.KinematicsToolboxPrivilegedConfigurationMessage src, toolbox_msgs.msg.dds.KinematicsToolboxPrivilegedConfigurationMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public KinematicsToolboxPrivilegedConfigurationMessagePubSubType newInstance()
   {
      return new KinematicsToolboxPrivilegedConfigurationMessagePubSubType();
   }
}
