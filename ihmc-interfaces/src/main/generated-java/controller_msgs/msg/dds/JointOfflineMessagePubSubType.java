package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "JointOfflineMessage" defined in "JointOfflineMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from JointOfflineMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit JointOfflineMessage_.idl instead.
*
*/
public class JointOfflineMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.JointOfflineMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::JointOfflineMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "a10a308c8168ad8a82499c21f64535e2e3dc3702eee3c1e26e7f05fb1bc8254c";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.JointOfflineMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.JointOfflineMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (6 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 4; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 4; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.JointOfflineMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.JointOfflineMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getJointOfflineHashCodes().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getNominalLeftFootContactPoints2d().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getNominalLeftFootContactPoints2d().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getNominalRightFootContactPoints2d().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getNominalRightFootContactPoints2d().get(i0), current_alignment);}

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.JointOfflineMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      if(data.getJointOfflineHashCodes().size() <= 6)
      cdr.write_type_e(data.getJointOfflineHashCodes());else
          throw new RuntimeException("joint_offline_hash_codes field exceeds the maximum length");

      if(data.getNominalLeftFootContactPoints2d().size() <= 4)
      cdr.write_type_e(data.getNominalLeftFootContactPoints2d());else
          throw new RuntimeException("nominal_left_foot_contact_points_2d field exceeds the maximum length");

      if(data.getNominalRightFootContactPoints2d().size() <= 4)
      cdr.write_type_e(data.getNominalRightFootContactPoints2d());else
          throw new RuntimeException("nominal_right_foot_contact_points_2d field exceeds the maximum length");

      cdr.write_type_6(data.getExecutionDelayTime());

   }

   public static void read(controller_msgs.msg.dds.JointOfflineMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      cdr.read_type_e(data.getJointOfflineHashCodes());	
      cdr.read_type_e(data.getNominalLeftFootContactPoints2d());	
      cdr.read_type_e(data.getNominalRightFootContactPoints2d());	
      data.setExecutionDelayTime(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.JointOfflineMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_e("joint_offline_hash_codes", data.getJointOfflineHashCodes());
      ser.write_type_e("nominal_left_foot_contact_points_2d", data.getNominalLeftFootContactPoints2d());
      ser.write_type_e("nominal_right_foot_contact_points_2d", data.getNominalRightFootContactPoints2d());
      ser.write_type_6("execution_delay_time", data.getExecutionDelayTime());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.JointOfflineMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_e("joint_offline_hash_codes", data.getJointOfflineHashCodes());
      ser.read_type_e("nominal_left_foot_contact_points_2d", data.getNominalLeftFootContactPoints2d());
      ser.read_type_e("nominal_right_foot_contact_points_2d", data.getNominalRightFootContactPoints2d());
      data.setExecutionDelayTime(ser.read_type_6("execution_delay_time"));
   }

   public static void staticCopy(controller_msgs.msg.dds.JointOfflineMessage src, controller_msgs.msg.dds.JointOfflineMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.JointOfflineMessage createData()
   {
      return new controller_msgs.msg.dds.JointOfflineMessage();
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
   
   public void serialize(controller_msgs.msg.dds.JointOfflineMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.JointOfflineMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.JointOfflineMessage src, controller_msgs.msg.dds.JointOfflineMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public JointOfflineMessagePubSubType newInstance()
   {
      return new JointOfflineMessagePubSubType();
   }
}
