package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "KinematicsStreamingToolboxOutputConfigurationMessage" defined in "KinematicsStreamingToolboxOutputConfigurationMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from KinematicsStreamingToolboxOutputConfigurationMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit KinematicsStreamingToolboxOutputConfigurationMessage_.idl instead.
*
*/
public class KinematicsStreamingToolboxOutputConfigurationMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.KinematicsStreamingToolboxOutputConfigurationMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::KinematicsStreamingToolboxOutputConfigurationMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.KinematicsStreamingToolboxOutputConfigurationMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.KinematicsStreamingToolboxOutputConfigurationMessage data) throws java.io.IOException
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


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.KinematicsStreamingToolboxOutputConfigurationMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.KinematicsStreamingToolboxOutputConfigurationMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.KinematicsStreamingToolboxOutputConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {

      cdr.write_type_4(data.getSequenceId());


      cdr.write_type_7(data.getEnableLeftArmJointspace());


      cdr.write_type_7(data.getEnableRightArmJointspace());


      cdr.write_type_7(data.getEnableNeckJointspace());


      cdr.write_type_7(data.getEnableLeftHandTaskspace());


      cdr.write_type_7(data.getEnableRightHandTaskspace());


      cdr.write_type_7(data.getEnableChestTaskspace());


      cdr.write_type_7(data.getEnablePelvisTaskspace());

   }

   public static void read(controller_msgs.msg.dds.KinematicsStreamingToolboxOutputConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {

      data.setSequenceId(cdr.read_type_4());
      	

      data.setEnableLeftArmJointspace(cdr.read_type_7());
      	

      data.setEnableRightArmJointspace(cdr.read_type_7());
      	

      data.setEnableNeckJointspace(cdr.read_type_7());
      	

      data.setEnableLeftHandTaskspace(cdr.read_type_7());
      	

      data.setEnableRightHandTaskspace(cdr.read_type_7());
      	

      data.setEnableChestTaskspace(cdr.read_type_7());
      	

      data.setEnablePelvisTaskspace(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.KinematicsStreamingToolboxOutputConfigurationMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {

      ser.write_type_4("sequence_id", data.getSequenceId());

      ser.write_type_7("enable_left_arm_jointspace", data.getEnableLeftArmJointspace());

      ser.write_type_7("enable_right_arm_jointspace", data.getEnableRightArmJointspace());

      ser.write_type_7("enable_neck_jointspace", data.getEnableNeckJointspace());

      ser.write_type_7("enable_left_hand_taskspace", data.getEnableLeftHandTaskspace());

      ser.write_type_7("enable_right_hand_taskspace", data.getEnableRightHandTaskspace());

      ser.write_type_7("enable_chest_taskspace", data.getEnableChestTaskspace());

      ser.write_type_7("enable_pelvis_taskspace", data.getEnablePelvisTaskspace());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.KinematicsStreamingToolboxOutputConfigurationMessage data)
   {

      data.setSequenceId(ser.read_type_4("sequence_id"));

      data.setEnableLeftArmJointspace(ser.read_type_7("enable_left_arm_jointspace"));

      data.setEnableRightArmJointspace(ser.read_type_7("enable_right_arm_jointspace"));

      data.setEnableNeckJointspace(ser.read_type_7("enable_neck_jointspace"));

      data.setEnableLeftHandTaskspace(ser.read_type_7("enable_left_hand_taskspace"));

      data.setEnableRightHandTaskspace(ser.read_type_7("enable_right_hand_taskspace"));

      data.setEnableChestTaskspace(ser.read_type_7("enable_chest_taskspace"));

      data.setEnablePelvisTaskspace(ser.read_type_7("enable_pelvis_taskspace"));
   }

   public static void staticCopy(controller_msgs.msg.dds.KinematicsStreamingToolboxOutputConfigurationMessage src, controller_msgs.msg.dds.KinematicsStreamingToolboxOutputConfigurationMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.KinematicsStreamingToolboxOutputConfigurationMessage createData()
   {
      return new controller_msgs.msg.dds.KinematicsStreamingToolboxOutputConfigurationMessage();
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
   
   public void serialize(controller_msgs.msg.dds.KinematicsStreamingToolboxOutputConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.KinematicsStreamingToolboxOutputConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.KinematicsStreamingToolboxOutputConfigurationMessage src, controller_msgs.msg.dds.KinematicsStreamingToolboxOutputConfigurationMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public KinematicsStreamingToolboxOutputConfigurationMessagePubSubType newInstance()
   {
      return new KinematicsStreamingToolboxOutputConfigurationMessagePubSubType();
   }
}
