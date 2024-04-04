package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "SakeHandDesiredCommandMessage" defined in "SakeHandDesiredCommandMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from SakeHandDesiredCommandMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit SakeHandDesiredCommandMessage_.idl instead.
*
*/
public class SakeHandDesiredCommandMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.SakeHandDesiredCommandMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::SakeHandDesiredCommandMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "20f5352da5886a3c39d884c2bb1fdcb6dcd5b2f3f7ae23d9dc354caae6fac7de";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.SakeHandDesiredCommandMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.SakeHandDesiredCommandMessage data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.SakeHandDesiredCommandMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.SakeHandDesiredCommandMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.SakeHandDesiredCommandMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_9(data.getRobotSide());

      cdr.write_type_7(data.getRequestCalibration());

      cdr.write_type_7(data.getRequestResetErrors());

      cdr.write_type_6(data.getNormalizedGripperDesiredPosition());

      cdr.write_type_6(data.getNormalizedGripperTorqueLimit());

      cdr.write_type_7(data.getTorqueOn());

   }

   public static void read(controller_msgs.msg.dds.SakeHandDesiredCommandMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setRobotSide(cdr.read_type_9());
      	
      data.setRequestCalibration(cdr.read_type_7());
      	
      data.setRequestResetErrors(cdr.read_type_7());
      	
      data.setNormalizedGripperDesiredPosition(cdr.read_type_6());
      	
      data.setNormalizedGripperTorqueLimit(cdr.read_type_6());
      	
      data.setTorqueOn(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.SakeHandDesiredCommandMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_7("request_calibration", data.getRequestCalibration());
      ser.write_type_7("request_reset_errors", data.getRequestResetErrors());
      ser.write_type_6("normalized_gripper_desired_position", data.getNormalizedGripperDesiredPosition());
      ser.write_type_6("normalized_gripper_torque_limit", data.getNormalizedGripperTorqueLimit());
      ser.write_type_7("torque_on", data.getTorqueOn());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.SakeHandDesiredCommandMessage data)
   {
      data.setRobotSide(ser.read_type_9("robot_side"));
      data.setRequestCalibration(ser.read_type_7("request_calibration"));
      data.setRequestResetErrors(ser.read_type_7("request_reset_errors"));
      data.setNormalizedGripperDesiredPosition(ser.read_type_6("normalized_gripper_desired_position"));
      data.setNormalizedGripperTorqueLimit(ser.read_type_6("normalized_gripper_torque_limit"));
      data.setTorqueOn(ser.read_type_7("torque_on"));
   }

   public static void staticCopy(controller_msgs.msg.dds.SakeHandDesiredCommandMessage src, controller_msgs.msg.dds.SakeHandDesiredCommandMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.SakeHandDesiredCommandMessage createData()
   {
      return new controller_msgs.msg.dds.SakeHandDesiredCommandMessage();
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
   
   public void serialize(controller_msgs.msg.dds.SakeHandDesiredCommandMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.SakeHandDesiredCommandMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.SakeHandDesiredCommandMessage src, controller_msgs.msg.dds.SakeHandDesiredCommandMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public SakeHandDesiredCommandMessagePubSubType newInstance()
   {
      return new SakeHandDesiredCommandMessagePubSubType();
   }
}
