package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "EtherSnacksSakeHandStatusMessage" defined in "EtherSnacksSakeHandStatusMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from EtherSnacksSakeHandStatusMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit EtherSnacksSakeHandStatusMessage_.idl instead.
*
*/
public class EtherSnacksSakeHandStatusMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.EtherSnacksSakeHandStatusMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::EtherSnacksSakeHandStatusMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "9011b84ea735afffd8ce67c2bc807203d4cff16fd1383a0888de31442120614a";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.EtherSnacksSakeHandStatusMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.EtherSnacksSakeHandStatusMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.EtherSnacksSakeHandStatusMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.EtherSnacksSakeHandStatusMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.EtherSnacksSakeHandStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_9(data.getRobotSide());

      cdr.write_type_6(data.getDesiredPosition());

      cdr.write_type_7(data.getTorqueOn());

      cdr.write_type_6(data.getTorqueLimit());

      cdr.write_type_6(data.getMeasuredPosition());

      cdr.write_type_6(data.getMeasuredTorque());

      cdr.write_type_2(data.getMeasuredTemperature());

      cdr.write_type_6(data.getMeasuredVelocity());

      cdr.write_type_2(data.getErrorCodes());

      cdr.write_type_2(data.getRealtimeTick());

      cdr.write_type_7(data.getIsCalibrated());

      cdr.write_type_7(data.getNeedsReset());

   }

   public static void read(controller_msgs.msg.dds.EtherSnacksSakeHandStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setRobotSide(cdr.read_type_9());
      	
      data.setDesiredPosition(cdr.read_type_6());
      	
      data.setTorqueOn(cdr.read_type_7());
      	
      data.setTorqueLimit(cdr.read_type_6());
      	
      data.setMeasuredPosition(cdr.read_type_6());
      	
      data.setMeasuredTorque(cdr.read_type_6());
      	
      data.setMeasuredTemperature(cdr.read_type_2());
      	
      data.setMeasuredVelocity(cdr.read_type_6());
      	
      data.setErrorCodes(cdr.read_type_2());
      	
      data.setRealtimeTick(cdr.read_type_2());
      	
      data.setIsCalibrated(cdr.read_type_7());
      	
      data.setNeedsReset(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.EtherSnacksSakeHandStatusMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_6("desired_position", data.getDesiredPosition());
      ser.write_type_7("torque_on", data.getTorqueOn());
      ser.write_type_6("torque_limit", data.getTorqueLimit());
      ser.write_type_6("measured_position", data.getMeasuredPosition());
      ser.write_type_6("measured_torque", data.getMeasuredTorque());
      ser.write_type_2("measured_temperature", data.getMeasuredTemperature());
      ser.write_type_6("measured_velocity", data.getMeasuredVelocity());
      ser.write_type_2("error_codes", data.getErrorCodes());
      ser.write_type_2("realtime_tick", data.getRealtimeTick());
      ser.write_type_7("is_calibrated", data.getIsCalibrated());
      ser.write_type_7("needs_reset", data.getNeedsReset());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.EtherSnacksSakeHandStatusMessage data)
   {
      data.setRobotSide(ser.read_type_9("robot_side"));
      data.setDesiredPosition(ser.read_type_6("desired_position"));
      data.setTorqueOn(ser.read_type_7("torque_on"));
      data.setTorqueLimit(ser.read_type_6("torque_limit"));
      data.setMeasuredPosition(ser.read_type_6("measured_position"));
      data.setMeasuredTorque(ser.read_type_6("measured_torque"));
      data.setMeasuredTemperature(ser.read_type_2("measured_temperature"));
      data.setMeasuredVelocity(ser.read_type_6("measured_velocity"));
      data.setErrorCodes(ser.read_type_2("error_codes"));
      data.setRealtimeTick(ser.read_type_2("realtime_tick"));
      data.setIsCalibrated(ser.read_type_7("is_calibrated"));
      data.setNeedsReset(ser.read_type_7("needs_reset"));
   }

   public static void staticCopy(controller_msgs.msg.dds.EtherSnacksSakeHandStatusMessage src, controller_msgs.msg.dds.EtherSnacksSakeHandStatusMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.EtherSnacksSakeHandStatusMessage createData()
   {
      return new controller_msgs.msg.dds.EtherSnacksSakeHandStatusMessage();
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
   
   public void serialize(controller_msgs.msg.dds.EtherSnacksSakeHandStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.EtherSnacksSakeHandStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.EtherSnacksSakeHandStatusMessage src, controller_msgs.msg.dds.EtherSnacksSakeHandStatusMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public EtherSnacksSakeHandStatusMessagePubSubType newInstance()
   {
      return new EtherSnacksSakeHandStatusMessagePubSubType();
   }
}
