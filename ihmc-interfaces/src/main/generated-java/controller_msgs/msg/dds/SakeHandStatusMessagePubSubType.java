package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "SakeHandStatusMessage" defined in "SakeHandStatusMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from SakeHandStatusMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit SakeHandStatusMessage_.idl instead.
*
*/
public class SakeHandStatusMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.SakeHandStatusMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::SakeHandStatusMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "714eb9d6a51f107d47d1ec844485b03a72735d0333d7609e928a41736f2aacce";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.SakeHandStatusMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.SakeHandStatusMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.SakeHandStatusMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.SakeHandStatusMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getErrorMessage().length() + 1;

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.SakeHandStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getRobotSide());

      cdr.write_type_6(data.getNormalizedDesiredPosition());

      cdr.write_type_6(data.getNormalizedDesiredTorque());

      cdr.write_type_6(data.getNormalizedTemperature());

      cdr.write_type_6(data.getNormalizedMeasuredPosition());

      cdr.write_type_6(data.getNormalizedMeasuredTorque());

      cdr.write_type_6(data.getNormalizedMeasuredVelocity());

      if(data.getErrorMessage().length() <= 255)
      cdr.write_type_d(data.getErrorMessage());else
          throw new RuntimeException("error_message field exceeds the maximum length");

      cdr.write_type_3(data.getRealtimeTick());

   }

   public static void read(controller_msgs.msg.dds.SakeHandStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setRobotSide(cdr.read_type_9());
      	
      data.setNormalizedDesiredPosition(cdr.read_type_6());
      	
      data.setNormalizedDesiredTorque(cdr.read_type_6());
      	
      data.setNormalizedTemperature(cdr.read_type_6());
      	
      data.setNormalizedMeasuredPosition(cdr.read_type_6());
      	
      data.setNormalizedMeasuredTorque(cdr.read_type_6());
      	
      data.setNormalizedMeasuredVelocity(cdr.read_type_6());
      	
      cdr.read_type_d(data.getErrorMessage());	
      data.setRealtimeTick(cdr.read_type_3());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.SakeHandStatusMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_6("normalized_desired_position", data.getNormalizedDesiredPosition());
      ser.write_type_6("normalized_desired_torque", data.getNormalizedDesiredTorque());
      ser.write_type_6("normalized_temperature", data.getNormalizedTemperature());
      ser.write_type_6("normalized_measured_position", data.getNormalizedMeasuredPosition());
      ser.write_type_6("normalized_measured_torque", data.getNormalizedMeasuredTorque());
      ser.write_type_6("normalized_measured_velocity", data.getNormalizedMeasuredVelocity());
      ser.write_type_d("error_message", data.getErrorMessage());
      ser.write_type_3("realtime_tick", data.getRealtimeTick());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.SakeHandStatusMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setRobotSide(ser.read_type_9("robot_side"));
      data.setNormalizedDesiredPosition(ser.read_type_6("normalized_desired_position"));
      data.setNormalizedDesiredTorque(ser.read_type_6("normalized_desired_torque"));
      data.setNormalizedTemperature(ser.read_type_6("normalized_temperature"));
      data.setNormalizedMeasuredPosition(ser.read_type_6("normalized_measured_position"));
      data.setNormalizedMeasuredTorque(ser.read_type_6("normalized_measured_torque"));
      data.setNormalizedMeasuredVelocity(ser.read_type_6("normalized_measured_velocity"));
      ser.read_type_d("error_message", data.getErrorMessage());
      data.setRealtimeTick(ser.read_type_3("realtime_tick"));
   }

   public static void staticCopy(controller_msgs.msg.dds.SakeHandStatusMessage src, controller_msgs.msg.dds.SakeHandStatusMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.SakeHandStatusMessage createData()
   {
      return new controller_msgs.msg.dds.SakeHandStatusMessage();
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
   
   public void serialize(controller_msgs.msg.dds.SakeHandStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.SakeHandStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.SakeHandStatusMessage src, controller_msgs.msg.dds.SakeHandStatusMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public SakeHandStatusMessagePubSubType newInstance()
   {
      return new SakeHandStatusMessagePubSubType();
   }
}
