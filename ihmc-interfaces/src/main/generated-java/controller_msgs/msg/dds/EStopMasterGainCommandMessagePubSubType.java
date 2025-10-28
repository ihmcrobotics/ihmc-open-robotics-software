package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "EStopMasterGainCommandMessage" defined in "EStopMasterGainCommandMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from EStopMasterGainCommandMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit EStopMasterGainCommandMessage_.idl instead.
*
*/
public class EStopMasterGainCommandMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.EStopMasterGainCommandMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::EStopMasterGainCommandMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "fd8b21b2edd236245629837baa09bf5372ef641ace2e0b4387619263bc622611";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.EStopMasterGainCommandMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.EStopMasterGainCommandMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.EStopMasterGainCommandMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.EStopMasterGainCommandMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.EStopMasterGainCommandMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_7(data.getEstop());

      cdr.write_type_7(data.getExecuteEstop());

      cdr.write_type_6(data.getDesiredMasterGain());

      cdr.write_type_7(data.getExecuteMasterGain());

      cdr.write_type_7(data.getServoRobot());

      cdr.write_type_7(data.getExecuteServoRobot());

      cdr.write_type_7(data.getUnservoQuickly());

      cdr.write_type_7(data.getExecuteUnservoQuickly());

      cdr.write_type_7(data.getEnablePublishingToRobot());

      cdr.write_type_7(data.getExecuteEnablePublishingToRobot());

      cdr.write_type_7(data.getClearFaults());

      cdr.write_type_7(data.getExecuteClearFaults());

      cdr.write_type_7(data.getCalibrateRobot());

      cdr.write_type_7(data.getExecuteCalibrateRobot());

      cdr.write_type_7(data.getEnableActuators());

      cdr.write_type_7(data.getExecuteEnableActuators());

   }

   public static void read(controller_msgs.msg.dds.EStopMasterGainCommandMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setEstop(cdr.read_type_7());
      	
      data.setExecuteEstop(cdr.read_type_7());
      	
      data.setDesiredMasterGain(cdr.read_type_6());
      	
      data.setExecuteMasterGain(cdr.read_type_7());
      	
      data.setServoRobot(cdr.read_type_7());
      	
      data.setExecuteServoRobot(cdr.read_type_7());
      	
      data.setUnservoQuickly(cdr.read_type_7());
      	
      data.setExecuteUnservoQuickly(cdr.read_type_7());
      	
      data.setEnablePublishingToRobot(cdr.read_type_7());
      	
      data.setExecuteEnablePublishingToRobot(cdr.read_type_7());
      	
      data.setClearFaults(cdr.read_type_7());
      	
      data.setExecuteClearFaults(cdr.read_type_7());
      	
      data.setCalibrateRobot(cdr.read_type_7());
      	
      data.setExecuteCalibrateRobot(cdr.read_type_7());
      	
      data.setEnableActuators(cdr.read_type_7());
      	
      data.setExecuteEnableActuators(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.EStopMasterGainCommandMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_7("estop", data.getEstop());
      ser.write_type_7("execute_estop", data.getExecuteEstop());
      ser.write_type_6("desired_master_gain", data.getDesiredMasterGain());
      ser.write_type_7("execute_master_gain", data.getExecuteMasterGain());
      ser.write_type_7("servo_robot", data.getServoRobot());
      ser.write_type_7("execute_servo_robot", data.getExecuteServoRobot());
      ser.write_type_7("unservo_quickly", data.getUnservoQuickly());
      ser.write_type_7("execute_unservo_quickly", data.getExecuteUnservoQuickly());
      ser.write_type_7("enable_publishing_to_robot", data.getEnablePublishingToRobot());
      ser.write_type_7("execute_enable_publishing_to_robot", data.getExecuteEnablePublishingToRobot());
      ser.write_type_7("clear_faults", data.getClearFaults());
      ser.write_type_7("execute_clear_faults", data.getExecuteClearFaults());
      ser.write_type_7("calibrate_robot", data.getCalibrateRobot());
      ser.write_type_7("execute_calibrate_robot", data.getExecuteCalibrateRobot());
      ser.write_type_7("enable_actuators", data.getEnableActuators());
      ser.write_type_7("execute_enable_actuators", data.getExecuteEnableActuators());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.EStopMasterGainCommandMessage data)
   {
      data.setEstop(ser.read_type_7("estop"));
      data.setExecuteEstop(ser.read_type_7("execute_estop"));
      data.setDesiredMasterGain(ser.read_type_6("desired_master_gain"));
      data.setExecuteMasterGain(ser.read_type_7("execute_master_gain"));
      data.setServoRobot(ser.read_type_7("servo_robot"));
      data.setExecuteServoRobot(ser.read_type_7("execute_servo_robot"));
      data.setUnservoQuickly(ser.read_type_7("unservo_quickly"));
      data.setExecuteUnservoQuickly(ser.read_type_7("execute_unservo_quickly"));
      data.setEnablePublishingToRobot(ser.read_type_7("enable_publishing_to_robot"));
      data.setExecuteEnablePublishingToRobot(ser.read_type_7("execute_enable_publishing_to_robot"));
      data.setClearFaults(ser.read_type_7("clear_faults"));
      data.setExecuteClearFaults(ser.read_type_7("execute_clear_faults"));
      data.setCalibrateRobot(ser.read_type_7("calibrate_robot"));
      data.setExecuteCalibrateRobot(ser.read_type_7("execute_calibrate_robot"));
      data.setEnableActuators(ser.read_type_7("enable_actuators"));
      data.setExecuteEnableActuators(ser.read_type_7("execute_enable_actuators"));
   }

   public static void staticCopy(controller_msgs.msg.dds.EStopMasterGainCommandMessage src, controller_msgs.msg.dds.EStopMasterGainCommandMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.EStopMasterGainCommandMessage createData()
   {
      return new controller_msgs.msg.dds.EStopMasterGainCommandMessage();
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
   
   public void serialize(controller_msgs.msg.dds.EStopMasterGainCommandMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.EStopMasterGainCommandMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.EStopMasterGainCommandMessage src, controller_msgs.msg.dds.EStopMasterGainCommandMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public EStopMasterGainCommandMessagePubSubType newInstance()
   {
      return new EStopMasterGainCommandMessagePubSubType();
   }
}
