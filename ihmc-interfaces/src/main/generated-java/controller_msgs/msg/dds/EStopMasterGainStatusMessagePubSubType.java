package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "EStopMasterGainStatusMessage" defined in "EStopMasterGainStatusMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from EStopMasterGainStatusMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit EStopMasterGainStatusMessage_.idl instead.
*
*/
public class EStopMasterGainStatusMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.EStopMasterGainStatusMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::EStopMasterGainStatusMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "625441146c889e4268a47efc7ceac534dba72fa10667c000133b21a0547c45a3";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.EStopMasterGainStatusMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.EStopMasterGainStatusMessage data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.EStopMasterGainStatusMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.EStopMasterGainStatusMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.EStopMasterGainStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_7(data.getIsEstopped());

      cdr.write_type_7(data.getRobotIsFaulted());

      cdr.write_type_7(data.getRobotIsServod());

      cdr.write_type_7(data.getRobotIsCalibrated());

      cdr.write_type_7(data.getPublishingToRobotIsEnabled());

      cdr.write_type_7(data.getActuatorsAreEnabled());

      cdr.write_type_6(data.getCurrentMasterGain());

   }

   public static void read(controller_msgs.msg.dds.EStopMasterGainStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setIsEstopped(cdr.read_type_7());
      	
      data.setRobotIsFaulted(cdr.read_type_7());
      	
      data.setRobotIsServod(cdr.read_type_7());
      	
      data.setRobotIsCalibrated(cdr.read_type_7());
      	
      data.setPublishingToRobotIsEnabled(cdr.read_type_7());
      	
      data.setActuatorsAreEnabled(cdr.read_type_7());
      	
      data.setCurrentMasterGain(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.EStopMasterGainStatusMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_7("is_estopped", data.getIsEstopped());
      ser.write_type_7("robot_is_faulted", data.getRobotIsFaulted());
      ser.write_type_7("robot_is_servod", data.getRobotIsServod());
      ser.write_type_7("robot_is_calibrated", data.getRobotIsCalibrated());
      ser.write_type_7("publishing_to_robot_is_enabled", data.getPublishingToRobotIsEnabled());
      ser.write_type_7("actuators_are_enabled", data.getActuatorsAreEnabled());
      ser.write_type_6("current_master_gain", data.getCurrentMasterGain());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.EStopMasterGainStatusMessage data)
   {
      data.setIsEstopped(ser.read_type_7("is_estopped"));
      data.setRobotIsFaulted(ser.read_type_7("robot_is_faulted"));
      data.setRobotIsServod(ser.read_type_7("robot_is_servod"));
      data.setRobotIsCalibrated(ser.read_type_7("robot_is_calibrated"));
      data.setPublishingToRobotIsEnabled(ser.read_type_7("publishing_to_robot_is_enabled"));
      data.setActuatorsAreEnabled(ser.read_type_7("actuators_are_enabled"));
      data.setCurrentMasterGain(ser.read_type_6("current_master_gain"));
   }

   public static void staticCopy(controller_msgs.msg.dds.EStopMasterGainStatusMessage src, controller_msgs.msg.dds.EStopMasterGainStatusMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.EStopMasterGainStatusMessage createData()
   {
      return new controller_msgs.msg.dds.EStopMasterGainStatusMessage();
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
   
   public void serialize(controller_msgs.msg.dds.EStopMasterGainStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.EStopMasterGainStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.EStopMasterGainStatusMessage src, controller_msgs.msg.dds.EStopMasterGainStatusMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public EStopMasterGainStatusMessagePubSubType newInstance()
   {
      return new EStopMasterGainStatusMessagePubSubType();
   }
}
