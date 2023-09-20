package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "HandSakeStatusMessage" defined in "HandSakeStatusMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from HandSakeStatusMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit HandSakeStatusMessage_.idl instead.
*
*/
public class HandSakeStatusMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.HandSakeStatusMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::HandSakeStatusMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "62826406821801fd66674114f404d55e395d103f78e9f2c67d89fa25bd7f94f0";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.HandSakeStatusMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.HandSakeStatusMessage data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.HandSakeStatusMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.HandSakeStatusMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.HandSakeStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getRobotSide());

      cdr.write_type_6(data.getTemperature());

      cdr.write_type_6(data.getPresentTorqueRatio());

      cdr.write_type_6(data.getPostionRatio());

      cdr.write_type_6(data.getGoalTorqueRatio());

      cdr.write_type_7(data.getCalibrated());

      cdr.write_type_7(data.getNeedsReset());

   }

   public static void read(controller_msgs.msg.dds.HandSakeStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setRobotSide(cdr.read_type_9());
      	
      data.setTemperature(cdr.read_type_6());
      	
      data.setPresentTorqueRatio(cdr.read_type_6());
      	
      data.setPostionRatio(cdr.read_type_6());
      	
      data.setGoalTorqueRatio(cdr.read_type_6());
      	
      data.setCalibrated(cdr.read_type_7());
      	
      data.setNeedsReset(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.HandSakeStatusMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_6("temperature", data.getTemperature());
      ser.write_type_6("present_torque_ratio", data.getPresentTorqueRatio());
      ser.write_type_6("postion_ratio", data.getPostionRatio());
      ser.write_type_6("goal_torque_ratio", data.getGoalTorqueRatio());
      ser.write_type_7("calibrated", data.getCalibrated());
      ser.write_type_7("needs_reset", data.getNeedsReset());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.HandSakeStatusMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setRobotSide(ser.read_type_9("robot_side"));
      data.setTemperature(ser.read_type_6("temperature"));
      data.setPresentTorqueRatio(ser.read_type_6("present_torque_ratio"));
      data.setPostionRatio(ser.read_type_6("postion_ratio"));
      data.setGoalTorqueRatio(ser.read_type_6("goal_torque_ratio"));
      data.setCalibrated(ser.read_type_7("calibrated"));
      data.setNeedsReset(ser.read_type_7("needs_reset"));
   }

   public static void staticCopy(controller_msgs.msg.dds.HandSakeStatusMessage src, controller_msgs.msg.dds.HandSakeStatusMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.HandSakeStatusMessage createData()
   {
      return new controller_msgs.msg.dds.HandSakeStatusMessage();
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
   
   public void serialize(controller_msgs.msg.dds.HandSakeStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.HandSakeStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.HandSakeStatusMessage src, controller_msgs.msg.dds.HandSakeStatusMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public HandSakeStatusMessagePubSubType newInstance()
   {
      return new HandSakeStatusMessagePubSubType();
   }
}
