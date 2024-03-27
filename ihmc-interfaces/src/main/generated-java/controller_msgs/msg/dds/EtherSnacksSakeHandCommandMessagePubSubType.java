package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "EtherSnacksSakeHandCommandMessage" defined in "EtherSnacksSakeHandCommandMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from EtherSnacksSakeHandCommandMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit EtherSnacksSakeHandCommandMessage_.idl instead.
*
*/
public class EtherSnacksSakeHandCommandMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.EtherSnacksSakeHandCommandMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::EtherSnacksSakeHandCommandMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "ab6c143aea9bf5d14678999acf54a880a05a7944aadb98a16c772d1be17021fb";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.EtherSnacksSakeHandCommandMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.EtherSnacksSakeHandCommandMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.EtherSnacksSakeHandCommandMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.EtherSnacksSakeHandCommandMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.EtherSnacksSakeHandCommandMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_9(data.getRobotSide());

      cdr.write_type_6(data.getDesiredPosition());

      cdr.write_type_6(data.getTorqueLimit());

      cdr.write_type_7(data.getTorqueOn());

      cdr.write_type_7(data.getCalibrate());

      cdr.write_type_7(data.getReset());

   }

   public static void read(controller_msgs.msg.dds.EtherSnacksSakeHandCommandMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setRobotSide(cdr.read_type_9());
      	
      data.setDesiredPosition(cdr.read_type_6());
      	
      data.setTorqueLimit(cdr.read_type_6());
      	
      data.setTorqueOn(cdr.read_type_7());
      	
      data.setCalibrate(cdr.read_type_7());
      	
      data.setReset(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.EtherSnacksSakeHandCommandMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_6("desired_position", data.getDesiredPosition());
      ser.write_type_6("torque_limit", data.getTorqueLimit());
      ser.write_type_7("torque_on", data.getTorqueOn());
      ser.write_type_7("calibrate", data.getCalibrate());
      ser.write_type_7("reset", data.getReset());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.EtherSnacksSakeHandCommandMessage data)
   {
      data.setRobotSide(ser.read_type_9("robot_side"));
      data.setDesiredPosition(ser.read_type_6("desired_position"));
      data.setTorqueLimit(ser.read_type_6("torque_limit"));
      data.setTorqueOn(ser.read_type_7("torque_on"));
      data.setCalibrate(ser.read_type_7("calibrate"));
      data.setReset(ser.read_type_7("reset"));
   }

   public static void staticCopy(controller_msgs.msg.dds.EtherSnacksSakeHandCommandMessage src, controller_msgs.msg.dds.EtherSnacksSakeHandCommandMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.EtherSnacksSakeHandCommandMessage createData()
   {
      return new controller_msgs.msg.dds.EtherSnacksSakeHandCommandMessage();
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
   
   public void serialize(controller_msgs.msg.dds.EtherSnacksSakeHandCommandMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.EtherSnacksSakeHandCommandMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.EtherSnacksSakeHandCommandMessage src, controller_msgs.msg.dds.EtherSnacksSakeHandCommandMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public EtherSnacksSakeHandCommandMessagePubSubType newInstance()
   {
      return new EtherSnacksSakeHandCommandMessagePubSubType();
   }
}
