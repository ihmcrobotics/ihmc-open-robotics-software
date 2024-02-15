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
   		return "ed25e34995e7c3d45b5f271ab90d387ef27134d803c3d9bfcbc31ca616ef8194";
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.SakeHandStatusMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.SakeHandStatusMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.SakeHandStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_9(data.getRobotSide());

      cdr.write_type_6(data.getTemperature());

      cdr.write_type_6(data.getNormalizedCurrentPosition());

      cdr.write_type_6(data.getNormalizedCurrentTorque());

      cdr.write_type_7(data.getIsCalibrated());

      cdr.write_type_7(data.getNeedsReset());

   }

   public static void read(controller_msgs.msg.dds.SakeHandStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setRobotSide(cdr.read_type_9());
      	
      data.setTemperature(cdr.read_type_6());
      	
      data.setNormalizedCurrentPosition(cdr.read_type_6());
      	
      data.setNormalizedCurrentTorque(cdr.read_type_6());
      	
      data.setIsCalibrated(cdr.read_type_7());
      	
      data.setNeedsReset(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.SakeHandStatusMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_6("temperature", data.getTemperature());
      ser.write_type_6("normalized_current_position", data.getNormalizedCurrentPosition());
      ser.write_type_6("normalized_current_torque", data.getNormalizedCurrentTorque());
      ser.write_type_7("is_calibrated", data.getIsCalibrated());
      ser.write_type_7("needs_reset", data.getNeedsReset());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.SakeHandStatusMessage data)
   {
      data.setRobotSide(ser.read_type_9("robot_side"));
      data.setTemperature(ser.read_type_6("temperature"));
      data.setNormalizedCurrentPosition(ser.read_type_6("normalized_current_position"));
      data.setNormalizedCurrentTorque(ser.read_type_6("normalized_current_torque"));
      data.setIsCalibrated(ser.read_type_7("is_calibrated"));
      data.setNeedsReset(ser.read_type_7("needs_reset"));
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
