package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FootLoadBearingMessage" defined in "FootLoadBearingMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FootLoadBearingMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FootLoadBearingMessage_.idl instead.
*
*/
public class FootLoadBearingMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.FootLoadBearingMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::FootLoadBearingMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "4bf668c63c8c31efba2f62875c8acb0321fffca7d21d1d827f23e2971b77e659";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.FootLoadBearingMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.FootLoadBearingMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FootLoadBearingMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FootLoadBearingMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.FootLoadBearingMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getRobotSide());

      cdr.write_type_9(data.getLoadBearingRequest());

      cdr.write_type_6(data.getExecutionDelayTime());

   }

   public static void read(controller_msgs.msg.dds.FootLoadBearingMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setRobotSide(cdr.read_type_9());
      	
      data.setLoadBearingRequest(cdr.read_type_9());
      	
      data.setExecutionDelayTime(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.FootLoadBearingMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_9("load_bearing_request", data.getLoadBearingRequest());
      ser.write_type_6("execution_delay_time", data.getExecutionDelayTime());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.FootLoadBearingMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setRobotSide(ser.read_type_9("robot_side"));
      data.setLoadBearingRequest(ser.read_type_9("load_bearing_request"));
      data.setExecutionDelayTime(ser.read_type_6("execution_delay_time"));
   }

   public static void staticCopy(controller_msgs.msg.dds.FootLoadBearingMessage src, controller_msgs.msg.dds.FootLoadBearingMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.FootLoadBearingMessage createData()
   {
      return new controller_msgs.msg.dds.FootLoadBearingMessage();
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
   
   public void serialize(controller_msgs.msg.dds.FootLoadBearingMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.FootLoadBearingMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.FootLoadBearingMessage src, controller_msgs.msg.dds.FootLoadBearingMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FootLoadBearingMessagePubSubType newInstance()
   {
      return new FootLoadBearingMessagePubSubType();
   }
}
