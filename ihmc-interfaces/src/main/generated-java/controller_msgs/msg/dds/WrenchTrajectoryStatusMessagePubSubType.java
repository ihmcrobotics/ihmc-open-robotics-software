package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "WrenchTrajectoryStatusMessage" defined in "WrenchTrajectoryStatusMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from WrenchTrajectoryStatusMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit WrenchTrajectoryStatusMessage_.idl instead.
*
*/
public class WrenchTrajectoryStatusMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.WrenchTrajectoryStatusMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::WrenchTrajectoryStatusMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "17a0d056f073bd8b6ca69a6cd098f7f4bb41e11c8bacf254f74e54d46ae5f14e";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.WrenchTrajectoryStatusMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.WrenchTrajectoryStatusMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.WrenchTrajectoryStatusMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.WrenchTrajectoryStatusMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getEndEffectorName().length() + 1;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.WrenchTrajectoryStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      if(data.getEndEffectorName().length() <= 255)
      cdr.write_type_d(data.getEndEffectorName());else
          throw new RuntimeException("end_effector_name field exceeds the maximum length");

      cdr.write_type_9(data.getTrajectoryExecutionStatus());

      cdr.write_type_6(data.getTimestamp());

   }

   public static void read(controller_msgs.msg.dds.WrenchTrajectoryStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      cdr.read_type_d(data.getEndEffectorName());	
      data.setTrajectoryExecutionStatus(cdr.read_type_9());
      	
      data.setTimestamp(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.WrenchTrajectoryStatusMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_d("end_effector_name", data.getEndEffectorName());
      ser.write_type_9("trajectory_execution_status", data.getTrajectoryExecutionStatus());
      ser.write_type_6("timestamp", data.getTimestamp());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.WrenchTrajectoryStatusMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_d("end_effector_name", data.getEndEffectorName());
      data.setTrajectoryExecutionStatus(ser.read_type_9("trajectory_execution_status"));
      data.setTimestamp(ser.read_type_6("timestamp"));
   }

   public static void staticCopy(controller_msgs.msg.dds.WrenchTrajectoryStatusMessage src, controller_msgs.msg.dds.WrenchTrajectoryStatusMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.WrenchTrajectoryStatusMessage createData()
   {
      return new controller_msgs.msg.dds.WrenchTrajectoryStatusMessage();
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
   
   public void serialize(controller_msgs.msg.dds.WrenchTrajectoryStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.WrenchTrajectoryStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.WrenchTrajectoryStatusMessage src, controller_msgs.msg.dds.WrenchTrajectoryStatusMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public WrenchTrajectoryStatusMessagePubSubType newInstance()
   {
      return new WrenchTrajectoryStatusMessagePubSubType();
   }
}
