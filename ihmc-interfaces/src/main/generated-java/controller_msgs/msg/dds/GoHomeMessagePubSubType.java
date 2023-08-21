package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "GoHomeMessage" defined in "GoHomeMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from GoHomeMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit GoHomeMessage_.idl instead.
*
*/
public class GoHomeMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.GoHomeMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::GoHomeMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "4e3c534c7eb1358bfba06a6ba28190ac4d1393267c1ea44bb8b92707e8b9774c";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.GoHomeMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.GoHomeMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.GoHomeMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.GoHomeMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.GoHomeMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getHumanoidBodyPart());

      cdr.write_type_9(data.getRobotSide());

      cdr.write_type_6(data.getTrajectoryTime());

      cdr.write_type_6(data.getExecutionDelayTime());

   }

   public static void read(controller_msgs.msg.dds.GoHomeMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setHumanoidBodyPart(cdr.read_type_9());
      	
      data.setRobotSide(cdr.read_type_9());
      	
      data.setTrajectoryTime(cdr.read_type_6());
      	
      data.setExecutionDelayTime(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.GoHomeMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("humanoid_body_part", data.getHumanoidBodyPart());
      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_6("trajectory_time", data.getTrajectoryTime());
      ser.write_type_6("execution_delay_time", data.getExecutionDelayTime());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.GoHomeMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setHumanoidBodyPart(ser.read_type_9("humanoid_body_part"));
      data.setRobotSide(ser.read_type_9("robot_side"));
      data.setTrajectoryTime(ser.read_type_6("trajectory_time"));
      data.setExecutionDelayTime(ser.read_type_6("execution_delay_time"));
   }

   public static void staticCopy(controller_msgs.msg.dds.GoHomeMessage src, controller_msgs.msg.dds.GoHomeMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.GoHomeMessage createData()
   {
      return new controller_msgs.msg.dds.GoHomeMessage();
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
   
   public void serialize(controller_msgs.msg.dds.GoHomeMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.GoHomeMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.GoHomeMessage src, controller_msgs.msg.dds.GoHomeMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public GoHomeMessagePubSubType newInstance()
   {
      return new GoHomeMessagePubSubType();
   }
}
