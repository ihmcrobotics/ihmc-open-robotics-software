package quadruped_msgs.msg.dds;

/**
* 
* Topic data type of the struct "QuadrupedFootLoadBearingMessage" defined in "QuadrupedFootLoadBearingMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from QuadrupedFootLoadBearingMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit QuadrupedFootLoadBearingMessage_.idl instead.
*
*/
public class QuadrupedFootLoadBearingMessagePubSubType implements us.ihmc.pubsub.TopicDataType<quadruped_msgs.msg.dds.QuadrupedFootLoadBearingMessage>
{
   public static final java.lang.String name = "quadruped_msgs::msg::dds_::QuadrupedFootLoadBearingMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "f8971562ce8ef7c3d4554c9b220dbf95c9c80cc70f97955df78198d2307e9a68";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(quadruped_msgs.msg.dds.QuadrupedFootLoadBearingMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, quadruped_msgs.msg.dds.QuadrupedFootLoadBearingMessage data) throws java.io.IOException
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


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(quadruped_msgs.msg.dds.QuadrupedFootLoadBearingMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(quadruped_msgs.msg.dds.QuadrupedFootLoadBearingMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(quadruped_msgs.msg.dds.QuadrupedFootLoadBearingMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getRobotQuadrant());

      cdr.write_type_6(data.getExecutionDelayTime());

   }

   public static void read(quadruped_msgs.msg.dds.QuadrupedFootLoadBearingMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setRobotQuadrant(cdr.read_type_9());
      	
      data.setExecutionDelayTime(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(quadruped_msgs.msg.dds.QuadrupedFootLoadBearingMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("robot_quadrant", data.getRobotQuadrant());
      ser.write_type_6("execution_delay_time", data.getExecutionDelayTime());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, quadruped_msgs.msg.dds.QuadrupedFootLoadBearingMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setRobotQuadrant(ser.read_type_9("robot_quadrant"));
      data.setExecutionDelayTime(ser.read_type_6("execution_delay_time"));
   }

   public static void staticCopy(quadruped_msgs.msg.dds.QuadrupedFootLoadBearingMessage src, quadruped_msgs.msg.dds.QuadrupedFootLoadBearingMessage dest)
   {
      dest.set(src);
   }

   @Override
   public quadruped_msgs.msg.dds.QuadrupedFootLoadBearingMessage createData()
   {
      return new quadruped_msgs.msg.dds.QuadrupedFootLoadBearingMessage();
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
   
   public void serialize(quadruped_msgs.msg.dds.QuadrupedFootLoadBearingMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(quadruped_msgs.msg.dds.QuadrupedFootLoadBearingMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(quadruped_msgs.msg.dds.QuadrupedFootLoadBearingMessage src, quadruped_msgs.msg.dds.QuadrupedFootLoadBearingMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public QuadrupedFootLoadBearingMessagePubSubType newInstance()
   {
      return new QuadrupedFootLoadBearingMessagePubSubType();
   }
}
