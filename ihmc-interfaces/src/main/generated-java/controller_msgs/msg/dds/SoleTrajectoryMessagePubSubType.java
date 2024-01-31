package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "SoleTrajectoryMessage" defined in "SoleTrajectoryMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from SoleTrajectoryMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit SoleTrajectoryMessage_.idl instead.
*
*/
public class SoleTrajectoryMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.SoleTrajectoryMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::SoleTrajectoryMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "1044ae908762ccfce0965ab94993575a99c7d34c39572fbdcf2fe99a4d5f5d00";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.SoleTrajectoryMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.SoleTrajectoryMessage data) throws java.io.IOException
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

      current_alignment += ihmc_common_msgs.msg.dds.EuclideanTrajectoryMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.SoleTrajectoryMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.SoleTrajectoryMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += ihmc_common_msgs.msg.dds.EuclideanTrajectoryMessagePubSubType.getCdrSerializedSize(data.getPositionTrajectory(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.SoleTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getRobotQuadrant());

      ihmc_common_msgs.msg.dds.EuclideanTrajectoryMessagePubSubType.write(data.getPositionTrajectory(), cdr);
   }

   public static void read(controller_msgs.msg.dds.SoleTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setRobotQuadrant(cdr.read_type_9());
      	
      ihmc_common_msgs.msg.dds.EuclideanTrajectoryMessagePubSubType.read(data.getPositionTrajectory(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.SoleTrajectoryMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("robot_quadrant", data.getRobotQuadrant());
      ser.write_type_a("position_trajectory", new ihmc_common_msgs.msg.dds.EuclideanTrajectoryMessagePubSubType(), data.getPositionTrajectory());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.SoleTrajectoryMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setRobotQuadrant(ser.read_type_9("robot_quadrant"));
      ser.read_type_a("position_trajectory", new ihmc_common_msgs.msg.dds.EuclideanTrajectoryMessagePubSubType(), data.getPositionTrajectory());

   }

   public static void staticCopy(controller_msgs.msg.dds.SoleTrajectoryMessage src, controller_msgs.msg.dds.SoleTrajectoryMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.SoleTrajectoryMessage createData()
   {
      return new controller_msgs.msg.dds.SoleTrajectoryMessage();
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
   
   public void serialize(controller_msgs.msg.dds.SoleTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.SoleTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.SoleTrajectoryMessage src, controller_msgs.msg.dds.SoleTrajectoryMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public SoleTrajectoryMessagePubSubType newInstance()
   {
      return new SoleTrajectoryMessagePubSubType();
   }
}
