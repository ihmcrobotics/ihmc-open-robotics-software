package quadruped_msgs.msg.dds;

/**
* 
* Topic data type of the struct "QuadrupedStepMessage" defined in "QuadrupedStepMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from QuadrupedStepMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit QuadrupedStepMessage_.idl instead.
*
*/
public class QuadrupedStepMessagePubSubType implements us.ihmc.pubsub.TopicDataType<quadruped_msgs.msg.dds.QuadrupedStepMessage>
{
   public static final java.lang.String name = "quadruped_msgs::msg::dds_::QuadrupedStepMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "0b62d6b1db3b046cc6be4b772e4857654792e75c6bed0c2cc78d8b96e3d766fb";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(quadruped_msgs.msg.dds.QuadrupedStepMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, quadruped_msgs.msg.dds.QuadrupedStepMessage data) throws java.io.IOException
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

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(quadruped_msgs.msg.dds.QuadrupedStepMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(quadruped_msgs.msg.dds.QuadrupedStepMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getGoalPosition(), current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(quadruped_msgs.msg.dds.QuadrupedStepMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getRobotQuadrant());

      geometry_msgs.msg.dds.PointPubSubType.write(data.getGoalPosition(), cdr);
      cdr.write_type_6(data.getGroundClearance());

      cdr.write_type_9(data.getTrajectoryType());

   }

   public static void read(quadruped_msgs.msg.dds.QuadrupedStepMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setRobotQuadrant(cdr.read_type_9());
      	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getGoalPosition(), cdr);	
      data.setGroundClearance(cdr.read_type_6());
      	
      data.setTrajectoryType(cdr.read_type_9());
      	

   }

   @Override
   public final void serialize(quadruped_msgs.msg.dds.QuadrupedStepMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("robot_quadrant", data.getRobotQuadrant());
      ser.write_type_a("goal_position", new geometry_msgs.msg.dds.PointPubSubType(), data.getGoalPosition());

      ser.write_type_6("ground_clearance", data.getGroundClearance());
      ser.write_type_9("trajectory_type", data.getTrajectoryType());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, quadruped_msgs.msg.dds.QuadrupedStepMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setRobotQuadrant(ser.read_type_9("robot_quadrant"));
      ser.read_type_a("goal_position", new geometry_msgs.msg.dds.PointPubSubType(), data.getGoalPosition());

      data.setGroundClearance(ser.read_type_6("ground_clearance"));
      data.setTrajectoryType(ser.read_type_9("trajectory_type"));
   }

   public static void staticCopy(quadruped_msgs.msg.dds.QuadrupedStepMessage src, quadruped_msgs.msg.dds.QuadrupedStepMessage dest)
   {
      dest.set(src);
   }

   @Override
   public quadruped_msgs.msg.dds.QuadrupedStepMessage createData()
   {
      return new quadruped_msgs.msg.dds.QuadrupedStepMessage();
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
   
   public void serialize(quadruped_msgs.msg.dds.QuadrupedStepMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(quadruped_msgs.msg.dds.QuadrupedStepMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(quadruped_msgs.msg.dds.QuadrupedStepMessage src, quadruped_msgs.msg.dds.QuadrupedStepMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public QuadrupedStepMessagePubSubType newInstance()
   {
      return new QuadrupedStepMessagePubSubType();
   }
}
