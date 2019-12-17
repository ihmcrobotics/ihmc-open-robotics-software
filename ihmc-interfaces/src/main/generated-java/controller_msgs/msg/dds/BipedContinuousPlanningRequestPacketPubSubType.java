package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "BipedContinuousPlanningRequestPacket" defined in "BipedContinuousPlanningRequestPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from BipedContinuousPlanningRequestPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit BipedContinuousPlanningRequestPacket_.idl instead.
*
*/
public class BipedContinuousPlanningRequestPacketPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.BipedContinuousPlanningRequestPacket>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::BipedContinuousPlanningRequestPacket_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.BipedContinuousPlanningRequestPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.BipedContinuousPlanningRequestPacket data) throws java.io.IOException
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

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.BipedContinuousPlanningRequestPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.BipedContinuousPlanningRequestPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getLeftStartPositionInWorld(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getLeftStartOrientationInWorld(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getRightStartPositionInWorld(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getRightStartOrientationInWorld(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getGoalPositionInWorld(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getGoalOrientationInWorld(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.BipedContinuousPlanningRequestPacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getStartTargetType());

      geometry_msgs.msg.dds.PointPubSubType.write(data.getLeftStartPositionInWorld(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getLeftStartOrientationInWorld(), cdr);
      geometry_msgs.msg.dds.PointPubSubType.write(data.getRightStartPositionInWorld(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getRightStartOrientationInWorld(), cdr);
      geometry_msgs.msg.dds.PointPubSubType.write(data.getGoalPositionInWorld(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getGoalOrientationInWorld(), cdr);
      cdr.write_type_2(data.getPlannerRequestId());

      cdr.write_type_6(data.getTimeout());

      cdr.write_type_6(data.getBestEffortTimeout());

      cdr.write_type_6(data.getHorizonLength());

   }

   public static void read(controller_msgs.msg.dds.BipedContinuousPlanningRequestPacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setStartTargetType(cdr.read_type_9());
      	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getLeftStartPositionInWorld(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getLeftStartOrientationInWorld(), cdr);	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getRightStartPositionInWorld(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getRightStartOrientationInWorld(), cdr);	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getGoalPositionInWorld(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getGoalOrientationInWorld(), cdr);	
      data.setPlannerRequestId(cdr.read_type_2());
      	
      data.setTimeout(cdr.read_type_6());
      	
      data.setBestEffortTimeout(cdr.read_type_6());
      	
      data.setHorizonLength(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.BipedContinuousPlanningRequestPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("start_target_type", data.getStartTargetType());
      ser.write_type_a("left_start_position_in_world", new geometry_msgs.msg.dds.PointPubSubType(), data.getLeftStartPositionInWorld());

      ser.write_type_a("left_start_orientation_in_world", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getLeftStartOrientationInWorld());

      ser.write_type_a("right_start_position_in_world", new geometry_msgs.msg.dds.PointPubSubType(), data.getRightStartPositionInWorld());

      ser.write_type_a("right_start_orientation_in_world", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getRightStartOrientationInWorld());

      ser.write_type_a("goal_position_in_world", new geometry_msgs.msg.dds.PointPubSubType(), data.getGoalPositionInWorld());

      ser.write_type_a("goal_orientation_in_world", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getGoalOrientationInWorld());

      ser.write_type_2("planner_request_id", data.getPlannerRequestId());
      ser.write_type_6("timeout", data.getTimeout());
      ser.write_type_6("best_effort_timeout", data.getBestEffortTimeout());
      ser.write_type_6("horizon_length", data.getHorizonLength());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.BipedContinuousPlanningRequestPacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setStartTargetType(ser.read_type_9("start_target_type"));
      ser.read_type_a("left_start_position_in_world", new geometry_msgs.msg.dds.PointPubSubType(), data.getLeftStartPositionInWorld());

      ser.read_type_a("left_start_orientation_in_world", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getLeftStartOrientationInWorld());

      ser.read_type_a("right_start_position_in_world", new geometry_msgs.msg.dds.PointPubSubType(), data.getRightStartPositionInWorld());

      ser.read_type_a("right_start_orientation_in_world", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getRightStartOrientationInWorld());

      ser.read_type_a("goal_position_in_world", new geometry_msgs.msg.dds.PointPubSubType(), data.getGoalPositionInWorld());

      ser.read_type_a("goal_orientation_in_world", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getGoalOrientationInWorld());

      data.setPlannerRequestId(ser.read_type_2("planner_request_id"));
      data.setTimeout(ser.read_type_6("timeout"));
      data.setBestEffortTimeout(ser.read_type_6("best_effort_timeout"));
      data.setHorizonLength(ser.read_type_6("horizon_length"));
   }

   public static void staticCopy(controller_msgs.msg.dds.BipedContinuousPlanningRequestPacket src, controller_msgs.msg.dds.BipedContinuousPlanningRequestPacket dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.BipedContinuousPlanningRequestPacket createData()
   {
      return new controller_msgs.msg.dds.BipedContinuousPlanningRequestPacket();
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
   
   public void serialize(controller_msgs.msg.dds.BipedContinuousPlanningRequestPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.BipedContinuousPlanningRequestPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.BipedContinuousPlanningRequestPacket src, controller_msgs.msg.dds.BipedContinuousPlanningRequestPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public BipedContinuousPlanningRequestPacketPubSubType newInstance()
   {
      return new BipedContinuousPlanningRequestPacketPubSubType();
   }
}
