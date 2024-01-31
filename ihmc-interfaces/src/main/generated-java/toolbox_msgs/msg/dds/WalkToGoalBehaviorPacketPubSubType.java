package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "WalkToGoalBehaviorPacket" defined in "WalkToGoalBehaviorPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from WalkToGoalBehaviorPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit WalkToGoalBehaviorPacket_.idl instead.
*
*/
public class WalkToGoalBehaviorPacketPubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.WalkToGoalBehaviorPacket>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::WalkToGoalBehaviorPacket_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "fc5dde07484e99beb698ccb01f53fc525a3a1938ace7213b3d664bc692319468";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.WalkToGoalBehaviorPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.WalkToGoalBehaviorPacket data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.WalkToGoalBehaviorPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.WalkToGoalBehaviorPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.WalkToGoalBehaviorPacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getWalkToGoalAction());

      cdr.write_type_6(data.getXGoal());

      cdr.write_type_6(data.getYGoal());

      cdr.write_type_6(data.getThetaGoal());

      cdr.write_type_9(data.getGoalRobotSide());

   }

   public static void read(toolbox_msgs.msg.dds.WalkToGoalBehaviorPacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setWalkToGoalAction(cdr.read_type_9());
      	
      data.setXGoal(cdr.read_type_6());
      	
      data.setYGoal(cdr.read_type_6());
      	
      data.setThetaGoal(cdr.read_type_6());
      	
      data.setGoalRobotSide(cdr.read_type_9());
      	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.WalkToGoalBehaviorPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("walk_to_goal_action", data.getWalkToGoalAction());
      ser.write_type_6("x_goal", data.getXGoal());
      ser.write_type_6("y_goal", data.getYGoal());
      ser.write_type_6("theta_goal", data.getThetaGoal());
      ser.write_type_9("goal_robot_side", data.getGoalRobotSide());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.WalkToGoalBehaviorPacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setWalkToGoalAction(ser.read_type_9("walk_to_goal_action"));
      data.setXGoal(ser.read_type_6("x_goal"));
      data.setYGoal(ser.read_type_6("y_goal"));
      data.setThetaGoal(ser.read_type_6("theta_goal"));
      data.setGoalRobotSide(ser.read_type_9("goal_robot_side"));
   }

   public static void staticCopy(toolbox_msgs.msg.dds.WalkToGoalBehaviorPacket src, toolbox_msgs.msg.dds.WalkToGoalBehaviorPacket dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.WalkToGoalBehaviorPacket createData()
   {
      return new toolbox_msgs.msg.dds.WalkToGoalBehaviorPacket();
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
   
   public void serialize(toolbox_msgs.msg.dds.WalkToGoalBehaviorPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.WalkToGoalBehaviorPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.WalkToGoalBehaviorPacket src, toolbox_msgs.msg.dds.WalkToGoalBehaviorPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public WalkToGoalBehaviorPacketPubSubType newInstance()
   {
      return new WalkToGoalBehaviorPacketPubSubType();
   }
}
