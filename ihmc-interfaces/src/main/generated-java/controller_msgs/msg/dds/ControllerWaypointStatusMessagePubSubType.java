package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ControllerWaypointStatusMessage" defined in "ControllerWaypointStatusMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ControllerWaypointStatusMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ControllerWaypointStatusMessage_.idl instead.
*
*/
public class ControllerWaypointStatusMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.ControllerWaypointStatusMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::ControllerWaypointStatusMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "bc8941f5b934cbb46b8b12dbde85f9ed97a4c08e566b32f0d1d2f89c8d8c387c";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.ControllerWaypointStatusMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.ControllerWaypointStatusMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ControllerWaypointStatusMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ControllerWaypointStatusMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.ControllerWaypointStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_12(data.getSequenceId());

      cdr.write_type_6(data.getGoalXPosition());

      cdr.write_type_6(data.getGoalYPosition());

      cdr.write_type_6(data.getGoalYaw());

      cdr.write_type_2(data.getWaypointsRemaining());

      cdr.write_type_7(data.getIsStarted());

   }

   public static void read(controller_msgs.msg.dds.ControllerWaypointStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_12());
      	
      data.setGoalXPosition(cdr.read_type_6());
      	
      data.setGoalYPosition(cdr.read_type_6());
      	
      data.setGoalYaw(cdr.read_type_6());
      	
      data.setWaypointsRemaining(cdr.read_type_2());
      	
      data.setIsStarted(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.ControllerWaypointStatusMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_12("sequence_id", data.getSequenceId());
      ser.write_type_6("goal_x_position", data.getGoalXPosition());
      ser.write_type_6("goal_y_position", data.getGoalYPosition());
      ser.write_type_6("goal_yaw", data.getGoalYaw());
      ser.write_type_2("waypoints_remaining", data.getWaypointsRemaining());
      ser.write_type_7("is_started", data.getIsStarted());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.ControllerWaypointStatusMessage data)
   {
      data.setSequenceId(ser.read_type_12("sequence_id"));
      data.setGoalXPosition(ser.read_type_6("goal_x_position"));
      data.setGoalYPosition(ser.read_type_6("goal_y_position"));
      data.setGoalYaw(ser.read_type_6("goal_yaw"));
      data.setWaypointsRemaining(ser.read_type_2("waypoints_remaining"));
      data.setIsStarted(ser.read_type_7("is_started"));
   }

   public static void staticCopy(controller_msgs.msg.dds.ControllerWaypointStatusMessage src, controller_msgs.msg.dds.ControllerWaypointStatusMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.ControllerWaypointStatusMessage createData()
   {
      return new controller_msgs.msg.dds.ControllerWaypointStatusMessage();
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
   
   public void serialize(controller_msgs.msg.dds.ControllerWaypointStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.ControllerWaypointStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.ControllerWaypointStatusMessage src, controller_msgs.msg.dds.ControllerWaypointStatusMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ControllerWaypointStatusMessagePubSubType newInstance()
   {
      return new ControllerWaypointStatusMessagePubSubType();
   }
}
