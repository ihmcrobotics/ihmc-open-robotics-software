package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ControllerWaypointGoalMessage" defined in "ControllerWaypointGoalMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ControllerWaypointGoalMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ControllerWaypointGoalMessage_.idl instead.
*
*/
public class ControllerWaypointGoalMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.ControllerWaypointGoalMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::ControllerWaypointGoalMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "260d0e700c78612e57373d4a7e7109f7d414d5f84cb753a7ec772955664d30d4";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.ControllerWaypointGoalMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.ControllerWaypointGoalMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ControllerWaypointGoalMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ControllerWaypointGoalMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.ControllerWaypointGoalMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_12(data.getSequenceId());

      cdr.write_type_6(data.getXPosition());

      cdr.write_type_6(data.getYPosition());

      cdr.write_type_6(data.getYaw());

      cdr.write_type_6(data.getTimeToReach());

      cdr.write_type_7(data.getHoldPosition());

      cdr.write_type_6(data.getPositionProximity());

      cdr.write_type_6(data.getOrientationProximity());

      cdr.write_type_7(data.getGoalOrientationMatters());

   }

   public static void read(controller_msgs.msg.dds.ControllerWaypointGoalMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_12());
      	
      data.setXPosition(cdr.read_type_6());
      	
      data.setYPosition(cdr.read_type_6());
      	
      data.setYaw(cdr.read_type_6());
      	
      data.setTimeToReach(cdr.read_type_6());
      	
      data.setHoldPosition(cdr.read_type_7());
      	
      data.setPositionProximity(cdr.read_type_6());
      	
      data.setOrientationProximity(cdr.read_type_6());
      	
      data.setGoalOrientationMatters(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.ControllerWaypointGoalMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_12("sequence_id", data.getSequenceId());
      ser.write_type_6("x_position", data.getXPosition());
      ser.write_type_6("y_position", data.getYPosition());
      ser.write_type_6("yaw", data.getYaw());
      ser.write_type_6("time_to_reach", data.getTimeToReach());
      ser.write_type_7("hold_position", data.getHoldPosition());
      ser.write_type_6("position_proximity", data.getPositionProximity());
      ser.write_type_6("orientation_proximity", data.getOrientationProximity());
      ser.write_type_7("goal_orientation_matters", data.getGoalOrientationMatters());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.ControllerWaypointGoalMessage data)
   {
      data.setSequenceId(ser.read_type_12("sequence_id"));
      data.setXPosition(ser.read_type_6("x_position"));
      data.setYPosition(ser.read_type_6("y_position"));
      data.setYaw(ser.read_type_6("yaw"));
      data.setTimeToReach(ser.read_type_6("time_to_reach"));
      data.setHoldPosition(ser.read_type_7("hold_position"));
      data.setPositionProximity(ser.read_type_6("position_proximity"));
      data.setOrientationProximity(ser.read_type_6("orientation_proximity"));
      data.setGoalOrientationMatters(ser.read_type_7("goal_orientation_matters"));
   }

   public static void staticCopy(controller_msgs.msg.dds.ControllerWaypointGoalMessage src, controller_msgs.msg.dds.ControllerWaypointGoalMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.ControllerWaypointGoalMessage createData()
   {
      return new controller_msgs.msg.dds.ControllerWaypointGoalMessage();
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
   
   public void serialize(controller_msgs.msg.dds.ControllerWaypointGoalMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.ControllerWaypointGoalMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.ControllerWaypointGoalMessage src, controller_msgs.msg.dds.ControllerWaypointGoalMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ControllerWaypointGoalMessagePubSubType newInstance()
   {
      return new ControllerWaypointGoalMessagePubSubType();
   }
}
