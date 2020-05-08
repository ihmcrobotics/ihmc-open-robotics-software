package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "WaypointBasedTrajectoryMessage" defined in "WaypointBasedTrajectoryMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from WaypointBasedTrajectoryMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit WaypointBasedTrajectoryMessage_.idl instead.
*
*/
public class WaypointBasedTrajectoryMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.WaypointBasedTrajectoryMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::WaypointBasedTrajectoryMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.WaypointBasedTrajectoryMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.WaypointBasedTrajectoryMessage data) throws java.io.IOException
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


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);}

      current_alignment += controller_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      current_alignment += controller_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);


      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.WaypointBasedTrajectoryMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.WaypointBasedTrajectoryMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getWaypointTimes().size() * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getWaypoints().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getWaypoints().get(i0), current_alignment);}


      current_alignment += controller_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.getCdrSerializedSize(data.getAngularSelectionMatrix(), current_alignment);


      current_alignment += controller_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.getCdrSerializedSize(data.getLinearSelectionMatrix(), current_alignment);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getControlFramePositionInEndEffector(), current_alignment);


      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getControlFrameOrientationInEndEffector(), current_alignment);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.WaypointBasedTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {

      cdr.write_type_4(data.getSequenceId());


      cdr.write_type_2(data.getEndEffectorHashCode());


      if(data.getWaypointTimes().size() <= 100)
      cdr.write_type_e(data.getWaypointTimes());else
          throw new RuntimeException("waypoint_times field exceeds the maximum length");


      if(data.getWaypoints().size() <= 100)
      cdr.write_type_e(data.getWaypoints());else
          throw new RuntimeException("waypoints field exceeds the maximum length");


      controller_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.write(data.getAngularSelectionMatrix(), cdr);

      controller_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.write(data.getLinearSelectionMatrix(), cdr);

      geometry_msgs.msg.dds.PointPubSubType.write(data.getControlFramePositionInEndEffector(), cdr);

      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getControlFrameOrientationInEndEffector(), cdr);

      cdr.write_type_6(data.getWeight());

   }

   public static void read(controller_msgs.msg.dds.WaypointBasedTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {

      data.setSequenceId(cdr.read_type_4());
      	

      data.setEndEffectorHashCode(cdr.read_type_2());
      	

      cdr.read_type_e(data.getWaypointTimes());	

      cdr.read_type_e(data.getWaypoints());	

      controller_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.read(data.getAngularSelectionMatrix(), cdr);	

      controller_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.read(data.getLinearSelectionMatrix(), cdr);	

      geometry_msgs.msg.dds.PointPubSubType.read(data.getControlFramePositionInEndEffector(), cdr);	

      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getControlFrameOrientationInEndEffector(), cdr);	

      data.setWeight(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.WaypointBasedTrajectoryMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {

      ser.write_type_4("sequence_id", data.getSequenceId());

      ser.write_type_2("end_effector_hash_code", data.getEndEffectorHashCode());

      ser.write_type_e("waypoint_times", data.getWaypointTimes());

      ser.write_type_e("waypoints", data.getWaypoints());

      ser.write_type_a("angular_selection_matrix", new controller_msgs.msg.dds.SelectionMatrix3DMessagePubSubType(), data.getAngularSelectionMatrix());


      ser.write_type_a("linear_selection_matrix", new controller_msgs.msg.dds.SelectionMatrix3DMessagePubSubType(), data.getLinearSelectionMatrix());


      ser.write_type_a("control_frame_position_in_end_effector", new geometry_msgs.msg.dds.PointPubSubType(), data.getControlFramePositionInEndEffector());


      ser.write_type_a("control_frame_orientation_in_end_effector", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getControlFrameOrientationInEndEffector());


      ser.write_type_6("weight", data.getWeight());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.WaypointBasedTrajectoryMessage data)
   {

      data.setSequenceId(ser.read_type_4("sequence_id"));

      data.setEndEffectorHashCode(ser.read_type_2("end_effector_hash_code"));

      ser.read_type_e("waypoint_times", data.getWaypointTimes());

      ser.read_type_e("waypoints", data.getWaypoints());

      ser.read_type_a("angular_selection_matrix", new controller_msgs.msg.dds.SelectionMatrix3DMessagePubSubType(), data.getAngularSelectionMatrix());


      ser.read_type_a("linear_selection_matrix", new controller_msgs.msg.dds.SelectionMatrix3DMessagePubSubType(), data.getLinearSelectionMatrix());


      ser.read_type_a("control_frame_position_in_end_effector", new geometry_msgs.msg.dds.PointPubSubType(), data.getControlFramePositionInEndEffector());


      ser.read_type_a("control_frame_orientation_in_end_effector", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getControlFrameOrientationInEndEffector());


      data.setWeight(ser.read_type_6("weight"));
   }

   public static void staticCopy(controller_msgs.msg.dds.WaypointBasedTrajectoryMessage src, controller_msgs.msg.dds.WaypointBasedTrajectoryMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.WaypointBasedTrajectoryMessage createData()
   {
      return new controller_msgs.msg.dds.WaypointBasedTrajectoryMessage();
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
   
   public void serialize(controller_msgs.msg.dds.WaypointBasedTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.WaypointBasedTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.WaypointBasedTrajectoryMessage src, controller_msgs.msg.dds.WaypointBasedTrajectoryMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public WaypointBasedTrajectoryMessagePubSubType newInstance()
   {
      return new WaypointBasedTrajectoryMessagePubSubType();
   }
}
