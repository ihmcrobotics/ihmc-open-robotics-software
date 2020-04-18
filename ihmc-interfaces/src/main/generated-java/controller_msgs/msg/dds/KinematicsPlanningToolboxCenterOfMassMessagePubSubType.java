package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "KinematicsPlanningToolboxCenterOfMassMessage" defined in "KinematicsPlanningToolboxCenterOfMassMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from KinematicsPlanningToolboxCenterOfMassMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit KinematicsPlanningToolboxCenterOfMassMessage_.idl instead.
*
*/
public class KinematicsPlanningToolboxCenterOfMassMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.KinematicsPlanningToolboxCenterOfMassMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::KinematicsPlanningToolboxCenterOfMassMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.KinematicsPlanningToolboxCenterOfMassMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.KinematicsPlanningToolboxCenterOfMassMessage data) throws java.io.IOException
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


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);}

      current_alignment += controller_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      current_alignment += controller_msgs.msg.dds.WeightMatrix3DMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.KinematicsPlanningToolboxCenterOfMassMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.KinematicsPlanningToolboxCenterOfMassMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getWayPointTimes().size() * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getDesiredWayPointPositionsInWorld().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getDesiredWayPointPositionsInWorld().get(i0), current_alignment);}


      current_alignment += controller_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.getCdrSerializedSize(data.getSelectionMatrix(), current_alignment);


      current_alignment += controller_msgs.msg.dds.WeightMatrix3DMessagePubSubType.getCdrSerializedSize(data.getWeights(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.KinematicsPlanningToolboxCenterOfMassMessage data, us.ihmc.idl.CDR cdr)
   {

      cdr.write_type_4(data.getSequenceId());


      if(data.getWayPointTimes().size() <= 100)
      cdr.write_type_e(data.getWayPointTimes());else
          throw new RuntimeException("way_point_times field exceeds the maximum length");


      if(data.getDesiredWayPointPositionsInWorld().size() <= 100)
      cdr.write_type_e(data.getDesiredWayPointPositionsInWorld());else
          throw new RuntimeException("desired_way_point_positions_in_world field exceeds the maximum length");


      controller_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.write(data.getSelectionMatrix(), cdr);

      controller_msgs.msg.dds.WeightMatrix3DMessagePubSubType.write(data.getWeights(), cdr);
   }

   public static void read(controller_msgs.msg.dds.KinematicsPlanningToolboxCenterOfMassMessage data, us.ihmc.idl.CDR cdr)
   {

      data.setSequenceId(cdr.read_type_4());
      	

      cdr.read_type_e(data.getWayPointTimes());	

      cdr.read_type_e(data.getDesiredWayPointPositionsInWorld());	

      controller_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.read(data.getSelectionMatrix(), cdr);	

      controller_msgs.msg.dds.WeightMatrix3DMessagePubSubType.read(data.getWeights(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.KinematicsPlanningToolboxCenterOfMassMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {

      ser.write_type_4("sequence_id", data.getSequenceId());

      ser.write_type_e("way_point_times", data.getWayPointTimes());

      ser.write_type_e("desired_way_point_positions_in_world", data.getDesiredWayPointPositionsInWorld());

      ser.write_type_a("selection_matrix", new controller_msgs.msg.dds.SelectionMatrix3DMessagePubSubType(), data.getSelectionMatrix());


      ser.write_type_a("weights", new controller_msgs.msg.dds.WeightMatrix3DMessagePubSubType(), data.getWeights());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.KinematicsPlanningToolboxCenterOfMassMessage data)
   {

      data.setSequenceId(ser.read_type_4("sequence_id"));

      ser.read_type_e("way_point_times", data.getWayPointTimes());

      ser.read_type_e("desired_way_point_positions_in_world", data.getDesiredWayPointPositionsInWorld());

      ser.read_type_a("selection_matrix", new controller_msgs.msg.dds.SelectionMatrix3DMessagePubSubType(), data.getSelectionMatrix());


      ser.read_type_a("weights", new controller_msgs.msg.dds.WeightMatrix3DMessagePubSubType(), data.getWeights());

   }

   public static void staticCopy(controller_msgs.msg.dds.KinematicsPlanningToolboxCenterOfMassMessage src, controller_msgs.msg.dds.KinematicsPlanningToolboxCenterOfMassMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.KinematicsPlanningToolboxCenterOfMassMessage createData()
   {
      return new controller_msgs.msg.dds.KinematicsPlanningToolboxCenterOfMassMessage();
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
   
   public void serialize(controller_msgs.msg.dds.KinematicsPlanningToolboxCenterOfMassMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.KinematicsPlanningToolboxCenterOfMassMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.KinematicsPlanningToolboxCenterOfMassMessage src, controller_msgs.msg.dds.KinematicsPlanningToolboxCenterOfMassMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public KinematicsPlanningToolboxCenterOfMassMessagePubSubType newInstance()
   {
      return new KinematicsPlanningToolboxCenterOfMassMessagePubSubType();
   }
}
