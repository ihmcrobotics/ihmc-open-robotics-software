package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "AdjustFootstepMessage" defined in "AdjustFootstepMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from AdjustFootstepMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit AdjustFootstepMessage_.idl instead.
*
*/
public class AdjustFootstepMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.AdjustFootstepMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::AdjustFootstepMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.AdjustFootstepMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.AdjustFootstepMessage data) throws java.io.IOException
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


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);}

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.AdjustFootstepMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.AdjustFootstepMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getLocation(), current_alignment);


      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getOrientation(), current_alignment);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getPredictedContactPoints2d().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getPredictedContactPoints2d().get(i0), current_alignment);}


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.AdjustFootstepMessage data, us.ihmc.idl.CDR cdr)
   {

      cdr.write_type_4(data.getSequenceId());


      cdr.write_type_9(data.getRobotSide());


      geometry_msgs.msg.dds.PointPubSubType.write(data.getLocation(), cdr);

      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getOrientation(), cdr);

      if(data.getPredictedContactPoints2d().size() <= 100)
      cdr.write_type_e(data.getPredictedContactPoints2d());else
          throw new RuntimeException("predicted_contact_points_2d field exceeds the maximum length");


      cdr.write_type_6(data.getExecutionDelayTime());

   }

   public static void read(controller_msgs.msg.dds.AdjustFootstepMessage data, us.ihmc.idl.CDR cdr)
   {

      data.setSequenceId(cdr.read_type_4());
      	

      data.setRobotSide(cdr.read_type_9());
      	

      geometry_msgs.msg.dds.PointPubSubType.read(data.getLocation(), cdr);	

      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getOrientation(), cdr);	

      cdr.read_type_e(data.getPredictedContactPoints2d());	

      data.setExecutionDelayTime(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.AdjustFootstepMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {

      ser.write_type_4("sequence_id", data.getSequenceId());

      ser.write_type_9("robot_side", data.getRobotSide());

      ser.write_type_a("location", new geometry_msgs.msg.dds.PointPubSubType(), data.getLocation());


      ser.write_type_a("orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getOrientation());


      ser.write_type_e("predicted_contact_points_2d", data.getPredictedContactPoints2d());

      ser.write_type_6("execution_delay_time", data.getExecutionDelayTime());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.AdjustFootstepMessage data)
   {

      data.setSequenceId(ser.read_type_4("sequence_id"));

      data.setRobotSide(ser.read_type_9("robot_side"));

      ser.read_type_a("location", new geometry_msgs.msg.dds.PointPubSubType(), data.getLocation());


      ser.read_type_a("orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getOrientation());


      ser.read_type_e("predicted_contact_points_2d", data.getPredictedContactPoints2d());

      data.setExecutionDelayTime(ser.read_type_6("execution_delay_time"));
   }

   public static void staticCopy(controller_msgs.msg.dds.AdjustFootstepMessage src, controller_msgs.msg.dds.AdjustFootstepMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.AdjustFootstepMessage createData()
   {
      return new controller_msgs.msg.dds.AdjustFootstepMessage();
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
   
   public void serialize(controller_msgs.msg.dds.AdjustFootstepMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.AdjustFootstepMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.AdjustFootstepMessage src, controller_msgs.msg.dds.AdjustFootstepMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public AdjustFootstepMessagePubSubType newInstance()
   {
      return new AdjustFootstepMessagePubSubType();
   }
}
