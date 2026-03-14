package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "HandContactMessage" defined in "HandContactMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from HandContactMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit HandContactMessage_.idl instead.
*
*/
public class HandContactMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.HandContactMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::HandContactMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "ba1a9390e6fbf25ab7df705f81b0ae813a775669089302bfef510d7418bc3fe9";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.HandContactMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.HandContactMessage data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 18; ++i0)
      {
          current_alignment += ihmc_common_msgs.msg.dds.Point2DMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.HandContactMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.HandContactMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getBracingPoint(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getBracingNormal(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getSupportRegionInMidFeetFrame().size(); ++i0)
      {
          current_alignment += ihmc_common_msgs.msg.dds.Point2DMessagePubSubType.getCdrSerializedSize(data.getSupportRegionInMidFeetFrame().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.HandContactMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_12(data.getSequenceId());

      cdr.write_type_7(data.getLoad());

      cdr.write_type_9(data.getRobotSide());

      cdr.write_type_6(data.getTrajectoryDuration());

      geometry_msgs.msg.dds.PointPubSubType.write(data.getBracingPoint(), cdr);
      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getBracingNormal(), cdr);
      if(data.getSupportRegionInMidFeetFrame().size() <= 18)
      cdr.write_type_e(data.getSupportRegionInMidFeetFrame());else
          throw new RuntimeException("support_region_in_mid_feet_frame field exceeds the maximum length: %d > %d".formatted(data.getSupportRegionInMidFeetFrame().size(), 18));

   }

   public static void read(controller_msgs.msg.dds.HandContactMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_12());
      	
      data.setLoad(cdr.read_type_7());
      	
      data.setRobotSide(cdr.read_type_9());
      	
      data.setTrajectoryDuration(cdr.read_type_6());
      	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getBracingPoint(), cdr);	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getBracingNormal(), cdr);	
      cdr.read_type_e(data.getSupportRegionInMidFeetFrame());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.HandContactMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_12("sequence_id", data.getSequenceId());
      ser.write_type_7("load", data.getLoad());
      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_6("trajectory_duration", data.getTrajectoryDuration());
      ser.write_type_a("bracing_point", new geometry_msgs.msg.dds.PointPubSubType(), data.getBracingPoint());

      ser.write_type_a("bracing_normal", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getBracingNormal());

      ser.write_type_e("support_region_in_mid_feet_frame", data.getSupportRegionInMidFeetFrame());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.HandContactMessage data)
   {
      data.setSequenceId(ser.read_type_12("sequence_id"));
      data.setLoad(ser.read_type_7("load"));
      data.setRobotSide(ser.read_type_9("robot_side"));
      data.setTrajectoryDuration(ser.read_type_6("trajectory_duration"));
      ser.read_type_a("bracing_point", new geometry_msgs.msg.dds.PointPubSubType(), data.getBracingPoint());

      ser.read_type_a("bracing_normal", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getBracingNormal());

      ser.read_type_e("support_region_in_mid_feet_frame", data.getSupportRegionInMidFeetFrame());
   }

   public static void staticCopy(controller_msgs.msg.dds.HandContactMessage src, controller_msgs.msg.dds.HandContactMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.HandContactMessage createData()
   {
      return new controller_msgs.msg.dds.HandContactMessage();
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
   
   public void serialize(controller_msgs.msg.dds.HandContactMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.HandContactMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.HandContactMessage src, controller_msgs.msg.dds.HandContactMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public HandContactMessagePubSubType newInstance()
   {
      return new HandContactMessagePubSubType();
   }
}
