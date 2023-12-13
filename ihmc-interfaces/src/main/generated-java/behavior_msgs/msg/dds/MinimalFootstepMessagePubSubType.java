package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "MinimalFootstepMessage" defined in "MinimalFootstepMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from MinimalFootstepMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit MinimalFootstepMessage_.idl instead.
*
*/
public class MinimalFootstepMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.MinimalFootstepMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::MinimalFootstepMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "d3717d0140950b413c698ff4f48b7bdea8f204f24eb59985b987125e548e08a1";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.MinimalFootstepMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.MinimalFootstepMessage data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 16; ++i0)
      {
          current_alignment += ihmc_common_msgs.msg.dds.Point2DMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.MinimalFootstepMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.MinimalFootstepMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getPosition(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getOrientation(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getSupportPolygon().size(); ++i0)
      {
          current_alignment += ihmc_common_msgs.msg.dds.Point2DMessagePubSubType.getCdrSerializedSize(data.getSupportPolygon().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getDescription().length() + 1;


      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.MinimalFootstepMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_9(data.getRobotSide());

      geometry_msgs.msg.dds.PointPubSubType.write(data.getPosition(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getOrientation(), cdr);
      if(data.getSupportPolygon().size() <= 16)
      cdr.write_type_e(data.getSupportPolygon());else
          throw new RuntimeException("support_polygon field exceeds the maximum length");

      if(data.getDescription().length() <= 255)
      cdr.write_type_d(data.getDescription());else
          throw new RuntimeException("description field exceeds the maximum length");

   }

   public static void read(behavior_msgs.msg.dds.MinimalFootstepMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setRobotSide(cdr.read_type_9());
      	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getPosition(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getOrientation(), cdr);	
      cdr.read_type_e(data.getSupportPolygon());	
      cdr.read_type_d(data.getDescription());	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.MinimalFootstepMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_a("position", new geometry_msgs.msg.dds.PointPubSubType(), data.getPosition());

      ser.write_type_a("orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getOrientation());

      ser.write_type_e("support_polygon", data.getSupportPolygon());
      ser.write_type_d("description", data.getDescription());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.MinimalFootstepMessage data)
   {
      data.setRobotSide(ser.read_type_9("robot_side"));
      ser.read_type_a("position", new geometry_msgs.msg.dds.PointPubSubType(), data.getPosition());

      ser.read_type_a("orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getOrientation());

      ser.read_type_e("support_polygon", data.getSupportPolygon());
      ser.read_type_d("description", data.getDescription());
   }

   public static void staticCopy(behavior_msgs.msg.dds.MinimalFootstepMessage src, behavior_msgs.msg.dds.MinimalFootstepMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.MinimalFootstepMessage createData()
   {
      return new behavior_msgs.msg.dds.MinimalFootstepMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.MinimalFootstepMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.MinimalFootstepMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.MinimalFootstepMessage src, behavior_msgs.msg.dds.MinimalFootstepMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public MinimalFootstepMessagePubSubType newInstance()
   {
      return new MinimalFootstepMessagePubSubType();
   }
}
