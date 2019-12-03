package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "VisibilityClusterMessage" defined in "VisibilityClusterMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from VisibilityClusterMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit VisibilityClusterMessage_.idl instead.
*
*/
public class VisibilityClusterMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.VisibilityClusterMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::VisibilityClusterMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.VisibilityClusterMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.VisibilityClusterMessage data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.VisibilityClusterPointsMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 25; ++i0)
      {
          current_alignment += controller_msgs.msg.dds.VisibilityClusterPointsMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 25; ++i0)
      {
          current_alignment += controller_msgs.msg.dds.VisibilityClusterPointsMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += controller_msgs.msg.dds.VisibilityClusterPointsMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.VisibilityClusterPointsMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.VisibilityClusterMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.VisibilityClusterMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getPoseInWorld(), current_alignment);

      current_alignment += controller_msgs.msg.dds.VisibilityClusterPointsMessagePubSubType.getCdrSerializedSize(data.getRawPointsInLocal(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getPreferredNavigableExtrusionsInLocal().size(); ++i0)
      {
          current_alignment += controller_msgs.msg.dds.VisibilityClusterPointsMessagePubSubType.getCdrSerializedSize(data.getPreferredNavigableExtrusionsInLocal().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getPreferredNonNavigableExtrusionsInLocal().size(); ++i0)
      {
          current_alignment += controller_msgs.msg.dds.VisibilityClusterPointsMessagePubSubType.getCdrSerializedSize(data.getPreferredNonNavigableExtrusionsInLocal().get(i0), current_alignment);}

      current_alignment += controller_msgs.msg.dds.VisibilityClusterPointsMessagePubSubType.getCdrSerializedSize(data.getNavigableExtrusionsInLocal(), current_alignment);

      current_alignment += controller_msgs.msg.dds.VisibilityClusterPointsMessagePubSubType.getCdrSerializedSize(data.getNonNavigableExtrusionsInLocal(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.VisibilityClusterMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_9(data.getExtrusionSide());

      cdr.write_type_9(data.getType());

      geometry_msgs.msg.dds.PosePubSubType.write(data.getPoseInWorld(), cdr);
      controller_msgs.msg.dds.VisibilityClusterPointsMessagePubSubType.write(data.getRawPointsInLocal(), cdr);
      if(data.getPreferredNavigableExtrusionsInLocal().size() <= 25)
      cdr.write_type_e(data.getPreferredNavigableExtrusionsInLocal());else
          throw new RuntimeException("preferred_navigable_extrusions_in_local field exceeds the maximum length");

      if(data.getPreferredNonNavigableExtrusionsInLocal().size() <= 25)
      cdr.write_type_e(data.getPreferredNonNavigableExtrusionsInLocal());else
          throw new RuntimeException("preferred_non_navigable_extrusions_in_local field exceeds the maximum length");

      controller_msgs.msg.dds.VisibilityClusterPointsMessagePubSubType.write(data.getNavigableExtrusionsInLocal(), cdr);
      controller_msgs.msg.dds.VisibilityClusterPointsMessagePubSubType.write(data.getNonNavigableExtrusionsInLocal(), cdr);
   }

   public static void read(controller_msgs.msg.dds.VisibilityClusterMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setExtrusionSide(cdr.read_type_9());
      	
      data.setType(cdr.read_type_9());
      	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getPoseInWorld(), cdr);	
      controller_msgs.msg.dds.VisibilityClusterPointsMessagePubSubType.read(data.getRawPointsInLocal(), cdr);	
      cdr.read_type_e(data.getPreferredNavigableExtrusionsInLocal());	
      cdr.read_type_e(data.getPreferredNonNavigableExtrusionsInLocal());	
      controller_msgs.msg.dds.VisibilityClusterPointsMessagePubSubType.read(data.getNavigableExtrusionsInLocal(), cdr);	
      controller_msgs.msg.dds.VisibilityClusterPointsMessagePubSubType.read(data.getNonNavigableExtrusionsInLocal(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.VisibilityClusterMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_9("extrusion_side", data.getExtrusionSide());
      ser.write_type_9("type", data.getType());
      ser.write_type_a("pose_in_world", new geometry_msgs.msg.dds.PosePubSubType(), data.getPoseInWorld());

      ser.write_type_a("raw_points_in_local", new controller_msgs.msg.dds.VisibilityClusterPointsMessagePubSubType(), data.getRawPointsInLocal());

      ser.write_type_e("preferred_navigable_extrusions_in_local", data.getPreferredNavigableExtrusionsInLocal());
      ser.write_type_e("preferred_non_navigable_extrusions_in_local", data.getPreferredNonNavigableExtrusionsInLocal());
      ser.write_type_a("navigable_extrusions_in_local", new controller_msgs.msg.dds.VisibilityClusterPointsMessagePubSubType(), data.getNavigableExtrusionsInLocal());

      ser.write_type_a("non_navigable_extrusions_in_local", new controller_msgs.msg.dds.VisibilityClusterPointsMessagePubSubType(), data.getNonNavigableExtrusionsInLocal());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.VisibilityClusterMessage data)
   {
      data.setExtrusionSide(ser.read_type_9("extrusion_side"));
      data.setType(ser.read_type_9("type"));
      ser.read_type_a("pose_in_world", new geometry_msgs.msg.dds.PosePubSubType(), data.getPoseInWorld());

      ser.read_type_a("raw_points_in_local", new controller_msgs.msg.dds.VisibilityClusterPointsMessagePubSubType(), data.getRawPointsInLocal());

      ser.read_type_e("preferred_navigable_extrusions_in_local", data.getPreferredNavigableExtrusionsInLocal());
      ser.read_type_e("preferred_non_navigable_extrusions_in_local", data.getPreferredNonNavigableExtrusionsInLocal());
      ser.read_type_a("navigable_extrusions_in_local", new controller_msgs.msg.dds.VisibilityClusterPointsMessagePubSubType(), data.getNavigableExtrusionsInLocal());

      ser.read_type_a("non_navigable_extrusions_in_local", new controller_msgs.msg.dds.VisibilityClusterPointsMessagePubSubType(), data.getNonNavigableExtrusionsInLocal());

   }

   public static void staticCopy(controller_msgs.msg.dds.VisibilityClusterMessage src, controller_msgs.msg.dds.VisibilityClusterMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.VisibilityClusterMessage createData()
   {
      return new controller_msgs.msg.dds.VisibilityClusterMessage();
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
   
   public void serialize(controller_msgs.msg.dds.VisibilityClusterMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.VisibilityClusterMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.VisibilityClusterMessage src, controller_msgs.msg.dds.VisibilityClusterMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public VisibilityClusterMessagePubSubType newInstance()
   {
      return new VisibilityClusterMessagePubSubType();
   }
}
