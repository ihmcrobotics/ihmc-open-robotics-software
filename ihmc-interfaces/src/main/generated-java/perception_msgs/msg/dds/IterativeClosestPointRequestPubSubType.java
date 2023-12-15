package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "IterativeClosestPointRequest" defined in "IterativeClosestPointRequest_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from IterativeClosestPointRequest_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit IterativeClosestPointRequest_.idl instead.
*
*/
public class IterativeClosestPointRequestPubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.IterativeClosestPointRequest>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::IterativeClosestPointRequest_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "aec0295651144711562a96d4526e21a51d210745192d03ed5cf6ea817440f326";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.IterativeClosestPointRequest data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.IterativeClosestPointRequest data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.IterativeClosestPointRequest data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.IterativeClosestPointRequest data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getProvidedPose(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.IterativeClosestPointRequest data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_4(data.getNodeId());

      cdr.write_type_9(data.getShape());

      cdr.write_type_5(data.getXLength());

      cdr.write_type_5(data.getYLength());

      cdr.write_type_5(data.getZLength());

      cdr.write_type_5(data.getXRadius());

      cdr.write_type_5(data.getYRadius());

      cdr.write_type_5(data.getZRadius());

      geometry_msgs.msg.dds.PosePubSubType.write(data.getProvidedPose(), cdr);
      cdr.write_type_7(data.getRunIcp());

      cdr.write_type_7(data.getUseProvidedPose());

   }

   public static void read(perception_msgs.msg.dds.IterativeClosestPointRequest data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setNodeId(cdr.read_type_4());
      	
      data.setShape(cdr.read_type_9());
      	
      data.setXLength(cdr.read_type_5());
      	
      data.setYLength(cdr.read_type_5());
      	
      data.setZLength(cdr.read_type_5());
      	
      data.setXRadius(cdr.read_type_5());
      	
      data.setYRadius(cdr.read_type_5());
      	
      data.setZRadius(cdr.read_type_5());
      	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getProvidedPose(), cdr);	
      data.setRunIcp(cdr.read_type_7());
      	
      data.setUseProvidedPose(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.IterativeClosestPointRequest data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_4("node_id", data.getNodeId());
      ser.write_type_9("shape", data.getShape());
      ser.write_type_5("x_length", data.getXLength());
      ser.write_type_5("y_length", data.getYLength());
      ser.write_type_5("z_length", data.getZLength());
      ser.write_type_5("x_radius", data.getXRadius());
      ser.write_type_5("y_radius", data.getYRadius());
      ser.write_type_5("z_radius", data.getZRadius());
      ser.write_type_a("provided_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getProvidedPose());

      ser.write_type_7("run_icp", data.getRunIcp());
      ser.write_type_7("use_provided_pose", data.getUseProvidedPose());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.IterativeClosestPointRequest data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setNodeId(ser.read_type_4("node_id"));
      data.setShape(ser.read_type_9("shape"));
      data.setXLength(ser.read_type_5("x_length"));
      data.setYLength(ser.read_type_5("y_length"));
      data.setZLength(ser.read_type_5("z_length"));
      data.setXRadius(ser.read_type_5("x_radius"));
      data.setYRadius(ser.read_type_5("y_radius"));
      data.setZRadius(ser.read_type_5("z_radius"));
      ser.read_type_a("provided_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getProvidedPose());

      data.setRunIcp(ser.read_type_7("run_icp"));
      data.setUseProvidedPose(ser.read_type_7("use_provided_pose"));
   }

   public static void staticCopy(perception_msgs.msg.dds.IterativeClosestPointRequest src, perception_msgs.msg.dds.IterativeClosestPointRequest dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.IterativeClosestPointRequest createData()
   {
      return new perception_msgs.msg.dds.IterativeClosestPointRequest();
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
   
   public void serialize(perception_msgs.msg.dds.IterativeClosestPointRequest data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.IterativeClosestPointRequest data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.IterativeClosestPointRequest src, perception_msgs.msg.dds.IterativeClosestPointRequest dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public IterativeClosestPointRequestPubSubType newInstance()
   {
      return new IterativeClosestPointRequestPubSubType();
   }
}
