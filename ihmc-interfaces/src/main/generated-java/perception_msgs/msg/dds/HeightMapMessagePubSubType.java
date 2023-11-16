package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "HeightMapMessage" defined in "HeightMapMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from HeightMapMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit HeightMapMessage_.idl instead.
*
*/
public class HeightMapMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.HeightMapMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::HeightMapMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "02834143142106e8df98428110fd9ed671901225742ce0e986fd45c55959544f";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.HeightMapMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.HeightMapMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (45000 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (45000 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (45000 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 45000; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 45000; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.HeightMapMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.HeightMapMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getKeys().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getHeights().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getVariances().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getCentroids().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getCentroids().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getNormals().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getNormals().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.HeightMapMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_6(data.getXyResolution());

      cdr.write_type_6(data.getGridSizeXy());

      cdr.write_type_6(data.getGridCenterX());

      cdr.write_type_6(data.getGridCenterY());

      cdr.write_type_6(data.getEstimatedGroundHeight());

      if(data.getKeys().size() <= 45000)
      cdr.write_type_e(data.getKeys());else
          throw new RuntimeException("keys field exceeds the maximum length");

      if(data.getHeights().size() <= 45000)
      cdr.write_type_e(data.getHeights());else
          throw new RuntimeException("heights field exceeds the maximum length");

      if(data.getVariances().size() <= 45000)
      cdr.write_type_e(data.getVariances());else
          throw new RuntimeException("variances field exceeds the maximum length");

      if(data.getCentroids().size() <= 45000)
      cdr.write_type_e(data.getCentroids());else
          throw new RuntimeException("centroids field exceeds the maximum length");

      if(data.getNormals().size() <= 45000)
      cdr.write_type_e(data.getNormals());else
          throw new RuntimeException("normals field exceeds the maximum length");

   }

   public static void read(perception_msgs.msg.dds.HeightMapMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setXyResolution(cdr.read_type_6());
      	
      data.setGridSizeXy(cdr.read_type_6());
      	
      data.setGridCenterX(cdr.read_type_6());
      	
      data.setGridCenterY(cdr.read_type_6());
      	
      data.setEstimatedGroundHeight(cdr.read_type_6());
      	
      cdr.read_type_e(data.getKeys());	
      cdr.read_type_e(data.getHeights());	
      cdr.read_type_e(data.getVariances());	
      cdr.read_type_e(data.getCentroids());	
      cdr.read_type_e(data.getNormals());	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.HeightMapMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_6("xy_resolution", data.getXyResolution());
      ser.write_type_6("grid_size_xy", data.getGridSizeXy());
      ser.write_type_6("grid_center_x", data.getGridCenterX());
      ser.write_type_6("grid_center_y", data.getGridCenterY());
      ser.write_type_6("estimated_ground_height", data.getEstimatedGroundHeight());
      ser.write_type_e("keys", data.getKeys());
      ser.write_type_e("heights", data.getHeights());
      ser.write_type_e("variances", data.getVariances());
      ser.write_type_e("centroids", data.getCentroids());
      ser.write_type_e("normals", data.getNormals());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.HeightMapMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setXyResolution(ser.read_type_6("xy_resolution"));
      data.setGridSizeXy(ser.read_type_6("grid_size_xy"));
      data.setGridCenterX(ser.read_type_6("grid_center_x"));
      data.setGridCenterY(ser.read_type_6("grid_center_y"));
      data.setEstimatedGroundHeight(ser.read_type_6("estimated_ground_height"));
      ser.read_type_e("keys", data.getKeys());
      ser.read_type_e("heights", data.getHeights());
      ser.read_type_e("variances", data.getVariances());
      ser.read_type_e("centroids", data.getCentroids());
      ser.read_type_e("normals", data.getNormals());
   }

   public static void staticCopy(perception_msgs.msg.dds.HeightMapMessage src, perception_msgs.msg.dds.HeightMapMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.HeightMapMessage createData()
   {
      return new perception_msgs.msg.dds.HeightMapMessage();
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
   
   public void serialize(perception_msgs.msg.dds.HeightMapMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.HeightMapMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.HeightMapMessage src, perception_msgs.msg.dds.HeightMapMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public HeightMapMessagePubSubType newInstance()
   {
      return new HeightMapMessagePubSubType();
   }
}
