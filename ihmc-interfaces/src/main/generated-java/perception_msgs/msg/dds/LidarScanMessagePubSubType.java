package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "LidarScanMessage" defined in "LidarScanMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from LidarScanMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit LidarScanMessage_.idl instead.
*
*/
public class LidarScanMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.LidarScanMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::LidarScanMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "ebfdbd3f7904af808f9876e4d8950753175e68ac0df246d4aee620d75d0d094e";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.LidarScanMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.LidarScanMessage data) throws java.io.IOException
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

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (2000000 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.LidarScanMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.LidarScanMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getLidarPosition(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getLidarOrientation(), current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getScan().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.LidarScanMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_11(data.getRobotTimestamp());

      geometry_msgs.msg.dds.PointPubSubType.write(data.getLidarPosition(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getLidarOrientation(), cdr);
      cdr.write_type_6(data.getSensorPoseConfidence());

      cdr.write_type_6(data.getPointCloudConfidence());

      cdr.write_type_2(data.getNumberOfPoints());

      if(data.getScan().size() <= 2000000)
      cdr.write_type_e(data.getScan());else
          throw new RuntimeException("scan field exceeds the maximum length");

   }

   public static void read(perception_msgs.msg.dds.LidarScanMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setRobotTimestamp(cdr.read_type_11());
      	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getLidarPosition(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getLidarOrientation(), cdr);	
      data.setSensorPoseConfidence(cdr.read_type_6());
      	
      data.setPointCloudConfidence(cdr.read_type_6());
      	
      data.setNumberOfPoints(cdr.read_type_2());
      	
      cdr.read_type_e(data.getScan());	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.LidarScanMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_11("robot_timestamp", data.getRobotTimestamp());
      ser.write_type_a("lidar_position", new geometry_msgs.msg.dds.PointPubSubType(), data.getLidarPosition());

      ser.write_type_a("lidar_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getLidarOrientation());

      ser.write_type_6("sensor_pose_confidence", data.getSensorPoseConfidence());
      ser.write_type_6("point_cloud_confidence", data.getPointCloudConfidence());
      ser.write_type_2("number_of_points", data.getNumberOfPoints());
      ser.write_type_e("scan", data.getScan());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.LidarScanMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setRobotTimestamp(ser.read_type_11("robot_timestamp"));
      ser.read_type_a("lidar_position", new geometry_msgs.msg.dds.PointPubSubType(), data.getLidarPosition());

      ser.read_type_a("lidar_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getLidarOrientation());

      data.setSensorPoseConfidence(ser.read_type_6("sensor_pose_confidence"));
      data.setPointCloudConfidence(ser.read_type_6("point_cloud_confidence"));
      data.setNumberOfPoints(ser.read_type_2("number_of_points"));
      ser.read_type_e("scan", data.getScan());
   }

   public static void staticCopy(perception_msgs.msg.dds.LidarScanMessage src, perception_msgs.msg.dds.LidarScanMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.LidarScanMessage createData()
   {
      return new perception_msgs.msg.dds.LidarScanMessage();
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
   
   public void serialize(perception_msgs.msg.dds.LidarScanMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.LidarScanMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.LidarScanMessage src, perception_msgs.msg.dds.LidarScanMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public LidarScanMessagePubSubType newInstance()
   {
      return new LidarScanMessagePubSubType();
   }
}
