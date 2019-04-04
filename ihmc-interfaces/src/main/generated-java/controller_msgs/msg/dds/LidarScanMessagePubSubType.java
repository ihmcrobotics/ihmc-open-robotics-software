package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "LidarScanMessage" defined in "LidarScanMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from LidarScanMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit LidarScanMessage_.idl instead.
*
*/
public class LidarScanMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.LidarScanMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::LidarScanMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.LidarScanMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.LidarScanMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (200000 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.LidarScanMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.LidarScanMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getLidarPosition(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getLidarOrientation(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getScan().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.LidarScanMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_11(data.getRobotTimestamp());

      geometry_msgs.msg.dds.PointPubSubType.write(data.getLidarPosition(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getLidarOrientation(), cdr);
      if(data.getScan().size() <= 200000)
      cdr.write_type_e(data.getScan());else
          throw new RuntimeException("scan field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.LidarScanMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setRobotTimestamp(cdr.read_type_11());
      	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getLidarPosition(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getLidarOrientation(), cdr);	
      cdr.read_type_e(data.getScan());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.LidarScanMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_11("robot_timestamp", data.getRobotTimestamp());
      ser.write_type_a("lidar_position", new geometry_msgs.msg.dds.PointPubSubType(), data.getLidarPosition());

      ser.write_type_a("lidar_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getLidarOrientation());

      ser.write_type_e("scan", data.getScan());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.LidarScanMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setRobotTimestamp(ser.read_type_11("robot_timestamp"));
      ser.read_type_a("lidar_position", new geometry_msgs.msg.dds.PointPubSubType(), data.getLidarPosition());

      ser.read_type_a("lidar_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getLidarOrientation());

      ser.read_type_e("scan", data.getScan());
   }

   public static void staticCopy(controller_msgs.msg.dds.LidarScanMessage src, controller_msgs.msg.dds.LidarScanMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.LidarScanMessage createData()
   {
      return new controller_msgs.msg.dds.LidarScanMessage();
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
   
   public void serialize(controller_msgs.msg.dds.LidarScanMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.LidarScanMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.LidarScanMessage src, controller_msgs.msg.dds.LidarScanMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public LidarScanMessagePubSubType newInstance()
   {
      return new LidarScanMessagePubSubType();
   }
}
