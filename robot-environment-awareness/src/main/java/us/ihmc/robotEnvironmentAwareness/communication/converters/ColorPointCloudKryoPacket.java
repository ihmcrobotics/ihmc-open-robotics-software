package us.ihmc.robotEnvironmentAwareness.communication.converters;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;

public class ColorPointCloudKryoPacket
{
   public long sequence_id;
   public long timestamp;
   public double sensor_position_x;
   public double sensor_position_y;
   public double sensor_position_z;

   public double sensor_orientation_qx;
   public double sensor_orientation_qy;
   public double sensor_orientation_qz;
   public double sensor_orientation_qs;

   public double pointcloud_center_x;
   public double pointcloud_center_y;
   public double pointcloud_center_z;

   public double resolution;

   public int number_of_points;

   public byte[] point_cloud, colors;

   public boolean lz4_compressed;

   public ColorPointCloudKryoPacket()
   {
   }

   public void set(StereoVisionPointCloudMessage msg)
   {
      sequence_id = msg.getSequenceId();
      timestamp = msg.getTimestamp();
      sensor_position_x = msg.getSensorPosition().getX();
      sensor_position_y = msg.getSensorPosition().getY();
      sensor_position_z = msg.getSensorPosition().getZ();
      sensor_orientation_qx = msg.getSensorOrientation().getX();
      sensor_orientation_qy = msg.getSensorOrientation().getY();
      sensor_orientation_qz = msg.getSensorOrientation().getZ();
      sensor_orientation_qs = msg.getSensorOrientation().getS();
      pointcloud_center_x = msg.getPointCloudCenter().getX();
      pointcloud_center_y = msg.getPointCloudCenter().getY();
      pointcloud_center_z = msg.getPointCloudCenter().getZ();
      resolution = msg.getResolution();
      number_of_points = msg.getNumberOfPoints();

      if (point_cloud == null || point_cloud.length < msg.getPointCloud().size())
         point_cloud = new byte[msg.getPointCloud().size()];
      msg.getPointCloud().toArray(point_cloud);

      if (colors == null || colors.length < msg.getColors().size())
         colors = new byte[msg.getColors().size()];
      msg.getColors().toArray(colors);

      lz4_compressed = msg.getLz4Compressed();
   }

   public void get(StereoVisionPointCloudMessage msg)
   {
      msg.setSequenceId(sequence_id);
      msg.setTimestamp(timestamp);
      msg.getSensorPosition().set(sensor_position_x, sensor_position_y, sensor_position_z);
      msg.getSensorOrientation().setUnsafe(sensor_orientation_qx, sensor_orientation_qy, sensor_orientation_qz, sensor_orientation_qs);
      msg.getPointCloudCenter().set(pointcloud_center_x, pointcloud_center_y, pointcloud_center_z);
      msg.setResolution(resolution);
      msg.setNumberOfPoints(number_of_points);

      msg.getPointCloud().resetQuick();
      msg.getPointCloud().addAll(point_cloud);
      msg.getColors().resetQuick();
      msg.getColors().addAll(colors);

      msg.setLz4Compressed(lz4_compressed);
   }
}
