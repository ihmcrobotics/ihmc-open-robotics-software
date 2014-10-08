package us.ihmc.utilities.ros.publisher;

import java.util.Arrays;

import org.ros.message.Time;

import std_msgs.Header;
import us.ihmc.utilities.lidar.polarLidar.LidarScan;
import us.ihmc.utilities.lidar.polarLidar.geometry.LidarScanParameters;

public class RosLidarPublisher extends RosTopicPublisher<sensor_msgs.LaserScan>
{
   private int seq = 0;

   public RosLidarPublisher(boolean latched)
   {
      super(sensor_msgs.LaserScan._TYPE, latched);
   }
   
   @Override
   public void connected()
   {

   }
   
   
   private float[] fakeIntensities = null;

   public void publish(LidarScan lidarScan, String frameId, Time timestamp)
   {
      sensor_msgs.LaserScan message = getMessage();
      
      Header header = message.getHeader();
      header.setStamp(timestamp);
      header.setFrameId(frameId);
      header.setSeq(seq++);
      message.setHeader(header);
      
      LidarScanParameters parameters = lidarScan.getScanParameters();
      message.setAngleMin(parameters.getSweepYawMin());
      message.setAngleMax(parameters.getSweepYawMax());
      message.setAngleIncrement(parameters.getAngleIncrement());
      message.setTimeIncrement(parameters.getTimeIncrement());
      message.setScanTime(parameters.getScanTime());
      message.setRangeMin(parameters.getMinRange());
      message.setRangeMax(parameters.getMaxRange());
      
      if(fakeIntensities==null || fakeIntensities.length != parameters.getPointsPerSweep())
      {
         fakeIntensities= new float[parameters.getPointsPerSweep()];
      }
      float[] ranges = lidarScan.getRanges();
      for(int i=0;i<ranges.length;i++)
         fakeIntensities[i] = 1/((ranges[i]*ranges[i])+1e-10f);

      message.setIntensities(fakeIntensities);
      
      message.setRanges(lidarScan.getRanges());
      
      publish(message);
   }
}
