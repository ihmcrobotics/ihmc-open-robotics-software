package us.ihmc.darpaRoboticsChallenge.networkProcessor.ros;

import sensor_msgs.LaserScan;
import us.ihmc.utilities.polarLidarGeometry.PolarLidarScanDefinition;

public abstract class RosLidarSubscriber extends RosTopicSubscriber<sensor_msgs.LaserScan>
{
   public RosLidarSubscriber()
   {
      super(sensor_msgs.LaserScan._TYPE);
   }

   public void onNewMessage(LaserScan message)
   {
      long timestamp = message.getHeader().getStamp().totalNsecs();
      float[] ranges = message.getRanges();
      float maxRange = message.getRangeMax();
      float minRange = message.getRangeMin();
      PolarLidarScanDefinition lidarScanDefinition = new PolarLidarScanDefinition(ranges.length, 1, message.getAngleMax(), message.getAngleMin(), 0.0f, 0.0f, minRange);
      
      newScan(timestamp, ranges, lidarScanDefinition, minRange, maxRange, message.getTimeIncrement());
      
   }

   protected abstract void newScan(long timeStamp, float[] ranges, PolarLidarScanDefinition scanDefinition, double rangeMin, double rangeMax, double timeIncrement);

}
