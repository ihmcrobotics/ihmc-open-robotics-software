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
      float[] ranges = message.getRanges();
      float maxRange = message.getRangeMax();
      float minRange = message.getRangeMin();
      PolarLidarScanDefinition lidarScanDefinition = new PolarLidarScanDefinition(ranges.length, 1, message.getAngleMax(), message.getAngleMin(), 0.0f, 0.0f, 0.0f);
      
      newScan(ranges, lidarScanDefinition, minRange, maxRange, message.getTimeIncrement());
      
   }

   protected abstract void newScan(float[] ranges, PolarLidarScanDefinition scanDefinition, double rangeMin, double rangeMax, double timeIncrement);

}
