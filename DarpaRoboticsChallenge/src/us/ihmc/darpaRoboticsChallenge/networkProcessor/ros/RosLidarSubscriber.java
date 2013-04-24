package us.ihmc.darpaRoboticsChallenge.networkProcessor.ros;

import sensor_msgs.LaserScan;
import us.ihmc.utilities.polarLidarGeometry.PolarLidarScanDefinition;

public abstract class RosLidarSubscriber extends AbstractRosTopicSubscriber<sensor_msgs.LaserScan>
{
   PolarLidarScanDefinition initialPolarLidarScanDefinition = null;

   public RosLidarSubscriber()
   {
      super(sensor_msgs.LaserScan._TYPE);
   }

   public void onNewMessage(LaserScan message)
   {
      long timestamp = message.getHeader().getStamp().totalNsecs();
      float[] ranges = message.getRanges();

      PolarLidarScanDefinition polarLidarScanDefinition = new PolarLidarScanDefinition(ranges.length, 1, message.getAngleMax(), message.getAngleMin(),
                                                             message.getAngleIncrement(), message.getTimeIncrement(), message.getScanTime(), 0.0f, 0.0f,
                                                             message.getRangeMin(), message.getRangeMax());

      verifyDataFromGazeboRemainsTheSame(polarLidarScanDefinition);

      newScan(timestamp, ranges, polarLidarScanDefinition, message.getRangeMin(), message.getRangeMax(), message.getTimeIncrement());

   }

   private void verifyDataFromGazeboRemainsTheSame(PolarLidarScanDefinition polarLidarScanDefinition)
   {
      verifyLidarScanDefinitionDoesNotChange(polarLidarScanDefinition);
      verifyTimeIncrementRemainsZero(polarLidarScanDefinition);
   }

   private void verifyTimeIncrementRemainsZero(PolarLidarScanDefinition polarLidarScanDefinition)
   {
      if(polarLidarScanDefinition.timeIncrement != 0.0)
      {
         System.err.println("WARNING: Gazebo LIDAR time increment no longer zero: " + polarLidarScanDefinition.timeIncrement);
      }
   }

   private void verifyLidarScanDefinitionDoesNotChange(PolarLidarScanDefinition polarLidarScanDefinition)
   {
      if(initialPolarLidarScanDefinition == null)
      {
         initialPolarLidarScanDefinition = polarLidarScanDefinition;
      }
      else
      {
         if(!polarLidarScanDefinition.equals(initialPolarLidarScanDefinition))
         {
            System.err.println("WARNING: your scan definition has changed");
            System.err.println("old scan definition:\n" + initialPolarLidarScanDefinition);
            System.err.println("new scan definition:\n" + polarLidarScanDefinition);
            initialPolarLidarScanDefinition = polarLidarScanDefinition;
         }
      }
   }

   protected abstract void newScan(long timeStamp, float[] ranges, PolarLidarScanDefinition scanDefinition, double rangeMin, double rangeMax,
                                   double timeIncrement);

}
