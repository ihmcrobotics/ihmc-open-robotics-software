package us.ihmc.utilities.ros.subscriber;

import sensor_msgs.LaserScan;
import us.ihmc.robotics.lidar.LidarScan;
import us.ihmc.robotics.lidar.LidarScanParameters;

public abstract class RosLidarSubscriber extends AbstractRosTopicSubscriber<sensor_msgs.LaserScan>
{
   private boolean DEBUG = false;
   private LidarScanParameters initialPolarLidarScanParameters = null;
   private final int sensorId;

   public RosLidarSubscriber(int sensorId)
   {
      super(sensor_msgs.LaserScan._TYPE);
      this.sensorId = sensorId;
   }

   public void onNewMessage(LaserScan message)
   {
      LidarScanParameters polarLidarScanParameters = new LidarScanParameters(message.getRanges().length, 1, message.getAngleMin(), message.getAngleMax(),
            message.getTimeIncrement(), message.getRangeMin(), message.getRangeMax(), message.getScanTime(), message.getHeader().getStamp().totalNsecs());

      if (DEBUG)
      {
         verifyDataFromGazeboRemainsTheSame(polarLidarScanParameters);
      }

      LidarScan polarLidarScan = new LidarScan(polarLidarScanParameters, message.getRanges(),sensorId);
      newScan(polarLidarScan);
   }

   private void verifyDataFromGazeboRemainsTheSame(LidarScanParameters polarLidarScanParameters)
   {
      verifyLidarScanDefinitionDoesNotChange(polarLidarScanParameters);
      verifyTimeIncrementRemainsZero(polarLidarScanParameters);
   }

   private void verifyTimeIncrementRemainsZero(LidarScanParameters polarLidarScanParameters)
   {
      if (polarLidarScanParameters.timeIncrement != 0.0)
      {
         System.err.println("WARNING: Gazebo LIDAR time increment no longer zero: " + polarLidarScanParameters.timeIncrement);
      }
   }

   private void verifyLidarScanDefinitionDoesNotChange(LidarScanParameters polarLidarScanParameters)
   {
      if (initialPolarLidarScanParameters == null)
      {
         initialPolarLidarScanParameters = polarLidarScanParameters;
      }
      else
      {
         if (!polarLidarScanParameters.equals(initialPolarLidarScanParameters))
         {
            System.err.println("WARNING: your scan definition has changed");
            System.err.println("old scan definition:\n" + initialPolarLidarScanParameters);
            System.err.println("new scan definition:\n" + polarLidarScanParameters);
            initialPolarLidarScanParameters = polarLidarScanParameters;
         }
      }
   }

   protected abstract void newScan(LidarScan polarLidarScan);
   
}

