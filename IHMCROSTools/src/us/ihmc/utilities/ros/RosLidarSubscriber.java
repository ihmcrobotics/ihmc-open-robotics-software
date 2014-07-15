package us.ihmc.utilities.ros;

import java.net.URI;
import java.net.URISyntaxException;

import sensor_msgs.LaserScan;
import us.ihmc.utilities.lidar.polarLidar.LidarScan;
import us.ihmc.utilities.lidar.polarLidar.geometry.LidarScanParameters;

public abstract class RosLidarSubscriber extends AbstractRosTopicSubscriber<sensor_msgs.LaserScan>
{
   private boolean DEBUG = false;
   private LidarScanParameters initialPolarLidarScanParameters = null;

   public RosLidarSubscriber()
   {
      super(sensor_msgs.LaserScan._TYPE);
   }

   public void onNewMessage(LaserScan message)
   {
      LidarScanParameters polarLidarScanParameters = new LidarScanParameters(message.getRanges().length, message.getAngleMin(), message.getAngleMax(),
            message.getTimeIncrement(), message.getRangeMin(), message.getRangeMax(), message.getScanTime(), message.getHeader().getStamp().totalNsecs());

      if (DEBUG)
      {
         verifyDataFromGazeboRemainsTheSame(polarLidarScanParameters);
      }

      LidarScan polarLidarScan = new LidarScan(polarLidarScanParameters, message.getRanges());
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

