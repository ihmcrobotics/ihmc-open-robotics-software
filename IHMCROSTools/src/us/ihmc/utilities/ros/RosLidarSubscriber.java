package us.ihmc.utilities.ros;

import us.ihmc.utilities.lidar.polarLidar.PolarLidarScan;
import sensor_msgs.LaserScan;
import us.ihmc.utilities.lidar.polarLidar.geometry.PolarLidarScanParameters;

public abstract class RosLidarSubscriber extends AbstractRosTopicSubscriber<sensor_msgs.LaserScan>
{
   private boolean DEBUG = false;
   private PolarLidarScanParameters initialPolarLidarScanParameters = null;

   public RosLidarSubscriber()
   {
      super(sensor_msgs.LaserScan._TYPE);
   }

   public void onNewMessage(LaserScan message)
   {
      PolarLidarScanParameters polarLidarScanParameters = new PolarLidarScanParameters(false, message.getRanges().length, 1, message.getAngleMax(),
                                                             message.getAngleMin(), message.getAngleIncrement(), message.getTimeIncrement(),
                                                             message.getScanTime(), 0.0f, 0.0f, message.getRangeMin(), message.getRangeMax());

      if (DEBUG)
      {
         verifyDataFromGazeboRemainsTheSame(polarLidarScanParameters);
      }

      PolarLidarScan polarLidarScan = new PolarLidarScan(message.getHeader().getStamp().totalNsecs(), message.getRanges(), polarLidarScanParameters);
      newScan(polarLidarScan);
   }

   private void verifyDataFromGazeboRemainsTheSame(PolarLidarScanParameters polarLidarScanParameters)
   {
      verifyLidarScanDefinitionDoesNotChange(polarLidarScanParameters);
      verifyTimeIncrementRemainsZero(polarLidarScanParameters);
   }

   private void verifyTimeIncrementRemainsZero(PolarLidarScanParameters polarLidarScanParameters)
   {
      if (polarLidarScanParameters.timeIncrement != 0.0)
      {
         System.err.println("WARNING: Gazebo LIDAR time increment no longer zero: " + polarLidarScanParameters.timeIncrement);
      }
   }

   private void verifyLidarScanDefinitionDoesNotChange(PolarLidarScanParameters polarLidarScanParameters)
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

   protected abstract void newScan(PolarLidarScan polarLidarScan);

}
