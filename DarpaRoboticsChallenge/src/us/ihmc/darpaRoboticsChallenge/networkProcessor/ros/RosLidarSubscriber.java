package us.ihmc.darpaRoboticsChallenge.networkProcessor.ros;

import sensor_msgs.LaserScan;
import us.ihmc.utilities.polarLidarGeometry.PolarLidarScanParameters;

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
      long timestamp = message.getHeader().getStamp().totalNsecs();

      PolarLidarScanParameters polarLidarScanParameters = new PolarLidarScanParameters(message.getRanges().length, 1, message.getAngleMax(),
                                                             message.getAngleMin(), message.getAngleIncrement(), message.getTimeIncrement(),
                                                             message.getScanTime(), 0.0f, 0.0f, message.getRangeMin(), message.getRangeMax());

      if (DEBUG)
      {
         verifyDataFromGazeboRemainsTheSame(polarLidarScanParameters);
      }

      newScan(timestamp, message.getRanges(), polarLidarScanParameters, message.getRangeMin(), message.getRangeMax(), message.getTimeIncrement());
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

   protected abstract void newScan(long timeStamp, float[] ranges, PolarLidarScanParameters scanDefinition, double rangeMin, double rangeMax,
                                   double timeIncrement);

}
