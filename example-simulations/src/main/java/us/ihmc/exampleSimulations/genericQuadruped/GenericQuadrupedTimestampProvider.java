package us.ihmc.exampleSimulations.genericQuadruped;

import us.ihmc.commons.Conversions;
import us.ihmc.sensorProcessing.sensorProcessors.SensorTimestampHolder;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;

public class GenericQuadrupedTimestampProvider implements SensorTimestampHolder
{
   private final FloatingRootJointRobot sdfRobot;

   public GenericQuadrupedTimestampProvider(FloatingRootJointRobot sdfRobot)
   {
      this.sdfRobot = sdfRobot;
   }
   
   @Override
   public long getWallTime()
   {
      return Conversions.secondsToNanoseconds(sdfRobot.getYoTime().getDoubleValue());
   }

   @Override
   public long getMonotonicTime()
   {
      return getWallTime();
   }

   @Override
   public long getSyncTimestamp()
   {
      return -1;
   }
}
