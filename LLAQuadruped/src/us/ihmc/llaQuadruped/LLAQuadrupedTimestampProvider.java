package us.ihmc.llaQuadruped;

import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.sensorProcessing.sensorProcessors.SensorTimestampHolder;

public class LLAQuadrupedTimestampProvider implements SensorTimestampHolder
{
   private final FloatingRootJointRobot sdfRobot;

   public LLAQuadrupedTimestampProvider(FloatingRootJointRobot sdfRobot)
   {
      this.sdfRobot = sdfRobot;
   }
   
   @Override
   public long getTimestamp()
   {
      return TimeTools.secondsToNanoSeconds(sdfRobot.getYoTime().getDoubleValue());
   }

   @Override
   public long getVisionSensorTimestamp()
   {
      return getTimestamp();
   }

   @Override
   public long getSensorHeadPPSTimestamp()
   {
      return -1;
   }
}
