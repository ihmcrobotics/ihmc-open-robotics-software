package us.ihmc.quadrupedRobotics.estimator.footSwitch;

import us.ihmc.commonWalkingControlModules.touchdownDetector.WrenchCalculator;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.scs2.simulation.robot.sensors.SimWrenchSensor;

public class WrenchCalculatorWrapper implements WrenchCalculator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final Wrench measuredWrench = new Wrench();

   private final SimWrenchSensor wrenchSensor;
   private final ReferenceFrame measurementFrame;

   public WrenchCalculatorWrapper(SimWrenchSensor wrenchSensor, ReferenceFrame measurementFrame)
   {
      this.wrenchSensor = wrenchSensor;
      this.measurementFrame = measurementFrame;
   }

   @Override
   public void calculate()
   {
      measuredWrench.setIncludingFrame(wrenchSensor.getWrench());
      measuredWrench.changeFrame(wrenchSensor.getFrame().getRootFrame());
      measuredWrench.setReferenceFrame(worldFrame);
      measuredWrench.setBodyFrame(measurementFrame);
   }

   @Override
   public WrenchReadOnly getWrench()
   {
      return measuredWrench;
   }

   @Override
   public String getName()
   {
      return wrenchSensor.getName();
   }

   @Override
   public boolean isTorquingIntoJointLimit()
   {
      return false;
   }
}