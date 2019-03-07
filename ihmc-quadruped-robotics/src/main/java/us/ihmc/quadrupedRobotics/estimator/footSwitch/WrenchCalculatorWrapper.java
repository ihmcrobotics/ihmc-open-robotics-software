package us.ihmc.quadrupedRobotics.estimator.footSwitch;

import us.ihmc.commonWalkingControlModules.touchdownDetector.WrenchCalculator;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.simulationconstructionset.simulatedSensors.WrenchCalculatorInterface;

public class WrenchCalculatorWrapper implements WrenchCalculator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final Wrench measuredWrench = new Wrench();

   private final WrenchCalculatorInterface wrenchCalculatorInterface;
   private final ReferenceFrame measurementFrame;

   public WrenchCalculatorWrapper(WrenchCalculatorInterface wrenchCalculatorInterface, ReferenceFrame measurementFrame)
   {
      this.wrenchCalculatorInterface = wrenchCalculatorInterface;
      this.measurementFrame = measurementFrame;
   }

   @Override
   public void calculate()
   {
      measuredWrench.setToZero(measurementFrame);
      measuredWrench.set(wrenchCalculatorInterface.getWrench());
      measuredWrench.changeFrame(worldFrame);
   }

   @Override
   public WrenchReadOnly getWrench()
   {
      return measuredWrench;
   }

   @Override
   public String getName()
   {
      return wrenchCalculatorInterface.getName();
   }
}