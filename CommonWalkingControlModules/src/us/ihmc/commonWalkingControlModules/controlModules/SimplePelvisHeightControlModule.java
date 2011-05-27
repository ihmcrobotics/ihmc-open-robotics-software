package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.PelvisHeightControlModule;
import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.PDController;
import com.yobotics.simulationconstructionset.util.math.filter.AlphaFilteredYoVariable;

public class SimplePelvisHeightControlModule implements PelvisHeightControlModule
{
   private final boolean DO_STANCE_HEIGHT_CONTROL = false;
   private final ProcessedSensorsInterface processedSensors;

   private final YoVariableRegistry registry = new YoVariableRegistry("PelvisHeightControlModule");

   private final DoubleYoVariable alphaFz = new DoubleYoVariable("alphaFz", registry);
   private final AlphaFilteredYoVariable fZ = new AlphaFilteredYoVariable("fZ", registry, alphaFz);
   private final DoubleYoVariable fZExtra = new DoubleYoVariable("fZExtra", "Extra fZ to make sure leg is straight during stance", registry);
   private final DoubleYoVariable fZHeight = new DoubleYoVariable("fZHeight", "fZ for stance height control effort", registry);
   private final DoubleYoVariable stanceHeightDes = new DoubleYoVariable("stanceHeightDes", registry);
   private final PDController stanceHeightPDcontroller;

   private final StanceHeightCalculator stanceHeightCalculator;
   private double controlDT;


   public SimplePelvisHeightControlModule(ProcessedSensorsInterface processedSensors, StanceHeightCalculator stanceHeightCalculator,
           YoVariableRegistry parentRegistry, double dt)
   {
      this.processedSensors = processedSensors;
      this.stanceHeightCalculator = stanceHeightCalculator;
      this.stanceHeightPDcontroller = new PDController("stanceHeight", registry);
      this.controlDT = dt;

      parentRegistry.addChild(registry);

   }

   public double doPelvisHeightControl(double desiredPelvisHeightInWorld, RobotSide supportLeg)
   {
      if (DO_STANCE_HEIGHT_CONTROL)
      {
         boolean inDoubleSupport = supportLeg == null;
         double stanceHeight = inDoubleSupport
                               ? stanceHeightCalculator.getStanceHeightUsingBothFeet() : stanceHeightCalculator.getStanceHeightUsingOneFoot(supportLeg);
         fZHeight.set(stanceHeightPDcontroller.compute(stanceHeight, stanceHeightDes.getDoubleValue(), 0.0, 0.0));
      }
      else
         fZHeight.set(0.0);

      double gravityZ = processedSensors.getGravityInWorldFrame().getZ();
      double totalMass = processedSensors.getTotalMass();

      double fZDueToGravity = -gravityZ * totalMass;

      fZ.set(fZDueToGravity + fZExtra.getDoubleValue() + fZHeight.getDoubleValue());

      return fZ.getDoubleValue();
   }

   public void setParametersForR2()
   {
      stanceHeightPDcontroller.setProportionalGain(1000.0);
      stanceHeightPDcontroller.setDerivativeGain(10.0);

      fZExtra.set(100.0);    // 80.0;
      stanceHeightDes.set(1.03);
   }

   public void setParametersForM2V2()
   {
      // TODO: tune
      stanceHeightPDcontroller.setProportionalGain(1000.0);
      stanceHeightPDcontroller.setDerivativeGain(10.0);

      fZExtra.set(50.0);    // 0.0);
      stanceHeightDes.set(0.95);

      alphaFz.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(10.0, controlDT));
   }
}
