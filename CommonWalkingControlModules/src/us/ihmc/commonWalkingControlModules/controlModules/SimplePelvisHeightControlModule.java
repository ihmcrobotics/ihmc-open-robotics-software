package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.PelvisHeightControlModule;
import us.ihmc.sensorProcessing.ProcessedSensorsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoUtilities.controllers.PDController;
import us.ihmc.yoUtilities.math.filters.AlphaFilteredYoVariable;


public class SimplePelvisHeightControlModule implements PelvisHeightControlModule
{
   private final boolean doStanceHeightControl;
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
           YoVariableRegistry parentRegistry, double dt, boolean doStanceHeightControl)
   {
      this.processedSensors = processedSensors;
      this.stanceHeightCalculator = stanceHeightCalculator;
      this.stanceHeightPDcontroller = new PDController("stanceHeight", registry);
      this.controlDT = dt;
      this.doStanceHeightControl = doStanceHeightControl;

      parentRegistry.addChild(registry);

   }

   public double doPelvisHeightControl(double desiredStanceHeightInWorld, RobotSide supportLeg)
   {
      if (doStanceHeightControl)
      {
         boolean inDoubleSupport = supportLeg == null;
         double stanceHeight = inDoubleSupport
                               ? stanceHeightCalculator.getStanceHeightUsingBothFeet() : stanceHeightCalculator.getStanceHeightUsingOneFoot(supportLeg);
         stanceHeightDes.set(desiredStanceHeightInWorld);
         fZHeight.set(stanceHeightPDcontroller.compute(stanceHeight, desiredStanceHeightInWorld, 0.0, 0.0));
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
   
   public void setParametersForR2InverseDynamics()
   {
      fZExtra.set(0.0);
      stanceHeightPDcontroller.setProportionalGain(5000.0);
//      stanceHeightPDcontroller.setDerivativeGain(1000.0); // FIXME: doesn't do anything, since the actual CoM velocity is set to zero.
   }

   public void setParametersForM2V2()
   {
      // TODO: tune
      stanceHeightPDcontroller.setProportionalGain(1000.0);
//      stanceHeightPDcontroller.setDerivativeGain(10.0); // FIXME: doesn't do anything, since the actual CoM velocity is set to zero.

      fZExtra.set(50.0);    // 0.0);
      stanceHeightDes.set(0.95);

      alphaFz.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(10.0, controlDT));
   }
}
