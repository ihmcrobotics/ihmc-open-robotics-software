package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.PelvisHeightControlModule;
import us.ihmc.commonWalkingControlModules.controllers.PDController;
import us.ihmc.commonWalkingControlModules.couplingRegistry.CouplingRegistry;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;
import us.ihmc.utilities.screwTheory.Wrench;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.filter.AlphaFilteredYoVariable;

public class SwingLegCompensationPelvisHeightControlModule implements PelvisHeightControlModule
{
   private final boolean DO_STANCE_HEIGHT_CONTROL = false;
   private final ProcessedSensorsInterface processedSensors;
   private final CouplingRegistry couplingRegistry;
   private final CommonWalkingReferenceFrames referenceFrames;

   private final YoVariableRegistry registry = new YoVariableRegistry("PelvisHeightControlModule");
   
   private final double legMass, totalMass;

   private final DoubleYoVariable fZHeight = new DoubleYoVariable("fZHeight", "fZ for stance height control effort", registry);
   private final DoubleYoVariable stanceHeightDes = new DoubleYoVariable("stanceHeightDes", registry);
   private final PDController stanceHeightPDcontroller;
   private final StanceHeightCalculator stanceHeightCalculator;
   
   private final DoubleYoVariable fZStanceLegCompensation = new DoubleYoVariable("fZStanceLegCompensation", registry);

   private final DoubleYoVariable fZSwingLegCompensation = new DoubleYoVariable("fZSwingLegCompensation", registry);
   
   private final DoubleYoVariable fZExtra = new DoubleYoVariable("fZExtra", "Extra fZ to make sure leg is straight during stance", registry);

   private final DoubleYoVariable alphaFz = new DoubleYoVariable("alphaFz", registry);
   private final AlphaFilteredYoVariable fZ = new AlphaFilteredYoVariable("fZ", registry, alphaFz);

   private double controlDT;
   private final Wrench upperBodyWrench = new Wrench();


   public SwingLegCompensationPelvisHeightControlModule(ProcessedSensorsInterface processedSensors, CouplingRegistry couplingRegistry,
           CommonWalkingReferenceFrames referenceFrames, StanceHeightCalculator stanceHeightCalculator, YoVariableRegistry parentRegistry, double controlDT, double legMass, double totalMass)
   {
      this.processedSensors = processedSensors;
      this.couplingRegistry = couplingRegistry;
      this.referenceFrames = referenceFrames;
      this.stanceHeightCalculator = stanceHeightCalculator;
      this.stanceHeightPDcontroller = new PDController("stanceHeight", registry);
      this.controlDT = controlDT;
      this.legMass = legMass;
      this.totalMass = totalMass;

      parentRegistry.addChild(registry);
   }
   
   public double doPelvisHeightControl(double desiredPelvisHeightInWorld, RobotSide supportLeg)
   {
      doStanceHeightControl(supportLeg);
      
      doStanceLegCompensation(supportLeg);

      doSwingLegCompensation(supportLeg);
      
      fZ.update(fZHeight.getDoubleValue() + fZStanceLegCompensation.getDoubleValue() + fZSwingLegCompensation.getDoubleValue() + fZExtra.getDoubleValue());

      return fZ.getDoubleValue();
   }
   
   private void doStanceHeightControl(RobotSide supportLeg)
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
   }

   private void doStanceLegCompensation(RobotSide supportLeg)
   {
      boolean useTotalMass = couplingRegistry.getUpperBodyWrench() == null;
      double massToCompensate = useTotalMass ? totalMass : legMass;
      double gravityZ = processedSensors.getGravityInWorldFrame().getZ();
      fZStanceLegCompensation.set(-gravityZ * massToCompensate);
   }
   
   private void doSwingLegCompensation(RobotSide supportLeg)
   {
      boolean doSwingLegCompensation = couplingRegistry.getUpperBodyWrench() != null;
      if (doSwingLegCompensation)
      {
         upperBodyWrench.set(couplingRegistry.getUpperBodyWrench());
         upperBodyWrench.changeFrame(referenceFrames.getPelvisFrame());
         fZSwingLegCompensation.set(upperBodyWrench.getForce().getZ());
      }
      else
         fZSwingLegCompensation.set(0.0);
   }
   
   public void setParametersForR2()
   {
      stanceHeightPDcontroller.setProportionalGain(1000.0);
      stanceHeightPDcontroller.setDerivativeGain(10.0);

      fZExtra.set(0.0); // 100.0);    // 80.0;
      stanceHeightDes.set(1.03);

      alphaFz.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(2.0, controlDT));
   }

   public void setParametersForM2V2()
   {
      stanceHeightPDcontroller.setProportionalGain(1000.0);
      stanceHeightPDcontroller.setDerivativeGain(10.0);

      fZExtra.set(0.0); // 50.0);    // 0.0);
      stanceHeightDes.set(0.95);

      alphaFz.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(10.0, controlDT));
   }
}
